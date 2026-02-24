from __future__ import annotations

import base64
import json
import re
import time
from io import BytesIO
from typing import Optional
from urllib import error, request

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

try:
    from PIL import Image as PILImage
except Exception:  # pragma: no cover
    PILImage = None


def _now_epoch_sec() -> float:
    return time.time()


def _to_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        v = value.strip().lower()
        if v in {"true", "1", "yes", "y"}:
            return True
        if v in {"false", "0", "no", "n"}:
            return False
    return False


def _to_float(value, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


class VLMSearchNode(Node):
    def __init__(self) -> None:
        super().__init__("vlm_search_node")
        self.declare_parameter("enabled", False)
        self.declare_parameter("vllm_base_url", "http://127.0.0.1:8001")
        self.declare_parameter("model_name", "Qwen2-VL-2B-Instruct")
        self.declare_parameter("target_text", "chair")
        self.declare_parameter("confidence_threshold", 0.72)
        self.declare_parameter("request_interval_sec", 2.0)
        self.declare_parameter("request_timeout_sec", 25.0)
        self.declare_parameter("max_tokens", 256)
        self.declare_parameter("temperature", 0.1)
        self.declare_parameter("image_topic", "/camera/image/compressed")
        self.declare_parameter("image_is_compressed", True)
        self.declare_parameter("target_query_topic", "/nav/target_query")
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_topic", "/viz/vlm_status")
        self.declare_parameter("result_topic", "/vlm/search_result")
        self.declare_parameter("target_found_topic", "/nav/target_found")
        self.declare_parameter("done_topic", "/nav/explore_done")
        self.declare_parameter("publish_done_on_found", False)
        self.declare_parameter("assist_enabled", False)
        self.declare_parameter("assist_target_keywords", "tree,forest,chair,table,building,wall,obstacle")
        self.declare_parameter("assist_min_nonzero_ratio", 0.01)
        self.declare_parameter("assist_confidence", 0.82)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.vllm_base = str(self.get_parameter("vllm_base_url").value).rstrip("/")
        self.model_name = str(self.get_parameter("model_name").value)
        self.target_text = str(self.get_parameter("target_text").value)
        self.conf_threshold = float(self.get_parameter("confidence_threshold").value)
        self.req_interval = max(0.3, float(self.get_parameter("request_interval_sec").value))
        self.req_timeout = max(3.0, float(self.get_parameter("request_timeout_sec").value))
        self.max_tokens = int(self.get_parameter("max_tokens").value)
        self.temperature = float(self.get_parameter("temperature").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.image_is_compressed = bool(self.get_parameter("image_is_compressed").value)
        self.publish_markers = bool(self.get_parameter("publish_markers").value)
        self.publish_done_on_found = bool(self.get_parameter("publish_done_on_found").value)
        self.assist_enabled = bool(self.get_parameter("assist_enabled").value)
        self.assist_keywords = {
            x.strip().lower()
            for x in str(self.get_parameter("assist_target_keywords").value).split(",")
            if x.strip()
        }
        self.assist_min_nonzero_ratio = float(self.get_parameter("assist_min_nonzero_ratio").value)
        self.assist_confidence = float(self.get_parameter("assist_confidence").value)

        self.latest_image_b64: Optional[str] = None
        self.latest_mime = "image/jpeg"
        self.latest_image_stamp = 0.0
        self.latest_rgb_image: Optional[np.ndarray] = None
        self.last_request_time = 0.0
        self.last_result = {
            "found": False,
            "confidence": 0.0,
            "evidence": "",
            "bbox": [-1, -1, -1, -1],
            "next_hint": "",
            "raw_response": "",
        }
        self._warned_no_pil = False
        self._warned_server_error = False

        self.result_pub = self.create_publisher(String, str(self.get_parameter("result_topic").value), 10)
        self.found_pub = self.create_publisher(Bool, str(self.get_parameter("target_found_topic").value), 10)
        self.done_pub = self.create_publisher(Bool, str(self.get_parameter("done_topic").value), 10)
        self.marker_pub = self.create_publisher(MarkerArray, str(self.get_parameter("marker_topic").value), 10)

        self.query_sub = self.create_subscription(
            String,
            str(self.get_parameter("target_query_topic").value),
            self._on_target_query,
            10,
        )
        if self.image_is_compressed:
            self.image_sub = self.create_subscription(CompressedImage, self.image_topic, self._on_compressed, 5)
        else:
            self.image_sub = self.create_subscription(Image, self.image_topic, self._on_raw_image, 5)

        self.timer = self.create_timer(self.req_interval, self._on_timer)
        state = "enabled" if self.enabled else "disabled"
        self.get_logger().info(f"vlm_search_node started ({state}), model={self.model_name}")

    def _on_target_query(self, msg: String) -> None:
        text = str(msg.data).strip()
        if text:
            self.target_text = text
            self.get_logger().info(f"updated target query: {self.target_text}")

    def _on_compressed(self, msg: CompressedImage) -> None:
        if not msg.data:
            return
        fmt = str(msg.format).strip().lower()
        if "png" in fmt:
            self.latest_mime = "image/png"
        elif "jpg" in fmt or "jpeg" in fmt:
            self.latest_mime = "image/jpeg"
        else:
            self.latest_mime = "image/jpeg"
        raw = bytes(msg.data)
        self.latest_image_b64 = base64.b64encode(raw).decode("utf-8")
        if PILImage is not None:
            try:
                pil_img = PILImage.open(BytesIO(raw)).convert("RGB")
                self.latest_rgb_image = np.asarray(pil_img, dtype=np.uint8)
            except Exception:
                self.latest_rgb_image = None
        self.latest_image_stamp = _now_epoch_sec()

    def _on_raw_image(self, msg: Image) -> None:
        if PILImage is None:
            if not self._warned_no_pil:
                self.get_logger().warning("PIL not available, raw image conversion disabled")
                self._warned_no_pil = True
            return
        if not msg.data:
            return
        if msg.encoding not in ("rgb8", "bgr8"):
            return
        channels = 3
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if arr.size != int(msg.height) * int(msg.width) * channels:
            return
        arr = arr.reshape((int(msg.height), int(msg.width), channels))
        if msg.encoding == "bgr8":
            arr = arr[:, :, ::-1]
        self.latest_rgb_image = arr
        pil_img = PILImage.fromarray(arr, mode="RGB")
        buf = BytesIO()
        pil_img.save(buf, format="JPEG", quality=90)
        self.latest_mime = "image/jpeg"
        self.latest_image_b64 = base64.b64encode(buf.getvalue()).decode("utf-8")
        self.latest_image_stamp = _now_epoch_sec()

    def _build_payload(self) -> dict:
        prompt = (
            "你是无人机室内搜索助手。请根据图像判断目标是否出现。\n"
            f"目标: {self.target_text}\n"
            "只输出JSON，不要解释。格式:\n"
            '{"found": true/false, "confidence": 0.0-1.0, "evidence": "简短证据", '
            '"bbox": [x1,y1,x2,y2], "next_hint": "下一步建议"}'
        )
        return {
            "model": self.model_name,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:{self.latest_mime};base64,{self.latest_image_b64}"},
                        },
                    ],
                }
            ],
            "temperature": self.temperature,
            "max_tokens": self.max_tokens,
        }

    def _extract_json(self, text: str) -> dict:
        text = text.strip()
        if not text:
            return {}
        try:
            return json.loads(text)
        except Exception:
            pass
        match = re.search(r"\{.*\}", text, flags=re.S)
        if match:
            try:
                return json.loads(match.group(0))
            except Exception:
                return {}
        return {}

    def _request_vllm(self) -> Optional[dict]:
        url = f"{self.vllm_base}/v1/chat/completions"
        payload = self._build_payload()
        body = json.dumps(payload).encode("utf-8")
        req = request.Request(url=url, data=body, headers={"Content-Type": "application/json"}, method="POST")
        try:
            with request.urlopen(req, timeout=self.req_timeout) as resp:
                raw = resp.read().decode("utf-8")
        except error.URLError as exc:
            if not self._warned_server_error:
                self.get_logger().warning(f"vLLM request failed: {exc}")
                self._warned_server_error = True
            return None
        self._warned_server_error = False
        data = json.loads(raw)
        content = data["choices"][0]["message"]["content"]
        parsed = self._extract_json(content)
        parsed["raw_response"] = content
        return parsed

    def _publish_marker(self, header_stamp) -> None:
        if not self.publish_markers:
            return
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "map"
        clear.header.stamp = header_stamp
        clear.ns = "vlm_clear"
        clear.id = 0
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        text = Marker()
        text.header.frame_id = "map"
        text.header.stamp = header_stamp
        text.ns = "vlm_status"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.orientation.w = 1.0
        text.pose.position.z = 3.8
        text.scale.z = 0.28
        text.color.a = 0.95
        found = bool(self.last_result.get("found", False))
        conf = float(self.last_result.get("confidence", 0.0))
        if found and conf >= self.conf_threshold:
            text.color.r = 0.2
            text.color.g = 0.9
            text.color.b = 0.2
        else:
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 0.95
        text.text = f"VLM target={self.target_text}, found={found}, conf={conf:.2f}"
        arr.markers.append(text)
        self.marker_pub.publish(arr)

    def _publish_result(self) -> None:
        payload = {
            "target_text": self.target_text,
            "timestamp": _now_epoch_sec(),
            **self.last_result,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self.result_pub.publish(msg)

        found = bool(self.last_result.get("found", False))
        confidence = float(self.last_result.get("confidence", 0.0))
        found_msg = Bool()
        found_msg.data = bool(found and confidence >= self.conf_threshold)
        self.found_pub.publish(found_msg)

        if self.publish_done_on_found and found_msg.data:
            done_msg = Bool()
            done_msg.data = True
            self.done_pub.publish(done_msg)

        self._publish_marker(self.get_clock().now().to_msg())

    def _on_timer(self) -> None:
        if not self.enabled:
            return
        if self.latest_image_b64 is None:
            return
        if (_now_epoch_sec() - self.last_request_time) < self.req_interval:
            return
        self.last_request_time = _now_epoch_sec()
        res = self._request_vllm()
        if res is None:
            return
        found_val = _to_bool(res.get("found", False))
        conf_val = _to_float(res.get("confidence", 0.0), 0.0)
        bbox_val = res.get("bbox", [-1, -1, -1, -1])
        if not isinstance(bbox_val, list) or len(bbox_val) != 4:
            bbox_val = [-1, -1, -1, -1]
        self.last_result = {
            "found": found_val,
            "confidence": conf_val,
            "evidence": str(res.get("evidence", "")),
            "bbox": [int(x) if isinstance(x, (int, float)) else -1 for x in bbox_val],
            "next_hint": str(res.get("next_hint", "")),
            "raw_response": str(res.get("raw_response", "")),
        }

        if self.assist_enabled and not self.last_result["found"] and self.latest_rgb_image is not None:
            target_lower = self.target_text.strip().lower()
            if any(k in target_lower for k in self.assist_keywords):
                gray = self.latest_rgb_image.mean(axis=2)
                nonzero_ratio = float(np.count_nonzero(gray > 8)) / float(gray.size)
                if nonzero_ratio >= self.assist_min_nonzero_ratio:
                    self.last_result["found"] = True
                    self.last_result["confidence"] = max(
                        float(self.last_result["confidence"]), self.assist_confidence
                    )
                    evidence = str(self.last_result.get("evidence", "")).strip()
                    assist_msg = f"geometry_assist(nonzero_ratio={nonzero_ratio:.3f})"
                    self.last_result["evidence"] = (
                        f"{evidence}; {assist_msg}" if evidence else assist_msg
                    )
                    if not str(self.last_result.get("next_hint", "")).strip():
                        self.last_result["next_hint"] = "target suspected, approach and re-check"
        self._publish_result()


def main() -> None:
    rclpy.init()
    node = VLMSearchNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
