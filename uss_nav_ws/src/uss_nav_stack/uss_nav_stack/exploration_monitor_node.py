from __future__ import annotations

import json
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from uss_nav_interfaces.msg import FrontierArray
from visualization_msgs.msg import Marker, MarkerArray

from .gcm import GlobalCoverageMask


@dataclass
class Sample:
    t_sec: float
    coverage_ratio: float
    frontier_count: int


class ExplorationMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("exploration_monitor_node")
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("min_runtime_sec", 20.0)
        self.declare_parameter("min_coverage_ratio", 0.85)
        self.declare_parameter("frontier_threshold", 2)
        self.declare_parameter("no_frontier_hold_sec", 8.0)
        self.declare_parameter("plateau_window_sec", 12.0)
        self.declare_parameter("coverage_plateau_delta", 0.01)
        self.declare_parameter("frontier_plateau_delta", 1.0)
        self.declare_parameter("stop_on_target_found", True)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_topic", "/viz/explore_status")

        self.min_runtime_sec = float(self.get_parameter("min_runtime_sec").value)
        self.min_coverage_ratio = float(self.get_parameter("min_coverage_ratio").value)
        self.frontier_threshold = int(self.get_parameter("frontier_threshold").value)
        self.no_frontier_hold_sec = float(self.get_parameter("no_frontier_hold_sec").value)
        self.plateau_window_sec = float(self.get_parameter("plateau_window_sec").value)
        self.coverage_plateau_delta = float(self.get_parameter("coverage_plateau_delta").value)
        self.frontier_plateau_delta = float(self.get_parameter("frontier_plateau_delta").value)
        self.stop_on_target_found = bool(self.get_parameter("stop_on_target_found").value)
        self.publish_markers = bool(self.get_parameter("publish_markers").value)

        self.metrics_pub = self.create_publisher(String, "/nav/explore_metrics", 10)
        self.done_pub = self.create_publisher(Bool, "/nav/explore_done", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            10,
        )

        self.gcm_sub = self.create_subscription(String, "/nav/gcm_json", self._on_gcm, 10)
        self.frontier_sub = self.create_subscription(FrontierArray, "/nav/frontiers", self._on_frontiers, 10)
        self.target_found_sub = self.create_subscription(Bool, "/nav/target_found", self._on_target_found, 10)

        self.start_t_sec = self._now_sec()
        self.latest_frontier_count = 0
        self.latest_coverage_ratio = 0.0
        self.latest_visited_cells = 0
        self.latest_total_cells = 0
        self.done = False
        self.done_reason = ""
        self.no_frontier_since: Optional[float] = None
        self.history: Deque[Sample] = deque(maxlen=512)
        self.last_header = None

        hz = max(0.2, float(self.get_parameter("publish_hz").value))
        self.timer = self.create_timer(1.0 / hz, self._on_timer)
        self.get_logger().info("exploration_monitor_node started")

    def _now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _on_frontiers(self, msg: FrontierArray) -> None:
        self.latest_frontier_count = len(msg.frontiers)
        self.last_header = msg.header
        now = self._now_sec()
        if self.latest_frontier_count == 0:
            if self.no_frontier_since is None:
                self.no_frontier_since = now
        else:
            self.no_frontier_since = None

    def _on_gcm(self, msg: String) -> None:
        gcm = GlobalCoverageMask.from_json_msg(msg.data)
        visited = sum(1 for v in gcm.visited.values() if v)
        total = len(gcm.unknown_counts)
        self.latest_visited_cells = int(visited)
        self.latest_total_cells = int(total)
        self.latest_coverage_ratio = float(visited / total) if total > 0 else 0.0

    def _on_target_found(self, msg: Bool) -> None:
        if not self.stop_on_target_found:
            return
        if bool(msg.data) and not self.done:
            self.done = True
            self.done_reason = "target_found"
            self.get_logger().info("exploration finished, reason=target_found")

    def _oldest_in_window(self, now_sec: float) -> Optional[Sample]:
        if not self.history:
            return None
        threshold = now_sec - self.plateau_window_sec
        candidate = None
        for item in self.history:
            if item.t_sec <= threshold:
                candidate = item
            else:
                break
        return candidate or self.history[0]

    def _evaluate_done(self, now_sec: float, coverage_delta: float, frontier_delta: float) -> Tuple[bool, str]:
        if self.done:
            return True, self.done_reason
        elapsed = now_sec - self.start_t_sec
        if elapsed < self.min_runtime_sec:
            return False, ""

        if self.latest_coverage_ratio >= self.min_coverage_ratio and self.latest_frontier_count <= self.frontier_threshold:
            return True, "coverage_frontier_threshold"

        if self.latest_frontier_count == 0 and self.no_frontier_since is not None:
            if (now_sec - self.no_frontier_since) >= self.no_frontier_hold_sec:
                return True, "no_frontier_timeout"

        if elapsed >= self.plateau_window_sec:
            coverage_stall = coverage_delta <= self.coverage_plateau_delta
            frontier_stall = abs(frontier_delta) <= self.frontier_plateau_delta
            frontier_low = self.latest_frontier_count <= (self.frontier_threshold + 2)
            if coverage_stall and frontier_stall and frontier_low:
                return True, "coverage_frontier_plateau"

        return False, ""

    def _publish_marker(self, metrics: dict) -> None:
        if not self.publish_markers:
            return
        marker_array = MarkerArray()

        clear = Marker()
        if self.last_header is not None:
            clear.header = self.last_header
        clear.ns = "explore_status_clear"
        clear.id = 0
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        text = Marker()
        if self.last_header is not None:
            text.header = self.last_header
        text.ns = "explore_status"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.orientation.w = 1.0
        text.pose.position.z = 3.2
        text.scale.z = 0.28
        text.color.a = 0.95
        if metrics["done"]:
            text.color.r = 0.1
            text.color.g = 0.95
            text.color.b = 0.2
        else:
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
        text.text = (
            f"coverage={metrics['coverage_ratio']:.3f}, frontiers={metrics['frontier_count']}, "
            f"done={metrics['done']}"
        )
        marker_array.markers.append(text)
        self.marker_pub.publish(marker_array)

    def _on_timer(self) -> None:
        now_sec = self._now_sec()
        self.history.append(
            Sample(
                t_sec=now_sec,
                coverage_ratio=self.latest_coverage_ratio,
                frontier_count=self.latest_frontier_count,
            )
        )
        old = self._oldest_in_window(now_sec)
        if old is None:
            coverage_delta = 0.0
            frontier_delta = 0.0
        else:
            coverage_delta = float(self.latest_coverage_ratio - old.coverage_ratio)
            frontier_delta = float(old.frontier_count - self.latest_frontier_count)

        done_now, reason = self._evaluate_done(now_sec, coverage_delta, frontier_delta)
        if done_now and not self.done:
            self.done = True
            self.done_reason = reason
            self.get_logger().info(f"exploration finished, reason={reason}")

        metrics = {
            "elapsed_sec": float(now_sec - self.start_t_sec),
            "coverage_ratio": float(self.latest_coverage_ratio),
            "visited_cells": int(self.latest_visited_cells),
            "tracked_cells": int(self.latest_total_cells),
            "frontier_count": int(self.latest_frontier_count),
            "coverage_delta_window": float(coverage_delta),
            "frontier_delta_window": float(frontier_delta),
            "done": bool(self.done),
            "done_reason": str(self.done_reason),
        }

        msg = String()
        msg.data = json.dumps(metrics, ensure_ascii=True)
        self.metrics_pub.publish(msg)

        done_msg = Bool()
        done_msg.data = bool(self.done)
        self.done_pub.publish(done_msg)
        self._publish_marker(metrics)


def main() -> None:
    rclpy.init()
    node = ExplorationMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
