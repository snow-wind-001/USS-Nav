from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from uss_nav_interfaces.msg import ObjectGraph, SCGGraph

from .object_fusion import ObjectFusionConfig, ObjectGraphMemory


class ObjectGraphNode(Node):
    def __init__(self) -> None:
        super().__init__("object_graph_node")
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("enable_synthetic", True)
        self.declare_parameter("synthetic_labels", ["chair", "sofa", "table", "bed"])
        self.declare_parameter("clip_dim", 512)
        self.declare_parameter("random_seed", 17)

        self.enable_synthetic = bool(self.get_parameter("enable_synthetic").value)
        self.labels = [str(x) for x in self.get_parameter("synthetic_labels").value]
        self.clip_dim = int(self.get_parameter("clip_dim").value)
        self.rng = np.random.default_rng(int(self.get_parameter("random_seed").value))
        self.clip_templates: Dict[str, np.ndarray] = {}
        self.latest_nodes: List[Tuple[int, np.ndarray]] = []

        self.memory = ObjectGraphMemory(config=ObjectFusionConfig())
        self.publisher = self.create_publisher(ObjectGraph, "/objects/graph", 10)
        self.sub = self.create_subscription(SCGGraph, "/scg/graph", self._on_scg, 10)
        hz = max(float(self.get_parameter("publish_hz").value), 0.2)
        self.timer = self.create_timer(1.0 / hz, self._on_timer)
        self.last_header = None
        self.get_logger().info("object_graph_node started")

    def _on_scg(self, msg: SCGGraph) -> None:
        self.last_header = msg.header
        nodes = []
        for node in msg.nodes:
            nodes.append((int(node.id), np.array([node.centroid.x, node.centroid.y, node.centroid.z], dtype=np.float32)))
        self.latest_nodes = nodes

    def _clip_template(self, label: str) -> np.ndarray:
        if label not in self.clip_templates:
            vec = self.rng.normal(size=(self.clip_dim,)).astype(np.float32)
            vec = vec / (np.linalg.norm(vec) + 1e-8)
            self.clip_templates[label] = vec
        return self.clip_templates[label]

    def _on_timer(self) -> None:
        if not self.enable_synthetic or not self.latest_nodes:
            return
        node_id, centroid = self.latest_nodes[int(self.rng.integers(0, len(self.latest_nodes)))]
        label = self.labels[int(self.rng.integers(0, len(self.labels)))]
        clip_vec = self._clip_template(label) + 0.02 * self.rng.normal(size=(self.clip_dim,)).astype(np.float32)
        points = centroid[None, :] + self.rng.normal(scale=[0.2, 0.2, 0.1], size=(120, 3)).astype(np.float32)
        self.memory.update_observation(label=label, clip_vec=clip_vec, points_xyz=points, anchor_scg_node_id=node_id)

        if self.last_header is None:
            stamp = self.get_clock().now().to_msg()
            frame_id = "map"
        else:
            stamp = self.last_header.stamp
            frame_id = self.last_header.frame_id
        self.publisher.publish(self.memory.to_msg(stamp, frame_id))


def main() -> None:
    rclpy.init()
    node = ObjectGraphNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
