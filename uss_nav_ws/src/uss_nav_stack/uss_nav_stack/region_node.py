from __future__ import annotations

from typing import Dict, List, Set

import rclpy
from rclpy.node import Node
from uss_nav_interfaces.msg import RegionLabel, RegionLabelArray, SCGGraph

from .common import SCGEdgeState
from .region import RegionConfig, incremental_region_update, leiden_available


class RegionNode(Node):
    def __init__(self) -> None:
        super().__init__("region_node")
        self.declare_parameter("leiden_resolution", 0.02)
        self.declare_parameter("forced_edge_epsilon", 0.05)
        self.declare_parameter("use_leiden", True)
        self.declare_parameter("min_region_size", 6)
        self.config = RegionConfig(
            leiden_resolution=float(self.get_parameter("leiden_resolution").value),
            forced_edge_epsilon=float(self.get_parameter("forced_edge_epsilon").value),
            use_leiden=bool(self.get_parameter("use_leiden").value),
            min_region_size=int(self.get_parameter("min_region_size").value),
        )
        self.prev_labels: Dict[int, int] = {}
        self.last_node_ids: Set[int] = set()
        self.publisher = self.create_publisher(RegionLabelArray, "/region/labels", 10)
        self.subscription = self.create_subscription(SCGGraph, "/scg/graph", self._on_graph, 10)
        self.get_logger().info("region_node started")
        if self.config.use_leiden and not leiden_available():
            self.get_logger().warn(
                "use_leiden=True but python igraph/leidenalg not available; fallback to networkx modularity"
            )

    def _on_graph(self, msg: SCGGraph) -> None:
        node_ids = [int(node.id) for node in msg.nodes]
        current_set = set(node_ids)
        new_ids = sorted(current_set - self.last_node_ids)
        edges: List[SCGEdgeState] = []
        for edge in msg.edges:
            edges.append(
                SCGEdgeState(
                    from_id=int(edge.from_id),
                    to_id=int(edge.to_id),
                    edge_type=int(edge.edge_type),
                    weight=float(edge.weight),
                )
            )
        self.prev_labels = incremental_region_update(
            node_ids=node_ids,
            edges=edges,
            prev_labels=self.prev_labels,
            new_node_ids=new_ids,
            config=self.config,
        )
        self.last_node_ids = current_set

        out = RegionLabelArray()
        out.header = msg.header
        out.labels = []
        for node_id in node_ids:
            item = RegionLabel()
            item.node_id = int(node_id)
            item.region_id = int(self.prev_labels.get(node_id, -1))
            out.labels.append(item)
        self.publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = RegionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
