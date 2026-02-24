from __future__ import annotations

from typing import List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from uss_nav_interfaces.msg import RollingGrid3D
from visualization_msgs.msg import Marker, MarkerArray

from .gcm import GCMConfig, GlobalCoverageMask
from .rolling_grid import RollingGrid


class GCMNode(Node):
    def __init__(self) -> None:
        super().__init__("gcm_node")
        self.declare_parameter("cell_size", 1.0)
        self.declare_parameter("visited_unknown_threshold", 50)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_topic", "/viz/gcm")
        self.declare_parameter("show_unvisited", False)
        self.declare_parameter("max_markers", 250)
        self.declare_parameter("marker_publish_stride", 5)
        config = GCMConfig(
            cell_size=float(self.get_parameter("cell_size").value),
            visited_unknown_threshold=int(self.get_parameter("visited_unknown_threshold").value),
        )
        self.gcm = GlobalCoverageMask(config=config)
        self.publisher = self.create_publisher(String, "/nav/gcm_json", 10)
        self.publish_markers = bool(self.get_parameter("publish_markers").value)
        self.show_unvisited = bool(self.get_parameter("show_unvisited").value)
        self.max_markers = int(self.get_parameter("max_markers").value)
        self.marker_publish_stride = max(1, int(self.get_parameter("marker_publish_stride").value))
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            10,
        )
        self._cb_count = 0
        self.subscription = self.create_subscription(
            RollingGrid3D,
            "/map/rolling_grid",
            self._on_grid,
            10,
        )
        self.get_logger().info("gcm_node started")

    def _cidx_center(self, cidx: Tuple[int, int, int]) -> Tuple[float, float, float]:
        cs = float(self.gcm.config.cell_size)
        return (cidx[0] + 0.5) * cs, (cidx[1] + 0.5) * cs, (cidx[2] + 0.5) * cs

    def _build_markers(self, msg: RollingGrid3D) -> MarkerArray:
        out = MarkerArray()

        clear = Marker()
        clear.header = msg.header
        clear.ns = "gcm_clear"
        clear.id = 0
        clear.action = Marker.DELETEALL
        out.markers.append(clear)

        marker_id = 1
        visited_items: List[Tuple[Tuple[int, int, int], int]] = []
        for cidx, unknown in self.gcm.unknown_counts.items():
            if self.gcm.visited.get(cidx, False):
                visited_items.append((cidx, int(unknown)))

        visited_items = sorted(visited_items, key=lambda x: x[1])[: self.max_markers]
        for cidx, _ in visited_items:
            marker = Marker()
            marker.header = msg.header
            marker.ns = "gcm_visited"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.scale.x = float(self.gcm.config.cell_size) * 0.9
            marker.scale.y = float(self.gcm.config.cell_size) * 0.9
            marker.scale.z = float(self.gcm.config.cell_size) * 0.9
            cx, cy, cz = self._cidx_center(cidx)
            marker.pose.position.x = cx
            marker.pose.position.y = cy
            marker.pose.position.z = cz
            marker.pose.orientation.w = 1.0
            marker.color.a = 0.25
            marker.color.r = 0.1
            marker.color.g = 0.9
            marker.color.b = 0.2
            out.markers.append(marker)

        if self.show_unvisited and marker_id <= self.max_markers:
            unvisited = [
                (cidx, int(unknown))
                for cidx, unknown in self.gcm.unknown_counts.items()
                if not self.gcm.visited.get(cidx, False)
            ]
            unvisited = sorted(unvisited, key=lambda x: -x[1])[: max(0, self.max_markers - marker_id + 1)]
            for cidx, _ in unvisited:
                marker = Marker()
                marker.header = msg.header
                marker.ns = "gcm_unvisited"
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.scale.x = float(self.gcm.config.cell_size) * 0.8
                marker.scale.y = float(self.gcm.config.cell_size) * 0.8
                marker.scale.z = float(self.gcm.config.cell_size) * 0.8
                cx, cy, cz = self._cidx_center(cidx)
                marker.pose.position.x = cx
                marker.pose.position.y = cy
                marker.pose.position.z = cz
                marker.pose.orientation.w = 1.0
                marker.color.a = 0.08
                marker.color.r = 0.9
                marker.color.g = 0.1
                marker.color.b = 0.1
                out.markers.append(marker)

        summary = Marker()
        summary.header = msg.header
        summary.ns = "gcm_text"
        summary.id = marker_id
        summary.type = Marker.TEXT_VIEW_FACING
        summary.action = Marker.ADD
        summary.pose.orientation.w = 1.0
        summary.pose.position.z = 2.5
        summary.scale.z = 0.3
        summary.color.a = 0.95
        summary.color.r = 1.0
        summary.color.g = 1.0
        summary.color.b = 1.0
        visited_count = sum(1 for v in self.gcm.visited.values() if v)
        total_count = len(self.gcm.unknown_counts)
        summary.text = f"GCM visited cells: {visited_count}/{total_count}"
        out.markers.append(summary)
        return out

    def _on_grid(self, msg: RollingGrid3D) -> None:
        grid = RollingGrid.from_msg(msg)
        self.gcm.update_from_grid(grid)
        payload = String()
        payload.data = self.gcm.to_json_msg()
        self.publisher.publish(payload)
        self._cb_count += 1
        if self.publish_markers and (self._cb_count % self.marker_publish_stride == 0):
            self.marker_pub.publish(self._build_markers(msg))


def main() -> None:
    rclpy.init()
    node = GCMNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
