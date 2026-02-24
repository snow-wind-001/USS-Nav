from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import String
from uss_nav_interfaces.msg import Frontier, FrontierArray, RollingGrid3D
from visualization_msgs.msg import Marker, MarkerArray

from .frontier import FrontierConfig, extract_frontiers
from .gcm import GlobalCoverageMask
from .rolling_grid import RollingGrid


class FrontierNode(Node):
    def __init__(self) -> None:
        super().__init__("frontier_node")
        self.declare_parameter("min_cluster_size", 5)
        self.declare_parameter("cluster_eps", 0.5)  # reserved for later DBSCAN upgrade
        self.declare_parameter("info_gain_radius", 1.0)  # reserved for richer gain model
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_topic", "/viz/frontiers")
        self.declare_parameter("point_scale", 0.15)

        self.config = FrontierConfig(
            min_cluster_size=int(self.get_parameter("min_cluster_size").value),
        )
        self.latest_gcm: GlobalCoverageMask | None = None
        self.publisher = self.create_publisher(FrontierArray, "/nav/frontiers", 10)
        self.publish_markers = bool(self.get_parameter("publish_markers").value)
        self.point_scale = float(self.get_parameter("point_scale").value)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            10,
        )
        self.gcm_sub = self.create_subscription(String, "/nav/gcm_json", self._on_gcm, 10)
        self.grid_sub = self.create_subscription(RollingGrid3D, "/map/rolling_grid", self._on_grid, 10)
        self.get_logger().info("frontier_node started")

    def _on_gcm(self, msg: String) -> None:
        self.latest_gcm = GlobalCoverageMask.from_json_msg(msg.data)

    def _build_markers(self, msg: RollingGrid3D, out_frontiers: FrontierArray) -> MarkerArray:
        markers = MarkerArray()

        clear = Marker()
        clear.header = msg.header
        clear.ns = "frontier_clear"
        clear.id = 0
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        points = Marker()
        points.header = msg.header
        points.ns = "frontiers"
        points.id = 1
        points.type = Marker.SPHERE_LIST
        points.action = Marker.ADD
        points.pose.orientation.w = 1.0
        points.scale.x = self.point_scale
        points.scale.y = self.point_scale
        points.scale.z = self.point_scale
        points.color.a = 0.95
        points.color.r = 1.0
        points.color.g = 0.55
        points.color.b = 0.1
        points.points = []
        for f in out_frontiers.frontiers:
            p = Point()
            p.x = float(f.position.x)
            p.y = float(f.position.y)
            p.z = float(f.position.z)
            points.points.append(p)
        markers.markers.append(points)

        text = Marker()
        text.header = msg.header
        text.ns = "frontier_text"
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.orientation.w = 1.0
        text.pose.position.z = 2.8
        text.scale.z = 0.3
        text.color.a = 0.95
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.text = f"Frontiers: {len(out_frontiers.frontiers)}"
        markers.markers.append(text)

        return markers

    def _on_grid(self, msg: RollingGrid3D) -> None:
        grid = RollingGrid.from_msg(msg)
        frontiers = extract_frontiers(grid, self.latest_gcm, self.config)
        out = FrontierArray()
        out.header = msg.header
        out.frontiers = []
        for frontier in frontiers:
            ros_frontier = Frontier()
            ros_frontier.position.x = float(frontier.position[0])
            ros_frontier.position.y = float(frontier.position[1])
            ros_frontier.position.z = float(frontier.position[2])
            ros_frontier.info_gain = float(frontier.info_gain)
            ros_frontier.region_id = int(frontier.region_id)
            out.frontiers.append(ros_frontier)
        self.publisher.publish(out)
        if self.publish_markers:
            self.marker_pub.publish(self._build_markers(msg, out))


def main() -> None:
    rclpy.init()
    node = FrontierNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
