from __future__ import annotations

from collections import defaultdict
from typing import Dict, List

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from uss_nav_interfaces.msg import FrontierArray, ObjectGraph, RegionLabelArray, RollingGrid3D
from visualization_msgs.msg import Marker, MarkerArray

from .common import FrontierState, PlannerContext
from .planner import PlannerConfig, choose_target_region, order_frontiers_tsp_like


class PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("planner_node")
        self.declare_parameter("target_label", "chair")
        self.declare_parameter("alpha", 1.0)
        self.declare_parameter("beta", 0.5)
        self.declare_parameter("explore_only", True)
        self.declare_parameter("stop_on_done", True)
        self.declare_parameter("command_waypoint_topic", "")
        self.declare_parameter("command_waypoint_frame_id", "")
        self.declare_parameter("command_waypoint_max_step_xy", 1.2)
        self.declare_parameter("command_waypoint_max_step_z", 0.35)
        self.declare_parameter("command_waypoint_min_z", 0.8)
        self.declare_parameter("command_waypoint_max_z", 2.0)
        self.declare_parameter("command_waypoint_abs_xy_limit", 30.0)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_topic", "/viz/waypoint")
        self.target_label = str(self.get_parameter("target_label").value)
        self.explore_only = bool(self.get_parameter("explore_only").value)
        self.stop_on_done = bool(self.get_parameter("stop_on_done").value)
        self.publish_markers = bool(self.get_parameter("publish_markers").value)
        self.command_waypoint_frame_id = str(self.get_parameter("command_waypoint_frame_id").value).strip()
        self.command_waypoint_max_step_xy = float(self.get_parameter("command_waypoint_max_step_xy").value)
        self.command_waypoint_max_step_z = float(self.get_parameter("command_waypoint_max_step_z").value)
        self.command_waypoint_min_z = float(self.get_parameter("command_waypoint_min_z").value)
        self.command_waypoint_max_z = float(self.get_parameter("command_waypoint_max_z").value)
        self.command_waypoint_abs_xy_limit = float(self.get_parameter("command_waypoint_abs_xy_limit").value)
        self.config = PlannerConfig(
            alpha=float(self.get_parameter("alpha").value),
            beta=float(self.get_parameter("beta").value),
        )
        self.current_position = np.zeros(3, dtype=np.float32)
        self.node_to_region: Dict[int, int] = {}
        self.region_object_scores: Dict[int, float] = {}
        self.explore_done = False

        self.target_pub = self.create_publisher(Int32, "/nav/target_region", 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, "/nav/waypoint", 10)
        self.command_waypoint_pub = None
        command_waypoint_topic = str(self.get_parameter("command_waypoint_topic").value).strip()
        if command_waypoint_topic:
            self.command_waypoint_pub = self.create_publisher(PoseStamped, command_waypoint_topic, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            10,
        )
        self.frontier_sub = self.create_subscription(FrontierArray, "/nav/frontiers", self._on_frontiers, 10)
        self.done_sub = self.create_subscription(Bool, "/nav/explore_done", self._on_done, 10)
        self.region_sub = None
        self.object_sub = None
        if not self.explore_only:
            self.region_sub = self.create_subscription(RegionLabelArray, "/region/labels", self._on_regions, 10)
            self.object_sub = self.create_subscription(ObjectGraph, "/objects/graph", self._on_objects, 10)
        self.grid_sub = self.create_subscription(RollingGrid3D, "/map/rolling_grid", self._on_grid, 10)
        mode_text = "explore_only" if self.explore_only else "hierarchical"
        self.get_logger().info(f"planner_node started in {mode_text} mode")

    def _on_done(self, msg: Bool) -> None:
        prev = self.explore_done
        self.explore_done = bool(msg.data)
        if self.explore_done and not prev:
            self.get_logger().info("planner received explore_done=True, stopping waypoint updates")
            self._publish_waypoint_marker(None, None)

    def _publish_waypoint_marker(self, header, waypoint: PoseStamped | None) -> None:
        if not self.publish_markers:
            return
        markers = MarkerArray()
        clear = Marker()
        if header is not None:
            clear.header = header
        clear.ns = "waypoint_clear"
        clear.id = 0
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        if waypoint is not None:
            sphere = Marker()
            sphere.header = waypoint.header
            sphere.ns = "waypoint"
            sphere.id = 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.scale.x = 0.25
            sphere.scale.y = 0.25
            sphere.scale.z = 0.25
            sphere.pose = waypoint.pose
            sphere.pose.orientation.w = 1.0
            sphere.color.a = 0.95
            sphere.color.r = 0.1
            sphere.color.g = 0.8
            sphere.color.b = 1.0
            markers.markers.append(sphere)

            text = Marker()
            text.header = waypoint.header
            text.ns = "waypoint_text"
            text.id = 2
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = waypoint.pose.position.x
            text.pose.position.y = waypoint.pose.position.y
            text.pose.position.z = waypoint.pose.position.z + 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = 0.24
            text.color.a = 0.95
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.text = "next waypoint"
            markers.markers.append(text)

        self.marker_pub.publish(markers)

    def _build_command_waypoint(self, waypoint: PoseStamped) -> PoseStamped:
        target = np.array(
            [
                waypoint.pose.position.x,
                waypoint.pose.position.y,
                waypoint.pose.position.z,
            ],
            dtype=np.float32,
        )
        current = np.array(self.current_position, dtype=np.float32)
        delta = target - current
        xy_norm = float(np.linalg.norm(delta[:2]))
        if xy_norm > max(0.01, self.command_waypoint_max_step_xy):
            scale = self.command_waypoint_max_step_xy / xy_norm
            delta[0] *= scale
            delta[1] *= scale
        delta[2] = float(np.clip(delta[2], -self.command_waypoint_max_step_z, self.command_waypoint_max_step_z))
        cmd_pos = current + delta
        # Soft absolute bounds: only move toward bounds with configured step size.
        lim_xy = max(0.1, float(self.command_waypoint_abs_xy_limit))
        desired_x = float(np.clip(cmd_pos[0], -lim_xy, lim_xy))
        desired_y = float(np.clip(cmd_pos[1], -lim_xy, lim_xy))
        cmd_pos[0] = float(current[0] + np.clip(desired_x - current[0], -self.command_waypoint_max_step_xy, self.command_waypoint_max_step_xy))
        cmd_pos[1] = float(current[1] + np.clip(desired_y - current[1], -self.command_waypoint_max_step_xy, self.command_waypoint_max_step_xy))
        desired_z = float(np.clip(cmd_pos[2], self.command_waypoint_min_z, self.command_waypoint_max_z))
        cmd_pos[2] = float(current[2] + np.clip(desired_z - current[2], -self.command_waypoint_max_step_z, self.command_waypoint_max_step_z))
        # Keep flight setpoint above floor even if odom briefly dips.
        cmd_pos[2] = float(np.clip(cmd_pos[2], self.command_waypoint_min_z, self.command_waypoint_max_z))

        cmd_waypoint = PoseStamped()
        cmd_waypoint.header = waypoint.header
        if self.command_waypoint_frame_id:
            cmd_waypoint.header.frame_id = self.command_waypoint_frame_id
        cmd_waypoint.pose = waypoint.pose
        cmd_waypoint.pose.position.x = float(cmd_pos[0])
        cmd_waypoint.pose.position.y = float(cmd_pos[1])
        cmd_waypoint.pose.position.z = float(cmd_pos[2])
        return cmd_waypoint

    def _on_grid(self, msg: RollingGrid3D) -> None:
        self.current_position = np.array(
            [
                msg.origin.position.x + 0.5 * msg.size_x * msg.resolution,
                msg.origin.position.y + 0.5 * msg.size_y * msg.resolution,
                msg.origin.position.z + 0.5 * msg.size_z * msg.resolution,
            ],
            dtype=np.float32,
        )

    def _on_regions(self, msg: RegionLabelArray) -> None:
        mapping: Dict[int, int] = {}
        for item in msg.labels:
            mapping[int(item.node_id)] = int(item.region_id)
        self.node_to_region = mapping

    def _on_objects(self, msg: ObjectGraph) -> None:
        scores = defaultdict(float)
        for obj in msg.objects:
            region = self.node_to_region.get(int(obj.anchor_scg_node_id), -1)
            if region < 0:
                continue
            if obj.label == self.target_label:
                scores[region] += 1.0
            else:
                scores[region] += 0.05
        self.region_object_scores = dict(scores)

    def _on_frontiers(self, msg: FrontierArray) -> None:
        if self.stop_on_done and self.explore_done:
            target_msg = Int32()
            target_msg.data = -1
            self.target_pub.publish(target_msg)
            self._publish_waypoint_marker(msg.header, None)
            return

        all_frontiers: List[FrontierState] = []
        grouped: Dict[int, List[FrontierState]] = defaultdict(list)
        for item in msg.frontiers:
            state = FrontierState(
                position=np.array([item.position.x, item.position.y, item.position.z], dtype=np.float32),
                info_gain=float(item.info_gain),
                region_id=int(item.region_id),
            )
            grouped[state.region_id].append(state)
            all_frontiers.append(state)

        target_msg = Int32()
        selected: List[FrontierState]
        if self.explore_only:
            # Pure exploration (A mode): ignore region/object hierarchy.
            target_msg.data = 0
            selected = all_frontiers
        else:
            context = PlannerContext(
                frontiers_by_region=dict(grouped),
                region_object_scores=self.region_object_scores,
                current_position=self.current_position,
                target_label=self.target_label,
            )
            target_region = choose_target_region(context)
            target_msg.data = int(target_region)
            selected = context.frontiers_by_region.get(target_region, [])
        self.target_pub.publish(target_msg)

        ordered = order_frontiers_tsp_like(selected, self.current_position, self.config)
        if not ordered:
            self._publish_waypoint_marker(msg.header, None)
            return
        waypoint = PoseStamped()
        waypoint.header = msg.header
        waypoint.pose.position.x = float(ordered[0].position[0])
        waypoint.pose.position.y = float(ordered[0].position[1])
        waypoint.pose.position.z = float(ordered[0].position[2])
        waypoint.pose.orientation.w = 1.0
        self.waypoint_pub.publish(waypoint)
        if self.command_waypoint_pub is not None:
            cmd_waypoint = self._build_command_waypoint(waypoint)
            self.command_waypoint_pub.publish(cmd_waypoint)
        self._publish_waypoint_marker(msg.header, waypoint)


def main() -> None:
    rclpy.init()
    node = PlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
