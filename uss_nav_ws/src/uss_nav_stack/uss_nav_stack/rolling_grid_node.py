from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from uss_nav_interfaces.msg import RollingGrid3D

from .rolling_grid import RollingGrid, RollingGridSpec


class RollingGridNode(Node):
    def __init__(self) -> None:
        super().__init__("rolling_grid_node")
        self.declare_parameter("size_xyz", [8.0, 8.0, 4.0])
        self.declare_parameter("resolution", 0.1)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("pose_topic_type", "pose_stamped")
        self.declare_parameter("points_topic", "")
        self.declare_parameter("use_external_points", True)
        self.declare_parameter("use_synthetic_fallback", True)
        self.declare_parameter("assume_points_already_in_frame", True)
        self.declare_parameter("points_timeout_sec", 1.0)
        self.declare_parameter("max_point_samples", 20000)
        self.declare_parameter("drop_z_below", -10.0)
        self.declare_parameter("drop_z_above", 10.0)
        self.declare_parameter("simulate_motion", True)
        self.declare_parameter("motion_radius", 1.5)
        self.declare_parameter("free_radius", 1.2)
        self.declare_parameter("enable_free_raycarving", True)
        self.declare_parameter("free_ray_step", 0.15)
        self.declare_parameter("max_free_ray_samples", 8000)
        self.declare_parameter("free_raycarving_stride", 1)
        self.declare_parameter("synthetic_obstacle_count", 90)
        self.declare_parameter("synthetic_seed", 7)

        size_xyz = tuple(float(x) for x in self.get_parameter("size_xyz").value)
        resolution = float(self.get_parameter("resolution").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.pose_topic_type = str(self.get_parameter("pose_topic_type").value).strip().lower()
        self.use_external_points = bool(self.get_parameter("use_external_points").value)
        self.use_synthetic_fallback = bool(self.get_parameter("use_synthetic_fallback").value)
        self.assume_points_already_in_frame = bool(self.get_parameter("assume_points_already_in_frame").value)
        self.points_timeout_sec = float(self.get_parameter("points_timeout_sec").value)
        self.max_point_samples = int(self.get_parameter("max_point_samples").value)
        self.drop_z_below = float(self.get_parameter("drop_z_below").value)
        self.drop_z_above = float(self.get_parameter("drop_z_above").value)
        self.simulate_motion = bool(self.get_parameter("simulate_motion").value)
        self.motion_radius = float(self.get_parameter("motion_radius").value)
        self.free_radius = float(self.get_parameter("free_radius").value)
        self.enable_free_raycarving = bool(self.get_parameter("enable_free_raycarving").value)
        self.free_ray_step = float(self.get_parameter("free_ray_step").value)
        self.max_free_ray_samples = max(1, int(self.get_parameter("max_free_ray_samples").value))
        self.free_raycarving_stride = max(1, int(self.get_parameter("free_raycarving_stride").value))
        self.synthetic_obstacle_count = int(self.get_parameter("synthetic_obstacle_count").value)
        self.rng = np.random.default_rng(int(self.get_parameter("synthetic_seed").value))
        self.current_position = np.zeros(3, dtype=np.float32)
        self.time_sec = 0.0
        self.latest_points: Optional[np.ndarray] = None
        self.latest_points_stamp_ns: int = 0
        self._warned_points_frame = False
        self._warned_points_timeout = False
        self._tick_count = 0

        self.grid = RollingGrid(RollingGridSpec(size_xyz=size_xyz, resolution=resolution))
        self.publisher = self.create_publisher(RollingGrid3D, "/map/rolling_grid", 10)

        pose_topic = str(self.get_parameter("pose_topic").value)
        self.pose_sub: Optional[object] = None
        if pose_topic:
            if self.pose_topic_type == "odometry":
                self.pose_sub = self.create_subscription(Odometry, pose_topic, self._on_odom, 20)
            else:
                self.pose_sub = self.create_subscription(PoseStamped, pose_topic, self._on_pose, 20)

        points_topic = str(self.get_parameter("points_topic").value)
        self.points_sub: Optional[object] = None
        if points_topic:
            self.points_sub = self.create_subscription(PointCloud2, points_topic, self._on_points, 10)

        hz = max(float(self.get_parameter("publish_hz").value), 0.5)
        self.timer = self.create_timer(1.0 / hz, self._on_timer)
        self.get_logger().info("rolling_grid_node started")

    def _on_pose(self, msg: PoseStamped) -> None:
        self.current_position[0] = float(msg.pose.position.x)
        self.current_position[1] = float(msg.pose.position.y)
        self.current_position[2] = float(msg.pose.position.z)

    def _on_odom(self, msg: Odometry) -> None:
        self.current_position[0] = float(msg.pose.pose.position.x)
        self.current_position[1] = float(msg.pose.pose.position.y)
        self.current_position[2] = float(msg.pose.pose.position.z)

    def _on_points(self, msg: PointCloud2) -> None:
        if not self.use_external_points:
            return
        if msg.header.frame_id and (msg.header.frame_id != self.frame_id) and not self.assume_points_already_in_frame:
            if not self._warned_points_frame:
                self.get_logger().warning(
                    f"points frame ({msg.header.frame_id}) != target frame ({self.frame_id}), "
                    "and assume_points_already_in_frame=False; skipping points"
                )
                self._warned_points_frame = True
            return
        try:
            total_points_hint = max(1, int(msg.width) * int(msg.height))
            sample_stride = max(1, total_points_hint // max(1, self.max_point_samples))
            # Fast path: available in Humble and significantly cheaper for large clouds.
            if hasattr(point_cloud2, "read_points_numpy"):
                points = point_cloud2.read_points_numpy(
                    msg,
                    field_names=["x", "y", "z"],
                    skip_nans=True,
                )
                points = np.asarray(points, dtype=np.float32)
                if points.ndim == 1 and points.size % 3 == 0:
                    points = points.reshape((-1, 3))
                if sample_stride > 1 and len(points) > 0:
                    points = points[::sample_stride]
            else:
                raw_points = point_cloud2.read_points(
                    msg,
                    field_names=("x", "y", "z"),
                    skip_nans=True,
                )
                sampled_points = []
                for i, p in enumerate(raw_points):
                    if sample_stride > 1 and (i % sample_stride) != 0:
                        continue
                    sampled_points.append((float(p[0]), float(p[1]), float(p[2])))
                    if len(sampled_points) >= self.max_point_samples:
                        break
                points = np.asarray(sampled_points, dtype=np.float32)
        except Exception as exc:  # pragma: no cover - defensive for malformed cloud schemas
            self.get_logger().warning(f"failed parsing PointCloud2: {exc}")
            return
        if points.size == 0:
            return
        if points.ndim == 1 and points.shape[0] == 3:
            points = points.reshape(1, 3)
        if points.ndim != 2 or points.shape[1] < 3:
            self.get_logger().warning(
                f"unexpected point shape from PointCloud2: {points.shape}"
            )
            return
        if points.shape[1] > 3:
            points = points[:, :3]
        z_mask = (points[:, 2] >= self.drop_z_below) & (points[:, 2] <= self.drop_z_above)
        points = points[z_mask]
        if points.size == 0:
            return
        if len(points) > self.max_point_samples:
            step = max(1, len(points) // self.max_point_samples)
            points = points[::step][: self.max_point_samples]
        self.latest_points = points.astype(np.float32)
        # Use local receive time to avoid cross-node timestamp skew causing false timeout.
        self.latest_points_stamp_ns = int(self.get_clock().now().nanoseconds)
        self._warned_points_timeout = False

    def _synthetic_points(self, center: np.ndarray) -> np.ndarray:
        offsets = self.rng.uniform(low=[-3.5, -3.5, -1.0], high=[3.5, 3.5, 2.0], size=(self.synthetic_obstacle_count, 3))
        points = offsets + center[None, :]
        return points.astype(np.float32)

    def _update_simulated_pose(self) -> None:
        if not self.simulate_motion:
            return
        self.time_sec += 0.1
        self.current_position[0] = self.motion_radius * np.cos(0.3 * self.time_sec)
        self.current_position[1] = self.motion_radius * np.sin(0.3 * self.time_sec)
        self.current_position[2] = 1.0 + 0.2 * np.sin(0.15 * self.time_sec)

    def _on_timer(self) -> None:
        self._update_simulated_pose()
        self._tick_count += 1
        self.grid.reset_unknown()
        self.grid.set_origin_centered(self.current_position)
        self.grid.mark_free_ball(self.current_position, self.free_radius)

        now_ns = int(self.get_clock().now().nanoseconds)
        points_to_use: Optional[np.ndarray] = None
        if self.use_external_points and self.latest_points is not None and self.latest_points_stamp_ns > 0:
            age_sec = max(0.0, (now_ns - self.latest_points_stamp_ns) * 1e-9)
            if age_sec <= self.points_timeout_sec:
                points_to_use = self.latest_points
            elif not self._warned_points_timeout:
                self.get_logger().warning(
                    f"external points timed out (age={age_sec:.2f}s), "
                    f"fallback={self.use_synthetic_fallback}"
                )
                self._warned_points_timeout = True
        if points_to_use is None and self.use_synthetic_fallback:
            points_to_use = self._synthetic_points(self.current_position)
        if points_to_use is not None:
            if (
                self.enable_free_raycarving
                and len(points_to_use) > 0
                and (self._tick_count % self.free_raycarving_stride == 0)
            ):
                ray_points = points_to_use
                if len(ray_points) > self.max_free_ray_samples:
                    step = max(1, len(ray_points) // self.max_free_ray_samples)
                    ray_points = ray_points[::step][: self.max_free_ray_samples]
                self.grid.mark_free_rays(self.current_position, ray_points, self.free_ray_step)
            self.grid.mark_points_occupied(points_to_use)

        msg = self.grid.as_msg(self.current_position, self.get_clock().now().to_msg(), self.frame_id)
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = RollingGridNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
