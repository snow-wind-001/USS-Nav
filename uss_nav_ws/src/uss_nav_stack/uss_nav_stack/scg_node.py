from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from uss_nav_interfaces.msg import RollingGrid3D, SCGGraph

from .rolling_grid import RollingGrid
from .scg import SCGConfig, SpatialConnectivityGraphBuilder


class SCGNode(Node):
    def __init__(self) -> None:
        super().__init__("scg_node")
        self.declare_parameter("fibonacci_samples", 50)
        self.declare_parameter("max_ray_radius", 1.4)
        self.declare_parameter("max_seed_extension", 2.0)
        self.declare_parameter("r_vis", 3.0)
        self.declare_parameter("max_nodes", 400)
        self.declare_parameter("steps_per_update", 2)

        config = SCGConfig(
            fibonacci_samples=int(self.get_parameter("fibonacci_samples").value),
            max_ray_radius=float(self.get_parameter("max_ray_radius").value),
            max_seed_extension=float(self.get_parameter("max_seed_extension").value),
            r_vis=float(self.get_parameter("r_vis").value),
            max_nodes=int(self.get_parameter("max_nodes").value),
        )
        self.steps_per_update = int(self.get_parameter("steps_per_update").value)
        self.builder = SpatialConnectivityGraphBuilder(config=config)
        self.publisher = self.create_publisher(SCGGraph, "/scg/graph", 10)
        self.subscription = self.create_subscription(RollingGrid3D, "/map/rolling_grid", self._on_grid, 10)
        self.get_logger().info("scg_node started")

    def _on_grid(self, msg: RollingGrid3D) -> None:
        grid = RollingGrid.from_msg(msg)
        center = np.array(
            [
                msg.origin.position.x + 0.5 * msg.size_x * msg.resolution,
                msg.origin.position.y + 0.5 * msg.size_y * msg.resolution,
                msg.origin.position.z + 0.5 * msg.size_z * msg.resolution,
            ],
            dtype=np.float32,
        )
        self.builder.update(grid, center, max_steps=self.steps_per_update)
        self.publisher.publish(self.builder.to_msg(msg.header.stamp, msg.header.frame_id))


def main() -> None:
    rclpy.init()
    node = SCGNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
