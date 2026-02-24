#!/usr/bin/env python3

import argparse
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class FakeOdomPublisher(Node):
    def __init__(self, topic: str, hz: float, radius: float, omega: float, z: float, z_amp: float) -> None:
        super().__init__("fake_odom_publisher")
        self.publisher = self.create_publisher(Odometry, topic, 20)
        self.radius = float(radius)
        self.omega = float(omega)
        self.z = float(z)
        self.z_amp = float(z_amp)
        self.start_sec = float(self.get_clock().now().nanoseconds) * 1e-9
        period = 1.0 / max(1.0, float(hz))
        self.timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f"publishing fake odom on {topic}, hz={hz}, radius={radius}, omega={omega}, z={z}, z_amp={z_amp}"
        )

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        t = float(now.nanoseconds) * 1e-9 - self.start_sec
        theta = self.omega * t
        x = self.radius * math.cos(theta)
        y = self.radius * math.sin(theta)
        z = self.z + self.z_amp * math.sin(0.5 * theta)

        vx = -self.radius * self.omega * math.sin(theta)
        vy = self.radius * self.omega * math.cos(theta)
        vz = 0.5 * self.z_amp * self.omega * math.cos(0.5 * theta)
        yaw = theta + math.pi * 0.5
        half = 0.5 * yaw

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "world"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = float(math.sin(half))
        msg.pose.pose.orientation.w = float(math.cos(half))
        msg.twist.twist.linear.x = float(vx)
        msg.twist.twist.linear.y = float(vy)
        msg.twist.twist.linear.z = float(vz)
        msg.twist.twist.angular.z = float(self.omega)
        self.publisher.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/odom")
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--radius", type=float, default=1.8)
    parser.add_argument("--omega", type=float, default=0.08)
    parser.add_argument("--z", type=float, default=1.0)
    parser.add_argument("--z-amp", type=float, default=0.12)
    args = parser.parse_args()

    rclpy.init()
    node = FakeOdomPublisher(
        topic=str(args.topic),
        hz=float(args.hz),
        radius=float(args.radius),
        omega=float(args.omega),
        z=float(args.z),
        z_amp=float(args.z_amp),
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
