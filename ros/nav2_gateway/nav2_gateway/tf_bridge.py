"""Publish robot-body TF while Nav2 localization owns ``map -> odom``."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class SimulatorTF(Node):
    def __init__(self):
        super().__init__("amr_simulator_tf")
        self._dynamic = TransformBroadcaster(self)
        self._static = StaticTransformBroadcaster(self)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._publish_static()

    def _publish_static(self):
        transforms = [
            self._static_transform("base_footprint", "base_link", 0.0, 0.0, 0.0),
            self._static_transform("base_link", "base_scan", -0.032, 0.0, 0.171),
        ]
        self._static.sendTransform(transforms)

    def _on_odom(self, message: Odometry):
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self._dynamic.sendTransform(transform)

    def _static_transform(
        self, parent: str, child: str, x: float, y: float, z: float
    ) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.w = 1.0
        return transform


def main(args=None):
    rclpy.init(args=args)
    node = SimulatorTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
