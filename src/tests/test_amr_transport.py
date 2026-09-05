"""Unit tests for AMR Gazebo Transport control and telemetry."""

from __future__ import annotations

import unittest

from gz.msgs.imu_pb2 import IMU
from gz.msgs.laserscan_pb2 import LaserScan
from gz.msgs.model_pb2 import Model
from gz.msgs.odometry_pb2 import Odometry

from amr.config import CONFIG
from amr.transport import AMRTransport


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        clone = type(message)()
        clone.CopyFrom(message)
        self.messages.append(clone)
        return True


class FakeNode:
    def __init__(self):
        self.publisher = FakePublisher()
        self.subscriptions = {}

    def advertise(self, topic, message_type):
        self.advertised = (topic, message_type)
        return self.publisher

    def subscribe(self, message_type, topic, callback):
        self.subscriptions[topic] = (message_type, callback)
        return True

    def unsubscribe(self, topic):
        self.subscriptions.pop(topic, None)
        return True

    def emit(self, topic, message):
        self.subscriptions[topic][1](message)


class AMRTransportTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.node = FakeNode()
        self.reset_count = 0

        def reset_client():
            self.reset_count += 1
            return True

        self.transport = AMRTransport(
            node=self.node,
            reset_client=reset_client,
            clock=lambda: self.now,
            auto_start=False,
        )

    def tearDown(self):
        self.transport.close()

    def test_velocity_is_clamped_and_published_as_twist(self):
        command = self.transport.set_velocity(10.0, -10.0)

        self.assertEqual(
            command["linear"], CONFIG.control.max_linear_velocity_mps
        )
        self.assertEqual(
            command["angular"], -CONFIG.control.max_angular_velocity_rps
        )
        message = self.node.publisher.messages[-1]
        self.assertEqual(message.linear.x, command["linear"])
        self.assertEqual(message.angular.z, command["angular"])

    def test_invalid_velocity_is_rejected(self):
        for value in (True, "fast", float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.transport.set_velocity(value, 0.0)

    def test_dead_man_timeout_returns_zero_command(self):
        self.transport.set_velocity(0.1, 0.2)
        self.now += CONFIG.control.command_timeout_seconds + 0.01

        command = self.transport.telemetry_snapshot()["command"]

        self.assertEqual(command["linear"], 0.0)
        self.assertEqual(command["angular"], 0.0)
        self.assertFalse(command["active"])

    def test_sensor_callbacks_produce_fresh_json_safe_snapshot(self):
        scan = LaserScan()
        scan.frame = "base_scan"
        scan.count = 3
        scan.angle_min = 0.0
        scan.angle_max = 1.0
        scan.angle_step = 0.5
        scan.range_min = 0.12
        scan.range_max = 3.5
        scan.ranges.extend([0.5, float("inf"), 1.5])
        self.node.emit(CONFIG.simulation.lidar_topic, scan)

        imu = IMU()
        imu.entity_name = "amr::imu_link"
        imu.orientation.w = 1.0
        imu.angular_velocity.z = 0.3
        imu.linear_acceleration.x = 0.1
        self.node.emit(CONFIG.simulation.imu_topic, imu)

        odometry = Odometry()
        odometry.pose.position.x = 1.25
        odometry.pose.orientation.w = 1.0
        odometry.twist.linear.x = 0.2
        self.node.emit(CONFIG.simulation.odometry_topic, odometry)

        joints = Model()
        joints.name = "amr"
        joint = joints.joint.add()
        joint.name = "wheel_left_joint"
        joint.axis1.position = 0.4
        joint.axis1.velocity = 1.2
        self.node.emit(CONFIG.simulation.joint_state_topic, joints)

        snapshot = self.transport.telemetry_snapshot()

        self.assertTrue(snapshot["ready"])
        self.assertEqual(snapshot["sensors"]["lidar"]["count"], 3)
        self.assertIsNone(snapshot["sensors"]["lidar"]["ranges"][1])
        self.assertEqual(snapshot["sensors"]["odometry"]["position"]["x"], 1.25)
        self.assertEqual(
            snapshot["sensors"]["joint_states"]["joints"][0]["name"],
            "wheel_left_joint",
        )

    def test_reset_stops_robot_and_calls_world_control(self):
        self.transport.set_velocity(0.1, 0.2)

        self.assertTrue(self.transport.reset())

        self.assertEqual(self.node.publisher.messages[-1].linear.x, 0.0)
        self.assertEqual(self.reset_count, 1)


if __name__ == "__main__":
    unittest.main()
