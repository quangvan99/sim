"""State-machine tests for autonomous command ownership."""

from __future__ import annotations

import math
import unittest

from navigation.core import NavigationState, Point2D
from navigation.manager import NavigationManager, NavigationSensorError


def scan(sequence: int = 1, front: float | None = None) -> dict:
    ranges = [None] * 360
    if front is not None:
        ranges[0] = front
    return {
        "angle_min": 0.0,
        "angle_max": 6.28,
        "angle_step": 6.28 / 359,
        "range_min": 0.12,
        "range_max": 3.5,
        "count": 360,
        "ranges": ranges,
        "invalid_count": 0,
        "sequence": sequence,
    }


class FakeTransport:
    def __init__(self):
        self.commands = []
        self.reset_count = 0
        self.lidar_age = 0.0
        self.x = 0.0
        self.y = 0.0
        self.model_x = -2.0
        self.model_y = -0.5
        self.model_yaw = 0.0
        self.lidar_sequence = 0
        self.front = None

    def telemetry_snapshot(self):
        self.lidar_sequence += 1
        return {
            "sensors": {
                "lidar": scan(self.lidar_sequence, self.front),
                "odometry": {
                    "position": {"x": self.x, "y": self.y, "z": 0.0},
                    "yaw": 0.0,
                    "linear_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                },
                "model_pose": {
                    "position": {
                        "x": self.model_x,
                        "y": self.model_y,
                        "z": 0.01,
                    },
                    "yaw": self.model_yaw,
                },
            },
            "freshness": {
                "lidar": {"received": True, "age_seconds": self.lidar_age},
                "odometry": {"received": True, "age_seconds": 0.0},
                "model_pose": {"received": True, "age_seconds": 0.0},
            },
        }

    def set_velocity(self, linear, angular):
        self.commands.append((linear, angular))
        return {"linear": linear, "angular": angular}

    def reset(self):
        self.reset_count += 1
        return True


class NavigationManagerTest(unittest.TestCase):
    def setUp(self):
        self.now = [0.0]
        self.transport = FakeTransport()
        self.manager = NavigationManager(
            self.transport, clock=lambda: self.now[0], auto_start=False
        )

    def tearDown(self):
        self.manager.close()

    def test_plan_start_control_pause_and_cancel(self):
        status = self.manager.plan(Point2D(-2.0, 0.5))
        self.assertEqual(status["state"], "READY")
        self.assertEqual(status["start"], {"x": -2.0, "y": -0.5})
        self.assertTrue(self.manager.owns_control)
        self.manager.start()
        self.manager._control_cycle()
        self.assertEqual(self.manager.state, NavigationState.NAVIGATING)
        self.assertNotEqual(self.transport.commands[-1], (0.0, 0.0))
        self.manager.pause()
        self.assertEqual(self.transport.commands[-1], (0.0, 0.0))
        self.manager.cancel()
        self.assertFalse(self.manager.owns_control)

    def test_stale_sensor_fails_plan(self):
        self.transport.lidar_age = 1.0
        with self.assertRaises(NavigationSensorError):
            self.manager.plan(Point2D(-2.0, 0.5))
        self.assertEqual(self.manager.state, NavigationState.ERROR)

    def test_goal_tolerance_stops_and_succeeds(self):
        self.manager.plan(Point2D(-2.0, -0.47))
        self.manager.start()
        self.manager._control_cycle()
        self.assertEqual(self.manager.state, NavigationState.SUCCEEDED)
        self.assertEqual(self.transport.commands[-1], (0.0, 0.0))

    def test_goal_at_current_pose_succeeds_with_zero_command(self):
        self.manager.plan(Point2D(-2.0, -0.5))
        self.manager.start()
        self.manager._control_cycle()
        self.assertEqual(self.manager.state, NavigationState.SUCCEEDED)
        self.assertEqual(self.transport.commands[-1], (0.0, 0.0))

    def test_completed_mission_can_be_followed_by_another_plan(self):
        self.manager.plan(Point2D(-2.0, -0.5))
        self.manager.start()
        self.manager._control_cycle()
        self.assertEqual(self.manager.state, NavigationState.SUCCEEDED)

        status = self.manager.plan(Point2D(-2.0, 0.5))

        self.assertEqual(status["state"], NavigationState.READY)
        self.assertEqual(status["goal"], {"x": -2.0, "y": 0.5})

    def test_reset_clears_mission(self):
        self.manager.plan(Point2D(-2.0, 0.5))
        self.assertTrue(self.manager.reset())
        self.assertEqual(self.manager.state, NavigationState.IDLE)
        self.assertEqual(self.transport.reset_count, 1)
        self.assertIsNone(self.manager.status()["goal"])

    def test_speed_mode_is_runtime_configurable(self):
        self.assertEqual(self.manager.status()["speed_mode"], "custom")
        self.assertEqual(self.manager.status()["speed_mps"], 0.50)
        status = self.manager.set_speed_mode("fast")
        self.assertEqual(status["speed_mode"], "fast")
        self.assertEqual(
            status["max_linear_velocity_mps"],
            0.90,
        )
        status = self.manager.set_speed_mps(0.50)
        self.assertEqual(status["speed_mode"], "custom")
        self.assertEqual(status["speed_mps"], 0.50)
        with self.assertRaises(ValueError):
            self.manager.set_speed_mode("turbo")
        with self.assertRaises(ValueError):
            self.manager.set_speed_mps(1.0)

    def test_entering_static_obstacle_transitions_to_blocked(self):
        self.manager.plan(Point2D(-2.0, 0.5))
        self.manager.start()
        self.transport.model_x = 0.0
        self.transport.model_y = 0.0
        self.manager._control_cycle()
        self.now[0] = 9.0
        self.manager._control_cycle()
        self.assertEqual(self.manager.state, NavigationState.BLOCKED)
        self.assertEqual(self.transport.commands[-1], (0.0, 0.0))

    def test_reported_inflated_start_commands_an_escape(self):
        self.transport.model_x = -1.29
        self.transport.model_y = -0.03
        self.transport.model_yaw = math.radians(-4.2)
        self.transport.front = 0.12
        self.manager.plan(Point2D(0.56, -1.50))
        self.manager.start()

        self.manager._control_cycle()

        self.assertEqual(self.manager.state, NavigationState.NAVIGATING)
        self.assertLess(self.transport.commands[-1][0], 0.0)
        self.assertEqual(self.manager.status()["local_reason"], "escaping")


if __name__ == "__main__":
    unittest.main()
