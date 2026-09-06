"""Synthetic LiDAR tests for the deterministic DWA-lite controller."""

from __future__ import annotations

import math
import unittest

from amr.config import CONFIG
from amr.navigation.controller import DWALocalPlanner, SensorDataError, lidar_points
from amr.navigation.core import Point2D, Pose2D, Velocity2D
from amr.navigation.planner import Costmap, GoalField, NavigationMap


def scan(*, front: float | None = None, invalid_count: int = 0) -> dict:
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
        "invalid_count": invalid_count,
    }


class DWALocalPlannerTest(unittest.TestCase):
    def setUp(self):
        self.navigation_map = NavigationMap.load()
        self.costmap = Costmap(self.navigation_map, ttl_cycles=25)
        self.controller = DWALocalPlanner(self.costmap)
        self.pose = Pose2D(-2.0, -0.5, math.pi / 2.0)
        self.path = (Point2D(-2.0, -0.5), Point2D(-2.0, 1.0))

    def choose(self, lidar=None):
        field = GoalField(self.costmap, self.path[-1])
        return self.controller.choose(
            pose=self.pose,
            velocity=Velocity2D(0.0, 0.0),
            global_path=self.path,
            lidar=lidar or scan(),
            previous_command=Velocity2D(0.0, 0.0),
            goal_field=field,
        )

    def test_clear_path_selects_safe_forward_command(self):
        result = self.choose()
        self.assertTrue(result.safe)
        self.assertGreater(result.command.linear, 0.0)
        self.assertLessEqual(
            result.command.linear, CONFIG.navigation.default_linear_velocity_mps
        )
        self.assertLessEqual(
            abs(result.command.angular),
            CONFIG.navigation.max_angular_velocity_rps,
        )

    def test_close_front_obstacle_never_drives_forward(self):
        result = self.choose(scan(front=0.16))
        self.assertTrue(result.safe)
        self.assertLessEqual(result.command.linear, 0.0)

    def test_lidar_obstacle_reduces_or_turns_the_command(self):
        result = self.choose(scan(front=0.30))
        self.assertTrue(result.safe)
        self.assertTrue(
            result.command.linear < 0.1
            or abs(result.command.angular) > 1e-9
        )

    def test_zero_remains_a_safety_candidate_while_robot_is_moving(self):
        linear, angular = self.controller._window(Velocity2D(0.18, 1.0))
        self.assertIn(0.0, linear)
        self.assertIn(0.0, angular)

    def test_speed_mode_changes_the_dynamic_window_cap(self):
        default_window, _ = self.controller._window(Velocity2D(0.90, 0.0))
        self.assertEqual(self.controller.speed_mode, "custom")
        self.assertAlmostEqual(max(default_window), CONFIG.navigation.default_linear_velocity_mps)

        self.controller.set_speed_mode("slow")
        slow, _ = self.controller._window(Velocity2D(0.90, 0.0))
        self.assertEqual(self.controller.speed_mode, "slow")
        self.assertAlmostEqual(max(slow), CONFIG.navigation.slow_linear_velocity_mps)

        self.controller.set_speed_mode("fast")
        fast, _ = self.controller._window(Velocity2D(0.90, 0.0))
        self.assertEqual(self.controller.speed_mode, "fast")
        self.assertAlmostEqual(max(fast), CONFIG.navigation.fast_linear_velocity_mps)

        self.controller.set_speed_mps(0.50)
        self.assertEqual(self.controller.speed_mode, "custom")
        self.assertAlmostEqual(self.controller.max_linear_velocity_mps, 0.50)

        with self.assertRaises(ValueError):
            self.controller.set_speed_mode("turbo")
        with self.assertRaises(ValueError):
            self.controller.set_speed_mps(1.0)

    def test_robot_inside_margin_can_stop_or_turn_without_getting_closer(self):
        lidar = scan()
        lidar["ranges"][270] = 0.13
        result = self.choose(lidar)
        self.assertTrue(result.safe)
        self.assertGreater(result.command.linear, 0.0)
        self.assertGreaterEqual(result.minimum_clearance_m, 0.0)

    def test_robot_can_escape_static_inflation_without_entering_raw_obstacle(self):
        pose = Pose2D(-1.39, 0.0, 2.60)
        self.assertFalse(self.navigation_map.is_free(pose.point()))
        self.assertTrue(self.costmap.is_raw_free(pose.point()))
        field = GoalField(self.costmap, Point2D(-1.9, 0.3))
        result = self.controller.choose(
            pose=pose,
            velocity=Velocity2D(0.0, 0.0),
            global_path=(pose.point(), Point2D(-1.9, 0.3)),
            lidar=scan(),
            previous_command=Velocity2D(0.0, 0.0),
            goal_field=field,
        )
        self.assertTrue(result.safe)
        self.assertGreater(result.command.linear, 0.0)

    def test_no_static_clearance_has_no_safe_trajectory(self):
        result = self.controller.choose(
            pose=Pose2D(0.0, 0.0, 0.0),
            velocity=Velocity2D(0.0, 0.0),
            global_path=(Point2D(0.0, 0.0), Point2D(0.5, 0.0)),
            lidar=scan(),
            previous_command=Velocity2D(0.0, 0.0),
        )
        self.assertFalse(result.safe)
        self.assertEqual(result.reason, "no_safe_map_trajectory")

    def test_reported_blocked_pose_selects_a_safe_escape_arc(self):
        pose = Pose2D(-1.29, -0.03, math.radians(-4.2))
        goal = Point2D(0.56, -1.50)
        field = GoalField(self.costmap, goal)
        path = field.path_from(pose.point()).path

        result = self.controller.choose(
            pose=pose,
            velocity=Velocity2D(0.0, 0.0),
            global_path=path,
            lidar=scan(front=0.12),
            previous_command=Velocity2D(0.0, 0.0),
            goal_field=field,
        )

        self.assertTrue(result.safe)
        self.assertEqual(result.reason, "escaping")
        self.assertLess(result.command.linear, 0.0)
        self.assertGreater(
            self.costmap.clearance_at(result.trajectory[-1].point()),
            self.costmap.clearance_at(pose.point()),
        )

    def test_free_pose_does_not_keep_reversing_after_escape(self):
        pose = Pose2D(-1.406, 0.009, math.radians(126.0))
        goal = Point2D(1.90, 0.0)
        field = GoalField(self.costmap, goal)
        result = self.controller.choose(
            pose=pose,
            velocity=Velocity2D(-0.06, 0.0),
            global_path=field.path_from(pose.point()).path,
            lidar=scan(),
            previous_command=Velocity2D(-0.06, 0.4),
            goal_field=field,
        )

        self.assertTrue(self.costmap.is_free(pose.point()))
        self.assertGreater(result.command.linear, 0.0)

    def test_free_pose_keeps_turning_until_forward_progress_is_available(self):
        pose = Pose2D(-1.4032, -0.0478, 0.0)
        goal = Point2D(0.56, -1.50)
        field = GoalField(self.costmap, goal)
        velocity = Velocity2D(0.0, 1.0)
        result = self.controller.choose(
            pose=pose,
            velocity=velocity,
            global_path=field.path_from(pose.point()).path,
            lidar=scan(),
            previous_command=velocity,
            goal_field=field,
        )

        self.assertTrue(self.costmap.is_free(pose.point()))
        self.assertGreaterEqual(
            abs(result.command.angular),
            0.5
            * CONFIG.navigation.angular_acceleration_rps2
            / CONFIG.navigation.control_hz,
        )
        self.assertGreaterEqual(result.command.linear, 0.0)

    def test_escape_does_not_reverse_into_a_rear_obstacle(self):
        pose = Pose2D(-1.29, -0.03, math.radians(-4.2))
        goal = Point2D(0.56, -1.50)
        field = GoalField(self.costmap, goal)
        lidar = scan(front=0.12)
        lidar["ranges"][180] = 0.12

        result = self.controller.choose(
            pose=pose,
            velocity=Velocity2D(0.0, 0.0),
            global_path=field.path_from(pose.point()).path,
            lidar=lidar,
            previous_command=Velocity2D(0.0, 0.0),
            goal_field=field,
        )

        self.assertTrue(result.safe)
        self.assertEqual(result.command.linear, 0.0)

    def test_invalid_scan_is_not_treated_as_free_space(self):
        with self.assertRaises(SensorDataError):
            self.choose(scan(invalid_count=1))

    def test_lidar_extrinsic_is_applied(self):
        points = lidar_points(scan(front=1.0))
        self.assertAlmostEqual(points[0].x, 1.0 - 0.032, places=6)
        self.assertAlmostEqual(points[0].y, 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
