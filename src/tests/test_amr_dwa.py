"""Synthetic LiDAR tests for the deterministic DWA-lite local planner."""

from __future__ import annotations

import math
import unittest

from config import CONFIG
from navigation.local_planner import LocalPlanner
from navigation.types import Point2D, Pose2D, Velocity
from navigation.global_planner import CostMap, GlobalPlanner, load_static_map
from navigation.helpers import SensorDataError, parse_lidar_points


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


class LocalPlannerTest(unittest.TestCase):
    def setUp(self):
        self.navigation_map = load_static_map(
            CONFIG.paths.navigation_map_path, CONFIG.navigation
        )
        self.costmap = CostMap(self.navigation_map, obstacle_ttl_seconds=5.0)
        self.local_planner = LocalPlanner(self.costmap, CONFIG.navigation)
        self.pose = Pose2D(-2.0, -0.5, math.pi / 2.0)
        self.path = (Point2D(-2.0, -0.5), Point2D(-2.0, 1.0))

    def command(self, lidar=None):
        field = GlobalPlanner(self.costmap, self.path[-1], CONFIG.navigation)
        return self.local_planner.command(
            pose=self.pose,
            velocity=Velocity(0.0, 0.0),
            path_world=self.path,
            obstacles_robot=parse_lidar_points(lidar or scan(), CONFIG.navigation),
            previous=Velocity(0.0, 0.0),
            progress_field=field,
        )

    def test_clear_path_selects_safe_forward_command(self):
        result = self.command()
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
        result = self.command(scan(front=0.16))
        self.assertTrue(result.safe)
        self.assertLessEqual(result.command.linear, 0.0)

    def test_lidar_obstacle_reduces_or_turns_the_command(self):
        result = self.command(scan(front=0.30))
        self.assertTrue(result.safe)
        self.assertTrue(
            result.command.linear < 0.1
            or abs(result.command.angular) > 1e-9
        )

    def test_zero_remains_a_safety_candidate_while_robot_is_moving(self):
        linear, angular = self.local_planner._window(Velocity(0.18, 1.0))
        self.assertIn(0.0, linear)
        self.assertIn(0.0, angular)

    def test_speed_cap_bounds_the_dynamic_window(self):
        default_window, _ = self.local_planner._window(Velocity(0.90, 0.0))
        self.assertAlmostEqual(
            max(default_window), CONFIG.navigation.default_linear_velocity_mps
        )

        for speed in (
            CONFIG.navigation.slow_linear_velocity_mps,
            CONFIG.navigation.fast_linear_velocity_mps,
            0.50,
        ):
            with self.subTest(speed=speed):
                self.local_planner.set_speed_mps(speed)
                window, _ = self.local_planner._window(Velocity(0.90, 0.0))
                self.assertAlmostEqual(self.local_planner.speed_mps, speed)
                self.assertAlmostEqual(max(window), speed)

        with self.assertRaises(ValueError):
            self.local_planner.set_speed_mps(1.0)
        with self.assertRaises(ValueError):
            self.local_planner.set_speed_mps(float("nan"))

    def test_robot_inside_margin_can_stop_or_turn_without_getting_closer(self):
        lidar = scan()
        lidar["ranges"][270] = 0.13
        result = self.command(lidar)
        self.assertTrue(result.safe)
        self.assertGreater(result.command.linear, 0.0)
        self.assertGreaterEqual(result.clearance_m, 0.0)

    def test_robot_can_escape_static_inflation_without_entering_raw_obstacle(self):
        pose = Pose2D(-1.39, 0.0, 2.60)
        self.assertFalse(self.navigation_map.is_free(pose.to_point()))
        self.assertTrue(self.costmap.is_raw_free(pose.to_point()))
        field = GlobalPlanner(self.costmap, Point2D(-1.9, 0.3), CONFIG.navigation)
        result = self.local_planner.command(
            pose=pose,
            velocity=Velocity(0.0, 0.0),
            path_world=(pose.to_point(), Point2D(-1.9, 0.3)),
            obstacles_robot=(),
            previous=Velocity(0.0, 0.0),
            progress_field=field,
        )
        self.assertTrue(result.safe)
        self.assertGreater(result.command.linear, 0.0)

    def test_no_static_clearance_has_no_safe_trajectory(self):
        field = GlobalPlanner(
            self.costmap, Point2D(0.5, 0.0), CONFIG.navigation
        )
        result = self.local_planner.command(
            pose=Pose2D(0.0, 0.0, 0.0),
            velocity=Velocity(0.0, 0.0),
            path_world=(Point2D(0.0, 0.0), Point2D(0.5, 0.0)),
            obstacles_robot=(),
            previous=Velocity(0.0, 0.0),
            progress_field=field,
        )
        self.assertFalse(result.safe)
        self.assertEqual(result.reason, "no_safe_map_trajectory")

    def test_reported_blocked_pose_selects_a_safe_escape_arc(self):
        pose = Pose2D(-1.29, -0.03, math.radians(-4.2))
        goal = Point2D(0.56, -1.50)
        field = GlobalPlanner(self.costmap, goal, CONFIG.navigation)
        path = field.plan(pose.to_point()).path

        result = self.local_planner.command(
            pose=pose,
            velocity=Velocity(0.0, 0.0),
            path_world=path,
            obstacles_robot=parse_lidar_points(
                scan(front=0.12), CONFIG.navigation
            ),
            previous=Velocity(0.0, 0.0),
            progress_field=field,
        )

        self.assertTrue(result.safe)
        self.assertEqual(result.reason, "escaping")
        self.assertLess(result.command.linear, 0.0)
        self.assertGreater(
            self.costmap.clearance_at(result.trajectory[-1].to_point()),
            self.costmap.clearance_at(pose.to_point()),
        )

    def test_free_pose_does_not_keep_reversing_after_escape(self):
        pose = Pose2D(-1.406, 0.009, math.radians(126.0))
        goal = Point2D(1.90, 0.0)
        field = GlobalPlanner(self.costmap, goal, CONFIG.navigation)
        result = self.local_planner.command(
            pose=pose,
            velocity=Velocity(-0.06, 0.0),
            path_world=field.plan(pose.to_point()).path,
            obstacles_robot=(),
            previous=Velocity(-0.06, 0.4),
            progress_field=field,
        )

        self.assertTrue(self.costmap.is_free(pose.to_point()))
        self.assertGreater(result.command.linear, 0.0)

    def test_free_pose_keeps_turning_until_forward_progress_is_available(self):
        pose = Pose2D(-1.4032, -0.0478, 0.0)
        goal = Point2D(0.56, -1.50)
        field = GlobalPlanner(self.costmap, goal, CONFIG.navigation)
        velocity = Velocity(0.0, 1.0)
        result = self.local_planner.command(
            pose=pose,
            velocity=velocity,
            path_world=field.plan(pose.to_point()).path,
            obstacles_robot=(),
            previous=velocity,
            progress_field=field,
        )

        self.assertTrue(self.costmap.is_free(pose.to_point()))
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
        field = GlobalPlanner(self.costmap, goal, CONFIG.navigation)
        lidar = scan(front=0.12)
        lidar["ranges"][180] = 0.12

        result = self.local_planner.command(
            pose=pose,
            velocity=Velocity(0.0, 0.0),
            path_world=field.plan(pose.to_point()).path,
            obstacles_robot=parse_lidar_points(lidar, CONFIG.navigation),
            previous=Velocity(0.0, 0.0),
            progress_field=field,
        )

        self.assertTrue(result.safe)
        self.assertEqual(result.command.linear, 0.0)

    def test_invalid_scan_is_not_treated_as_free_space(self):
        with self.assertRaises(SensorDataError):
            self.command(scan(invalid_count=1))

    def test_lidar_extrinsic_is_applied(self):
        points = parse_lidar_points(scan(front=1.0), CONFIG.navigation)
        self.assertAlmostEqual(points[0].x, 1.0 - 0.032, places=6)
        self.assertAlmostEqual(points[0].y, 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
