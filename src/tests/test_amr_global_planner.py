"""Unit tests for deterministic goal-field planning."""

from __future__ import annotations

import unittest

from config import CONFIG
from navigation.types import Point2D
from navigation.global_planner import CostMap, load_static_map
from navigation.global_planner import GlobalPlanner, PlanningError


class GlobalPlannerTest(unittest.TestCase):
    def setUp(self):
        self.navigation_map = load_static_map(
            CONFIG.paths.navigation_map_path, CONFIG.navigation
        )
        self.costmap = CostMap(self.navigation_map, obstacle_ttl_seconds=5.0)
        self.start = Point2D(-2.0, -0.5)

    def plan(self, start: Point2D, goal: Point2D):
        return GlobalPlanner(self.costmap, goal, CONFIG.navigation).plan(start)

    def test_direct_path_is_deterministic_and_collision_free(self):
        goal = Point2D(-2.0, 0.5)
        first = self.plan(self.start, goal)
        second = self.plan(self.start, goal)
        self.assertEqual(first, second)
        self.assertEqual(first.path[0], self.start)
        self.assertEqual(first.path[-1], goal)
        for start, end in zip(first.path, first.path[1:]):
            self.assertTrue(self.navigation_map.line_is_free(start, end))

    def test_path_detours_around_static_obstacles(self):
        goal = Point2D(2.0, 0.5)
        result = self.plan(self.start, goal)
        self.assertGreater(result.length_m, 4.0)
        self.assertTrue(
            all(self.navigation_map.is_free(point) for point in result.path)
        )

    def test_smoothing_preserves_clearance_for_reported_goal(self):
        result = self.plan(self.start, Point2D(-0.44, 0.43))
        minimum_clearance = min(
            min(
                self.navigation_map.clearance_at(point)
                for point in self.navigation_map.line_samples(start, end)
            )
            for start, end in zip(result.path, result.path[1:])
        )
        self.assertGreaterEqual(
            minimum_clearance,
            self.navigation_map.inflation_radius - 1e-9,
        )

    def test_representative_free_edge_goals_have_safe_paths(self):
        goals = (
            Point2D(-1.85, -0.45),
            Point2D(-2.45, 0.0),
            Point2D(2.15, 0.55),
            Point2D(0.0, 2.0),
            Point2D(0.0, -2.0),
            Point2D(-0.55, 0.55),
            Point2D(0.0, 0.55),
            Point2D(1.1, 0.55),
            Point2D(1.70, 3.99),
        )
        for goal in goals:
            with self.subTest(goal=goal):
                self.assertTrue(self.navigation_map.is_free(goal))
                result = self.plan(self.start, goal)
                self.assertEqual(result.path[0], self.start)
                self.assertEqual(result.path[-1], goal)
                self.assertTrue(
                    all(
                        self.navigation_map.line_is_free(start, end)
                        for start, end in zip(result.path, result.path[1:])
                    )
                )

    def test_inflated_goal_is_rejected_without_snapping(self):
        goal = Point2D(-1.39, 0.0)
        self.assertFalse(self.navigation_map.is_free(goal))
        with self.assertRaises(PlanningError) as caught:
            self.plan(self.start, goal)
        self.assertEqual(caught.exception.code, "GOAL_BLOCKED")

    def test_drifted_start_can_recover_out_of_inflation(self):
        start = Point2D(-1.39, 0.0)
        self.assertFalse(self.navigation_map.is_free(start))
        self.assertTrue(self.costmap.is_raw_free(start))
        result = self.plan(start, Point2D(-2.0, 0.5))
        self.assertEqual(result.path[0], start)
        self.assertGreater(
            self.navigation_map.clearance_at(result.raw[1]),
            self.navigation_map.clearance_at(start),
        )

    def test_start_inside_raw_obstacle_is_rejected(self):
        with self.assertRaises(PlanningError) as caught:
            self.plan(Point2D(0.0, 0.0), Point2D(-2.0, 0.5))
        self.assertEqual(caught.exception.code, "START_BLOCKED")

    def test_blocked_goal_is_rejected(self):
        with self.assertRaises(PlanningError) as caught:
            self.plan(self.start, Point2D(0.0, 0.0))
        self.assertEqual(caught.exception.code, "GOAL_BLOCKED")

    def test_temporary_wall_returns_no_path(self):
        wall = []
        for row in range(self.navigation_map.height):
            cell = (row, self.navigation_map.world_to_grid(Point2D(-1.7, 0.0))[1])
            wall.append(self.navigation_map.grid_to_world(cell))
        self.costmap.update_world_obstacles(tuple(wall), now=0.0)
        with self.assertRaises(PlanningError):
            self.plan(Point2D(-2.2, 0.0), Point2D(-1.2, 0.5))


if __name__ == "__main__":
    unittest.main()
