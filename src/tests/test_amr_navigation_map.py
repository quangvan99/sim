"""Contract tests for the versioned AMR navigation map."""

from __future__ import annotations

import json
import unittest
import xml.etree.ElementTree as ET
from collections import deque

from config import CONFIG
from navigation.types import Point2D
from navigation.global_planner import CostMap, StaticMap


class MapTest(unittest.TestCase):
    def setUp(self):
        self.navigation_map = StaticMap.load(
            CONFIG.paths.navigation_map_path,
            config=CONFIG.navigation,
            world_name=CONFIG.simulation.world_name,
        )

    def test_map_contract_and_known_cells(self):
        self.assertEqual(self.navigation_map.version, "amr-world-nav-v1")
        self.assertEqual(self.navigation_map.world, "amr_world")
        self.assertTrue(self.navigation_map.is_free(Point2D(-2.0, -0.5)))
        self.assertFalse(self.navigation_map.is_free(Point2D(0.0, 0.0)))
        self.assertTrue(self.navigation_map.is_free(Point2D(3.1, 0.0)))
        self.assertFalse(self.navigation_map.is_free(Point2D(10.1, 0.0)))

    def test_open_ground_outside_old_hexagon_is_free(self):
        self.assertTrue(self.navigation_map.is_free(Point2D(2.45, 0.0)))
        self.assertTrue(self.navigation_map.is_free(Point2D(1.70, 3.99)))
        self.assertTrue(self.navigation_map.is_free(Point2D(-1.8, -2.2)))

    def test_world_cylinders_are_frozen_in_map(self):
        payload = json.loads(CONFIG.paths.navigation_map_path.read_text())
        mapped = {
            obstacle["id"]: (obstacle["x"], obstacle["y"], obstacle["radius"])
            for obstacle in payload["obstacles"]
            if obstacle["type"] == "circle"
        }
        root = ET.parse(
            CONFIG.paths.models_root / "world" / "model.sdf"
        ).getroot()
        collisions = root.findall(".//collision")[:9]
        for collision in collisions:
            values = [float(value) for value in collision.findtext("pose").split()]
            radius = float(collision.findtext("geometry/cylinder/radius"))
            self.assertEqual(
                mapped[collision.attrib["name"]],
                (values[0], values[1], radius),
            )

    def test_only_nine_pillars_are_navigation_obstacles(self):
        payload = json.loads(CONFIG.paths.navigation_map_path.read_text())
        self.assertEqual(len(payload["obstacles"]), 9)
        self.assertTrue(
            all(item["type"] == "circle" for item in payload["obstacles"])
        )
        model_root = ET.parse(
            CONFIG.paths.models_root / "world" / "model.sdf"
        ).getroot()
        collisions = {
            collision.attrib["name"]: collision
            for collision in model_root.findall(".//collision")
        }
        decorative = (
            "head",
            "left_hand",
            "right_hand",
            "left_foot",
            "right_foot",
            "body",
        )
        for name in decorative:
            pose = [float(value) for value in collisions[name].findtext("pose").split()]
            self.assertLessEqual(pose[2], -100.0)

    def test_coordinate_round_trip_uses_cell_center(self):
        point = Point2D(-2.0, -0.5)
        center = self.navigation_map.grid_to_world(
            self.navigation_map.world_to_grid(point)
        )
        self.assertLess(abs(center.x - point.x), self.navigation_map.resolution_m)
        self.assertLess(abs(center.y - point.y), self.navigation_map.resolution_m)

    def test_static_scan_points_are_not_runtime_obstacles(self):
        costmap = CostMap(self.navigation_map, obstacle_ttl_seconds=2.0)
        version = costmap.version
        self.assertFalse(
            costmap.update_world_obstacles((Point2D(-1.1, 0.16),), now=0.0)
        )
        self.assertEqual(costmap.version, version)

    def test_raw_and_inflated_occupancy_are_distinct(self):
        costmap = CostMap(self.navigation_map, obstacle_ttl_seconds=2.0)
        point = Point2D(-1.39, 0.0)
        self.assertTrue(costmap.is_raw_free(point))
        self.assertFalse(costmap.is_free(point))

    def test_temporary_obstacles_expire(self):
        layer = CostMap(self.navigation_map, obstacle_ttl_seconds=2.0)
        point = Point2D(-2.0, 0.5)
        self.assertTrue(layer.update_world_obstacles((point,), now=0.0))
        self.assertFalse(layer.is_free(point))
        self.assertFalse(layer.update_world_obstacles((), now=1.0))
        self.assertTrue(layer.update_world_obstacles((), now=2.0))
        self.assertTrue(layer.is_free(point))

    def test_every_free_cell_is_reachable_from_spawn(self):
        start = self.navigation_map.world_to_grid(Point2D(-2.0, -0.5))
        reachable = {start}
        frontier = deque([start])
        neighbors = (
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        )
        while frontier:
            row, column = frontier.popleft()
            for delta_row, delta_column in neighbors:
                cell = (row + delta_row, column + delta_column)
                if (
                    cell in reachable
                    or not self.navigation_map.contains_cell(cell)
                    or self.navigation_map.occupied[cell]
                ):
                    continue
                if delta_row and delta_column:
                    if (
                        self.navigation_map.occupied[row + delta_row, column]
                        or self.navigation_map.occupied[row, column + delta_column]
                    ):
                        continue
                reachable.add(cell)
                frontier.append(cell)

        free_cells = {
            (row, column)
            for row in range(self.navigation_map.height)
            for column in range(self.navigation_map.width)
            if not self.navigation_map.occupied[row, column]
        }
        self.assertEqual(reachable, free_cells)


if __name__ == "__main__":
    unittest.main()
