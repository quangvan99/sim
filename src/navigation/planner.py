"""Static map, per-cycle costmap, and Dijkstra-from-goal planning.

Planning is not an event here: the manager rebuilds the cost field whenever
the costmap changes and reads a fresh path from the robot's cell every cycle.
"""

from __future__ import annotations

import heapq
import json
import math
from collections.abc import Iterator, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from config import CONFIG
from navigation.core import (
    PlanResult,
    Point2D,
    path_length,
    point_in_polygon,
    resample_path,
)

_DIAGONAL = math.sqrt(2.0)
_NEIGHBORS = (
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (-1, 1, _DIAGONAL),
    (1, 1, _DIAGONAL),
    (1, -1, _DIAGONAL),
    (-1, -1, _DIAGONAL),
)


class PlanningError(RuntimeError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class MapBounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float


def clearance_grid(occupied: np.ndarray, resolution: float) -> np.ndarray:
    """Distance (m) from every cell to the nearest occupied cell centre."""

    if not occupied.any():
        return np.full(occupied.shape, math.inf)
    try:
        from scipy.ndimage import distance_transform_edt
    except ImportError:  # pragma: no cover - numpy-only fallback
        cells = np.argwhere(occupied)
        columns = np.arange(occupied.shape[1])
        result = np.empty(occupied.shape)
        for row in range(occupied.shape[0]):
            delta_row = cells[:, 0] - row
            delta_column = cells[None, :, 1] - columns[:, None]
            result[row] = np.sqrt(delta_row[None, :] ** 2 + delta_column**2).min(axis=1)
        return result * resolution
    return distance_transform_edt(~occupied) * resolution


class NavigationMap:
    """Load geometry JSON; expose grids for static geometry only."""

    def __init__(self, payload: dict):
        self.version = str(payload["version"])
        self.world = str(payload["world"])
        self.resolution = float(payload["resolution_m"])
        bounds = payload["bounds"]
        self.bounds = MapBounds(
            float(bounds["min_x"]),
            float(bounds["max_x"]),
            float(bounds["min_y"]),
            float(bounds["max_y"]),
        )
        self.robot_radius = float(payload["robot_radius_m"])
        self.safety_margin = float(payload["safety_margin_m"])
        self.inflation_radius = self.robot_radius + self.safety_margin
        self.boundary = tuple(
            Point2D(float(x), float(y)) for x, y in payload["boundary"]
        )
        self.obstacles = tuple(payload["obstacles"])
        self._validate()
        self.width = math.ceil(
            (self.bounds.max_x - self.bounds.min_x) / self.resolution
        )
        self.height = math.ceil(
            (self.bounds.max_y - self.bounds.min_y) / self.resolution
        )
        # Cell-centre distances overestimate surface distance by up to half a
        # cell; widen the band by that much so the grid is never less
        # conservative than exact geometric inflation.
        self.grid_bias = 0.5 * self.resolution
        self.raw_occupied = self._rasterize()
        self.clearance = clearance_grid(self.raw_occupied, self.resolution)
        self.occupied = self.clearance <= self.inflation_radius + self.grid_bias

    @classmethod
    def load(cls, path: Path | None = None) -> NavigationMap:
        source = path or CONFIG.paths.navigation_map_path
        return cls(json.loads(source.read_text()))

    def _validate(self) -> None:
        if self.version != CONFIG.navigation.map_version:
            raise ValueError(f"unsupported navigation map version: {self.version}")
        if self.world != CONFIG.simulation.world_name:
            raise ValueError(f"navigation map world mismatch: {self.world}")
        if self.resolution <= 0.0:
            raise ValueError("map resolution must be positive")
        if not (
            self.bounds.min_x < self.bounds.max_x
            and self.bounds.min_y < self.bounds.max_y
        ):
            raise ValueError("invalid map bounds")
        if len(self.boundary) < 3:
            raise ValueError("map boundary needs at least three points")
        if abs(self.robot_radius - CONFIG.navigation.robot_radius_m) > 1e-9:
            raise ValueError("map robot radius does not match config")
        if abs(self.safety_margin - CONFIG.navigation.safety_margin_m) > 1e-9:
            raise ValueError("map safety margin does not match config")
        identifiers = set()
        for obstacle in self.obstacles:
            identifier = obstacle.get("id")
            if not identifier or identifier in identifiers:
                raise ValueError("obstacle ids must be unique and non-empty")
            identifiers.add(identifier)
            if obstacle.get("type") not in {"circle", "polygon"}:
                raise ValueError(f"unsupported obstacle type: {obstacle.get('type')}")

    def world_to_grid(self, point: Point2D) -> tuple[int, int]:
        column = math.floor((point.x - self.bounds.min_x) / self.resolution)
        row = math.floor((point.y - self.bounds.min_y) / self.resolution)
        return row, column

    def grid_to_world(self, cell: tuple[int, int]) -> Point2D:
        row, column = cell
        return Point2D(
            self.bounds.min_x + (column + 0.5) * self.resolution,
            self.bounds.min_y + (row + 0.5) * self.resolution,
        )

    def contains_cell(self, cell: tuple[int, int]) -> bool:
        row, column = cell
        return 0 <= row < self.height and 0 <= column < self.width

    def _inside_obstacle(self, point: Point2D) -> bool:
        for obstacle in self.obstacles:
            if obstacle["type"] == "circle":
                if (
                    math.hypot(
                        point.x - float(obstacle["x"]),
                        point.y - float(obstacle["y"]),
                    )
                    <= float(obstacle["radius"])
                ):
                    return True
            elif point_in_polygon(
                point,
                tuple(Point2D(float(x), float(y)) for x, y in obstacle["points"]),
            ):
                return True
        return False

    def _rasterize(self) -> np.ndarray:
        occupied = np.zeros((self.height, self.width), dtype=np.bool_)
        for row in range(self.height):
            for column in range(self.width):
                point = self.grid_to_world((row, column))
                occupied[row, column] = not point_in_polygon(
                    point, self.boundary
                ) or self._inside_obstacle(point)
        return occupied

    def is_free(self, point: Point2D) -> bool:
        cell = self.world_to_grid(point)
        return self.contains_cell(cell) and not bool(self.occupied[cell])

    def clearance_at(self, point: Point2D) -> float:
        cell = self.world_to_grid(point)
        return float(self.clearance[cell]) if self.contains_cell(cell) else 0.0

    def line_samples(self, start: Point2D, end: Point2D) -> Iterator[Point2D]:
        length = math.hypot(end.x - start.x, end.y - start.y)
        samples = max(1, math.ceil(length / (self.resolution * 0.5)))
        for index in range(samples + 1):
            amount = index / samples
            yield Point2D(
                start.x + (end.x - start.x) * amount,
                start.y + (end.y - start.y) * amount,
            )

    def line_is_free(self, start: Point2D, end: Point2D) -> bool:
        return all(self.is_free(point) for point in self.line_samples(start, end))


class Costmap:
    """Static geometry plus LiDAR-detected cells that expire after a fixed
    number of cycles. Grids are rebuilt only when the marked set changes."""

    def __init__(self, navigation_map: NavigationMap, ttl_cycles: int):
        self.map = navigation_map
        self.ttl_cycles = max(1, ttl_cycles)
        self._marks = np.zeros((navigation_map.height, navigation_map.width), dtype=np.int32)
        self.version = 0
        self._rebuild()

    def _rebuild(self) -> None:
        self.raw_occupied = self.map.raw_occupied | (self._marks > 0)
        self.clearance = clearance_grid(self.raw_occupied, self.map.resolution)
        self.occupied = self.clearance <= self.map.inflation_radius + self.map.grid_bias
        self.version += 1

    def update(self, points: Sequence[Point2D]) -> bool:
        """Age old marks, add new ones; return True if occupancy changed."""

        before = self._marks > 0
        np.maximum(self._marks - 1, 0, out=self._marks)
        for point in points:
            cell = self.map.world_to_grid(point)
            # Returns from known geometry (already inside its inflation band)
            # carry no new information and would only jitter the band.
            if self.map.contains_cell(cell) and not self.map.occupied[cell]:
                self._marks[cell] = self.ttl_cycles
        changed = bool(np.any(before != (self._marks > 0)))
        if changed:
            self._rebuild()
        return changed

    def clear(self) -> None:
        if self._marks.any():
            self._marks.fill(0)
            self._rebuild()

    def is_free(self, point: Point2D) -> bool:
        cell = self.map.world_to_grid(point)
        return self.map.contains_cell(cell) and not bool(self.occupied[cell])

    def is_raw_free(self, point: Point2D) -> bool:
        cell = self.map.world_to_grid(point)
        return self.map.contains_cell(cell) and not bool(self.raw_occupied[cell])

    def clearance_at(self, point: Point2D) -> float:
        cell = self.map.world_to_grid(point)
        return float(self.clearance[cell]) if self.map.contains_cell(cell) else 0.0

    def line_is_free(self, start: Point2D, end: Point2D) -> bool:
        return all(self.is_free(point) for point in self.map.line_samples(start, end))

    def path_is_free(self, path: Sequence[Point2D]) -> bool:
        return all(self.line_is_free(a, b) for a, b in zip(path, path[1:]))


class GoalField:
    """Cost-to-go from every free cell to the goal (Dijkstra expanded from
    the goal), with the same clearance-penalised step cost as before."""

    def __init__(self, costmap: Costmap, goal: Point2D):
        self.costmap = costmap
        self.map = costmap.map
        self.goal = goal
        self.goal_cell = self.map.world_to_grid(goal)
        if not self.map.contains_cell(self.goal_cell):
            raise PlanningError("OUT_OF_BOUNDS", "goal is outside the map")
        if costmap.occupied[self.goal_cell]:
            raise PlanningError("GOAL_BLOCKED", "goal is in an inflated obstacle")
        self.cost, self.parent = self._dijkstra()

    def _dijkstra(self) -> tuple[np.ndarray, dict[tuple[int, int], tuple[int, int]]]:
        occupied = self.costmap.occupied
        clearance = self.costmap.clearance
        preferred = CONFIG.navigation.astar_preferred_clearance_m
        weight = CONFIG.navigation.astar_clearance_cost_weight
        cost = np.full(occupied.shape, math.inf)
        parent: dict[tuple[int, int], tuple[int, int]] = {}
        cost[self.goal_cell] = 0.0
        queue = [(0.0, self.goal_cell)]
        while queue:
            current_cost, (row, column) = heapq.heappop(queue)
            if current_cost > cost[row, column]:
                continue
            for dr, dc, step in _NEIGHBORS:
                cell = (row + dr, column + dc)
                if not self.map.contains_cell(cell) or occupied[cell]:
                    continue
                if dr and dc and (occupied[row + dr, column] or occupied[row, column + dc]):
                    continue
                deficit = max(0.0, (preferred - clearance[cell]) / preferred)
                candidate = current_cost + step * (1.0 + weight * deficit * deficit)
                if candidate + 1e-12 < cost[cell]:
                    cost[cell] = candidate
                    parent[cell] = (row, column)
                    heapq.heappush(queue, (candidate, cell))
        return cost, parent

    def _reachable_cell(self, point: Point2D) -> tuple[int, int]:
        cell = self.map.world_to_grid(point)
        if not self.map.contains_cell(cell):
            raise PlanningError("OUT_OF_BOUNDS", "start is outside the map")
        if math.isfinite(self.cost[cell]):
            return cell
        # Pose noise can leave the robot inside the inflation band: step to the
        # nearest reachable cell that is visible through raw-free space.
        reach = math.ceil(self.map.safety_margin / self.map.resolution) + 1
        candidates = [
            (row, column)
            for row in range(cell[0] - reach, cell[0] + reach + 1)
            for column in range(cell[1] - reach, cell[1] + reach + 1)
            if self.map.contains_cell((row, column))
            and math.isfinite(self.cost[row, column])
            and all(
                self.costmap.is_raw_free(sample)
                for sample in self.map.line_samples(
                    point, self.map.grid_to_world((row, column))
                )
            )
        ]
        if not candidates:
            code = "START_BLOCKED" if self.costmap.occupied[cell] else "NO_PATH"
            raise PlanningError(code, f"no collision-free path to goal ({code})")
        return min(
            candidates,
            key=lambda item: (math.hypot(item[0] - cell[0], item[1] - cell[1]), item),
        )

    def cost_to_go(self, point: Point2D) -> float:
        cell = self.map.world_to_grid(point)
        return float(self.cost[cell]) if self.map.contains_cell(cell) else math.inf

    def path_from(self, start: Point2D) -> PlanResult:
        cell = self._reachable_cell(start)
        cells = [cell]
        while cells[-1] != self.goal_cell:
            cells.append(self.parent[cells[-1]])
        raw = [self.map.grid_to_world(item) for item in cells]
        if self.map.world_to_grid(start) == cell:
            raw[0] = start
        else:
            raw.insert(0, start)
        raw[-1] = self.goal
        path = resample_path(self._smooth(tuple(raw)), CONFIG.navigation.waypoint_spacing_m)
        return PlanResult(tuple(raw), path, path_length(path))

    def _smooth(self, path: tuple[Point2D, ...]) -> tuple[Point2D, ...]:
        """Greedy line-of-sight shortcuts that never reduce clearance."""

        result = [path[0]]
        anchor = 0
        while anchor < len(path) - 1:
            target = len(path) - 1
            while target > anchor + 1:
                segment = path[anchor : target + 1]
                original = min(self.costmap.clearance_at(point) for point in segment)
                shortcut = min(
                    self.costmap.clearance_at(point)
                    for point in self.map.line_samples(path[anchor], path[target])
                )
                if shortcut + 1e-9 >= original and self.costmap.line_is_free(
                    path[anchor], path[target]
                ):
                    break
                target -= 1
            result.append(path[target])
            anchor = target
        return tuple(result)