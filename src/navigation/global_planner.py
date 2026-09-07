"""Static map, costmap, and goal-directed global planning."""

from __future__ import annotations

import heapq
import json
import math
from collections.abc import Iterator, Sequence
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path

import numpy as np

from config import NavigationConfig
from navigation.helpers import path_length, point_in_polygon, resample_path
from navigation.types import GlobalPlan, Point2D

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

# -- global planning ----------------------------------------------------------


class PlanningErrorCode(StrEnum):
    OUT_OF_BOUNDS = "OUT_OF_BOUNDS"
    START_BLOCKED = "START_BLOCKED"
    GOAL_BLOCKED = "GOAL_BLOCKED"
    NO_PATH = "NO_PATH"


class PlanningError(RuntimeError):
    def __init__(self, code: PlanningErrorCode, message: str):
        super().__init__(message)
        self.code = code


class GlobalPlanner:
    """Maintain a Dijkstra cost-to-go field for one world-frame goal."""

    def __init__(
        self,
        cost_map: CostMap,
        goal: Point2D,
        config: NavigationConfig,
    ):
        self.cost_map = cost_map
        self.map = cost_map.map
        self.goal = goal
        self.config = config
        self.goal_cell = self.map.world_to_grid(goal)
        if not self.map.contains_cell(self.goal_cell):
            raise PlanningError(
                PlanningErrorCode.OUT_OF_BOUNDS,
                "goal is outside the map",
            )
        self.costs = np.empty((0, 0))
        self.next_cell: dict[tuple[int, int], tuple[int, int]] = {}
        self._map_version = -1
        self.refresh()

    def refresh(self) -> None:
        """Rebuild the cost field after the costmap changes."""

        if self.cost_map.occupied[self.goal_cell]:
            raise PlanningError(
                PlanningErrorCode.GOAL_BLOCKED,
                "goal is in an inflated obstacle",
            )
        if self._map_version == self.cost_map.version:
            return
        self.costs, self.next_cell = self._dijkstra()
        self._map_version = self.cost_map.version

    def _dijkstra(self) -> tuple[np.ndarray, dict[tuple[int, int], tuple[int, int]]]:
        occupied = self.cost_map.occupied
        clearance = self.cost_map.clearance
        preferred = self.config.astar_preferred_clearance_m
        weight = self.config.astar_clearance_cost_weight
        costs = np.full(occupied.shape, math.inf)
        next_cell: dict[tuple[int, int], tuple[int, int]] = {}
        costs[self.goal_cell] = 0.0
        queue = [(0.0, self.goal_cell)]
        while queue:
            current_cost, (row, column) = heapq.heappop(queue)
            if current_cost > costs[row, column]:
                continue
            for dr, dc, step in _NEIGHBORS:
                cell = (row + dr, column + dc)
                if not self.map.contains_cell(cell) or occupied[cell]:
                    continue
                if dr and dc and (
                    occupied[row + dr, column] or occupied[row, column + dc]
                ):
                    continue
                deficit = max(0.0, (preferred - clearance[cell]) / preferred)
                candidate = current_cost + step * (1.0 + weight * deficit * deficit)
                if candidate + 1e-12 < costs[cell]:
                    costs[cell] = candidate
                    next_cell[cell] = (row, column)
                    heapq.heappush(queue, (candidate, cell))
        return costs, next_cell

    def _reachable_cell(self, point: Point2D) -> tuple[int, int]:
        cell = self.map.world_to_grid(point)
        if not self.map.contains_cell(cell):
            raise PlanningError(
                PlanningErrorCode.OUT_OF_BOUNDS,
                "start is outside the map",
            )
        if math.isfinite(self.costs[cell]):
            return cell
        reach = math.ceil(self.map.safety_margin / self.map.resolution_m) + 1
        candidates = [
            (row, column)
            for row in range(cell[0] - reach, cell[0] + reach + 1)
            for column in range(cell[1] - reach, cell[1] + reach + 1)
            if self.map.contains_cell((row, column))
            and math.isfinite(self.costs[row, column])
            and all(
                self.cost_map.is_raw_free(sample)
                for sample in self.map.line_samples(
                    point, self.map.grid_to_world((row, column))
                )
            )
        ]
        if not candidates:
            code = (
                PlanningErrorCode.START_BLOCKED
                if self.cost_map.occupied[cell]
                else PlanningErrorCode.NO_PATH
            )
            raise PlanningError(code, f"no collision-free path to goal ({code})")
        return min(
            candidates,
            key=lambda item: (
                math.hypot(item[0] - cell[0], item[1] - cell[1]),
                item,
            ),
        )

    def cost_to_go(self, point: Point2D) -> float:
        cell = self.map.world_to_grid(point)
        return float(self.costs[cell]) if self.map.contains_cell(cell) else math.inf

    def progress_m(self, start: Point2D, end: Point2D) -> float:
        before = self.cost_to_go(start)
        after = self.cost_to_go(end)
        if math.isfinite(after) and not math.isfinite(before):
            return math.inf
        if not math.isfinite(before) or not math.isfinite(after):
            return -math.inf
        return (before - after) * self.map.resolution_m

    def plan(self, start: Point2D) -> GlobalPlan:
        cell = self._reachable_cell(start)
        cells = [cell]
        while cells[-1] != self.goal_cell:
            cells.append(self.next_cell[cells[-1]])
        raw = [self.map.grid_to_world(item) for item in cells]
        if self.map.world_to_grid(start) == cell:
            raw[0] = start
        else:
            raw.insert(0, start)
        raw[-1] = self.goal
        path = resample_path(
            self._smooth(tuple(raw)), self.config.waypoint_spacing_m
        )
        return GlobalPlan(tuple(raw), path, path_length(path))

    def _smooth(self, path: tuple[Point2D, ...]) -> tuple[Point2D, ...]:
        """Greedy line-of-sight shortcuts that never reduce clearance."""

        smoothed = [path[0]]
        anchor = 0
        while anchor < len(path) - 1:
            target = len(path) - 1
            while target > anchor + 1:
                segment = path[anchor : target + 1]
                original = min(
                    self.cost_map.clearance_at(point) for point in segment
                )
                shortcut = min(
                    self.cost_map.clearance_at(point)
                    for point in self.map.line_samples(path[anchor], path[target])
                )
                if shortcut + 1e-9 >= original and self.cost_map.line_is_free(
                    path[anchor], path[target]
                ):
                    break
                target -= 1
            smoothed.append(path[target])
            anchor = target
        return tuple(smoothed)


# -- map representation -------------------------------------------------------


@dataclass(frozen=True)
class MapBounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float


def _build_clearance(occupied: np.ndarray, resolution_m: float) -> np.ndarray:
    """Distance in metres from each cell to the nearest occupied cell centre."""

    if not occupied.any():
        return np.full(occupied.shape, math.inf)
    try:
        from scipy.ndimage import distance_transform_edt
    except ImportError:  # pragma: no cover - numpy-only fallback
        cells = np.argwhere(occupied)
        columns = np.arange(occupied.shape[1])
        distances = np.empty(occupied.shape)
        for row in range(occupied.shape[0]):
            delta_row = cells[:, 0] - row
            delta_column = cells[None, :, 1] - columns[:, None]
            distances[row] = np.sqrt(
                delta_row[None, :] ** 2 + delta_column**2
            ).min(axis=1)
        return distances * resolution_m
    return distance_transform_edt(~occupied) * resolution_m


class StaticMap:
    """Validated static geometry and its raw and inflated occupancy grids."""

    def __init__(
        self,
        payload: dict,
        *,
        config: NavigationConfig,
        world_name: str,
    ):
        self.config = config
        self.expected_world = world_name
        self.version = str(payload["version"])
        self.world = str(payload["world"])
        self.resolution_m = float(payload["resolution_m"])
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
            (self.bounds.max_x - self.bounds.min_x) / self.resolution_m
        )
        self.height = math.ceil(
            (self.bounds.max_y - self.bounds.min_y) / self.resolution_m
        )
        self.grid_bias = 0.5 * self.resolution_m
        self.raw_occupied = self._rasterize()
        self.clearance = _build_clearance(self.raw_occupied, self.resolution_m)
        self.occupied = self.clearance <= self.inflation_radius + self.grid_bias

    @classmethod
    def load(
        cls,
        path: Path,
        *,
        config: NavigationConfig,
        world_name: str,
    ) -> "StaticMap":
        return cls(
            json.loads(path.read_text()),
            config=config,
            world_name=world_name,
        )

    def _validate(self) -> None:
        if self.version != self.config.map_version:
            raise ValueError(f"unsupported navigation map version: {self.version}")
        if self.world != self.expected_world:
            raise ValueError(f"navigation map world mismatch: {self.world}")
        if self.resolution_m <= 0.0:
            raise ValueError("map resolution must be positive")
        if not (
            self.bounds.min_x < self.bounds.max_x
            and self.bounds.min_y < self.bounds.max_y
        ):
            raise ValueError("invalid map bounds")
        if len(self.boundary) < 3:
            raise ValueError("map boundary needs at least three points")
        if abs(self.robot_radius - self.config.robot_radius_m) > 1e-9:
            raise ValueError("map robot radius does not match config")
        if abs(self.safety_margin - self.config.safety_margin_m) > 1e-9:
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
        column = math.floor((point.x - self.bounds.min_x) / self.resolution_m)
        row = math.floor((point.y - self.bounds.min_y) / self.resolution_m)
        return row, column

    def grid_to_world(self, cell: tuple[int, int]) -> Point2D:
        row, column = cell
        return Point2D(
            self.bounds.min_x + (column + 0.5) * self.resolution_m,
            self.bounds.min_y + (row + 0.5) * self.resolution_m,
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
        samples = max(1, math.ceil(length / (self.resolution_m * 0.5)))
        for index in range(samples + 1):
            amount = index / samples
            yield Point2D(
                start.x + (end.x - start.x) * amount,
                start.y + (end.y - start.y) * amount,
            )

    def line_is_free(self, start: Point2D, end: Point2D) -> bool:
        return all(self.is_free(point) for point in self.line_samples(start, end))


class CostMap:
    """Static occupancy plus world-frame obstacles with time-based expiry."""

    def __init__(self, static_map: StaticMap, obstacle_ttl_seconds: float):
        if obstacle_ttl_seconds <= 0.0:
            raise ValueError("obstacle TTL must be positive")
        self.map = static_map
        self.obstacle_ttl_seconds = obstacle_ttl_seconds
        self._expires_at = np.zeros(
            (static_map.height, static_map.width), dtype=np.float64
        )
        self._dynamic_occupied = np.zeros_like(
            self._expires_at, dtype=np.bool_
        )
        self.version = 0
        self._rebuild()

    def _rebuild(self) -> None:
        self.raw_occupied = self.map.raw_occupied | self._dynamic_occupied
        self.clearance = _build_clearance(self.raw_occupied, self.map.resolution_m)
        self.occupied = self.clearance <= self.map.inflation_radius + self.map.grid_bias
        self.version += 1

    def update_world_obstacles(
        self, points: Sequence[Point2D], now: float
    ) -> bool:
        """Update obstacle expiry and return whether occupancy changed."""

        for point in points:
            cell = self.map.world_to_grid(point)
            if self.map.contains_cell(cell) and not self.map.occupied[cell]:
                self._expires_at[cell] = now + self.obstacle_ttl_seconds
        active = self._expires_at > now
        changed = bool(np.any(active != self._dynamic_occupied))
        if changed:
            self._dynamic_occupied = active
            self._rebuild()
        return changed

    def clear_dynamic_obstacles(self) -> None:
        if self._dynamic_occupied.any():
            self._expires_at.fill(0.0)
            self._dynamic_occupied.fill(False)
            self._rebuild()

    def is_free(self, point: Point2D) -> bool:
        cell = self.map.world_to_grid(point)
        return self.map.contains_cell(cell) and not bool(self.occupied[cell])

    def is_raw_free(self, point: Point2D) -> bool:
        cell = self.map.world_to_grid(point)
        return self.map.contains_cell(cell) and not bool(self.raw_occupied[cell])

    def static_is_free(self, point: Point2D) -> bool:
        return self.map.is_free(point)

    def clearance_at(self, point: Point2D) -> float:
        cell = self.map.world_to_grid(point)
        return float(self.clearance[cell]) if self.map.contains_cell(cell) else 0.0

    def line_is_free(self, start: Point2D, end: Point2D) -> bool:
        return all(self.is_free(point) for point in self.map.line_samples(start, end))
