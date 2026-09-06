"""Shared navigation data types and dependency-free 2D geometry."""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from enum import StrEnum


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float

    def as_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y}


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    def point(self) -> Point2D:
        return Point2D(self.x, self.y)


@dataclass(frozen=True)
class Velocity2D:
    linear: float
    angular: float

    def as_dict(self) -> dict[str, float]:
        return {"linear": self.linear, "angular": self.angular}


@dataclass(frozen=True)
class NavigationInput:
    """Validated sensor input consumed by one navigation control cycle."""

    pose: Pose2D
    velocity: Velocity2D
    lidar: dict


class NavigationState(StrEnum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    READY = "READY"
    NAVIGATING = "NAVIGATING"
    PAUSED = "PAUSED"
    REPLANNING = "REPLANNING"  # kept for API compatibility; no longer entered
    SUCCEEDED = "SUCCEEDED"
    CANCELLED = "CANCELLED"
    BLOCKED = "BLOCKED"
    ERROR = "ERROR"


@dataclass(frozen=True)
class PlanResult:
    raw_path: tuple[Point2D, ...]
    path: tuple[Point2D, ...]
    length_m: float


@dataclass(frozen=True)
class ControlResult:
    command: Velocity2D
    trajectory: tuple[Pose2D, ...]
    minimum_clearance_m: float | None
    safe: bool
    reason: str


def distance(first: Point2D, second: Point2D) -> float:
    return math.hypot(second.x - first.x, second.y - first.y)


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def point_in_polygon(point: Point2D, polygon: Sequence[Point2D]) -> bool:
    """Return True for points inside or on the edge of a simple polygon."""

    inside = False
    previous = polygon[-1]
    for current in polygon:
        if point_segment_distance(point, previous, current) <= 1e-9:
            return True
        crosses = (current.y > point.y) != (previous.y > point.y)
        if crosses:
            edge_x = (previous.x - current.x) * (point.y - current.y) / (
                previous.y - current.y
            ) + current.x
            if point.x < edge_x:
                inside = not inside
        previous = current
    return inside


def point_segment_distance(
    point: Point2D, start: Point2D, end: Point2D
) -> float:
    dx = end.x - start.x
    dy = end.y - start.y
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return distance(point, start)
    amount = max(
        0.0,
        min(
            1.0,
            ((point.x - start.x) * dx + (point.y - start.y) * dy)
            / length_squared,
        ),
    )
    projection = Point2D(start.x + amount * dx, start.y + amount * dy)
    return distance(point, projection)


def path_length(path: Sequence[Point2D]) -> float:
    return sum(distance(first, second) for first, second in zip(path, path[1:]))


def resample_path(path: Sequence[Point2D], spacing: float) -> tuple[Point2D, ...]:
    if spacing <= 0.0:
        raise ValueError("path spacing must be positive")
    if len(path) < 2:
        return tuple(path)
    result = [path[0]]
    for start, end in zip(path, path[1:]):
        segment_length = distance(start, end)
        divisions = max(1, math.ceil(segment_length / spacing))
        for index in range(1, divisions + 1):
            amount = index / divisions
            result.append(
                Point2D(
                    start.x + (end.x - start.x) * amount,
                    start.y + (end.y - start.y) * amount,
                )
            )
    result[-1] = path[-1]
    return tuple(result)


def closest_path_index(point: Point2D, path: Sequence[Point2D]) -> int:
    if not path:
        raise ValueError("path is empty")
    return min(range(len(path)), key=lambda index: distance(point, path[index]))


def local_target(
    point: Point2D, path: Sequence[Point2D], lookahead: float
) -> Point2D:
    index = closest_path_index(point, path)
    travelled = distance(point, path[index])
    for current in range(index, len(path) - 1):
        travelled += distance(path[current], path[current + 1])
        if travelled >= lookahead:
            return path[current + 1]
    return path[-1]


def minimum_distance(point: Point2D, others: Iterable[Point2D]) -> float:
    return min((distance(point, other) for other in others), default=math.inf)