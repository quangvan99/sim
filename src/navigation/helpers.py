"""Stateless geometry and sensor-boundary helpers."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence

from config import NavigationConfig
from navigation.types import NavigationInput, Point2D, Pose2D, Velocity


def distance(first: Point2D, second: Point2D) -> float:
    return math.hypot(second.x - first.x, second.y - first.y)


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def transform_point(pose: Pose2D, point: Point2D) -> Point2D:
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return Point2D(
        pose.x + point.x * cosine - point.y * sine,
        pose.y + point.x * sine + point.y * cosine,
    )


def transform_points(
    points: Sequence[Point2D], pose: Pose2D
) -> tuple[Point2D, ...]:
    return tuple(transform_point(pose, point) for point in points)


def point_in_polygon(point: Point2D, polygon: Sequence[Point2D]) -> bool:
    """Return True for points inside or on the edge of a simple polygon."""

    inside = False
    previous = polygon[-1]
    for current in polygon:
        if distance_to_segment(point, previous, current) <= 1e-9:
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


def distance_to_segment(
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


def nearest_index(point: Point2D, path: Sequence[Point2D]) -> int:
    if not path:
        raise ValueError("path is empty")
    return min(range(len(path)), key=lambda index: distance(point, path[index]))


def target_point(
    point: Point2D, path: Sequence[Point2D], lookahead: float
) -> Point2D:
    index = nearest_index(point, path)
    travelled = distance(point, path[index])
    for current in range(index, len(path) - 1):
        travelled += distance(path[current], path[current + 1])
        if travelled >= lookahead:
            return path[current + 1]
    return path[-1]


def min_distance(point: Point2D, others: Iterable[Point2D]) -> float:
    return min((distance(point, other) for other in others), default=math.inf)


# -- sensor boundary ----------------------------------------------------------


class SensorDataError(RuntimeError):
    pass


def _mapping(value: object, message: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise SensorDataError(message)
    return value


def parse_lidar_points(
    lidar: Mapping[str, object], config: NavigationConfig
) -> tuple[Point2D, ...]:
    """Convert a validated LaserScan snapshot to robot-frame points."""

    required = (
        "angle_min",
        "angle_step",
        "range_min",
        "range_max",
        "count",
        "ranges",
    )
    if any(lidar.get(name) is None for name in required):
        raise SensorDataError("LiDAR metadata is incomplete")
    try:
        count = int(lidar["count"])
        ranges = lidar["ranges"]
        invalid_count = int(lidar.get("invalid_count", 0))
        angle_min = float(lidar["angle_min"])
        angle_step = float(lidar["angle_step"])
        range_min = float(lidar["range_min"])
        range_max = float(lidar["range_max"])
    except (TypeError, ValueError) as error:
        raise SensorDataError("LiDAR metadata is invalid") from error
    if not isinstance(ranges, Sequence) or isinstance(ranges, (str, bytes)):
        raise SensorDataError("LiDAR ranges are invalid")
    if count != config.lidar_count or len(ranges) != count:
        raise SensorDataError(f"LiDAR scan must contain {config.lidar_count} rays")
    if invalid_count:
        raise SensorDataError("LiDAR scan contains NaN values")
    if not all(
        math.isfinite(value)
        for value in (angle_min, angle_step, range_min, range_max)
    ):
        raise SensorDataError("LiDAR metadata must be finite")
    if angle_step <= 0.0 or range_min <= 0.0 or range_max <= range_min:
        raise SensorDataError("LiDAR metadata is invalid")

    points = []
    for index, raw_range in enumerate(ranges):
        if raw_range is None:
            continue
        try:
            measured_range = float(raw_range)
        except (TypeError, ValueError) as error:
            raise SensorDataError("LiDAR ranges must be finite or null") from error
        if not math.isfinite(measured_range):
            raise SensorDataError("LiDAR ranges must be finite or null")
        measured_range = min(range_max, max(range_min, measured_range))
        if measured_range >= range_max - 1e-6:
            continue
        angle = angle_min + index * angle_step + config.lidar_sensor_yaw_rad
        points.append(
            Point2D(
                config.lidar_sensor_x_m + measured_range * math.cos(angle),
                config.lidar_sensor_y_m + measured_range * math.sin(angle),
            )
        )
    return tuple(points)


def parse_navigation_input(
    snapshot: Mapping[str, object], config: NavigationConfig
) -> NavigationInput:
    """Validate one transport snapshot and return its typed navigation view."""

    freshness = _mapping(snapshot.get("freshness"), "sensor freshness is unavailable")
    for name in ("lidar", "odometry", "model_pose"):
        item = _mapping(freshness.get(name), f"{name} has not been received")
        age = item.get("age_seconds")
        if not item.get("received") or age is None:
            raise SensorDataError(f"{name} has not been received")
        try:
            stale = float(age) > config.sensor_timeout_seconds
        except (TypeError, ValueError) as error:
            raise SensorDataError(f"{name} timestamp is invalid") from error
        if stale:
            raise SensorDataError(f"{name} is stale")

    sensors = _mapping(snapshot.get("sensors"), "navigation sensors are unavailable")
    odometry = _mapping(sensors.get("odometry"), "odometry is unavailable")
    model_pose = _mapping(sensors.get("model_pose"), "model pose is unavailable")
    lidar = _mapping(sensors.get("lidar"), "LiDAR is unavailable")
    try:
        linear_velocity = float(
            _mapping(odometry["linear_velocity"], "odometry is invalid")["x"]
        )
        angular_velocity = float(
            _mapping(odometry["angular_velocity"], "odometry is invalid")["z"]
        )
        position = _mapping(model_pose["position"], "model pose is invalid")
        pose = Pose2D(
            float(position["x"]),
            float(position["y"]),
            float(model_pose["yaw"]),
        )
    except (KeyError, TypeError, ValueError) as error:
        raise SensorDataError("model pose or odometry is invalid") from error
    if not all(
        math.isfinite(value)
        for value in (pose.x, pose.y, pose.yaw, linear_velocity, angular_velocity)
    ):
        raise SensorDataError("model pose or odometry is invalid")
    try:
        sequence = int(lidar.get("sequence", 0))
    except (TypeError, ValueError) as error:
        raise SensorDataError("LiDAR sequence is invalid") from error
    if sequence < 0:
        raise SensorDataError("LiDAR sequence is invalid")
    return NavigationInput(
        pose=pose,
        velocity=Velocity(linear_velocity, angular_velocity),
        scan_sequence=sequence,
        obstacles_robot=parse_lidar_points(lidar, config),
    )
