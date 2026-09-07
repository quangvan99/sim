"""LiDAR processing and compact dynamic-window local planning."""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from config import CONFIG, NavigationConfig
from navigation.core import (
    Decision,
    Point2D,
    Pose2D,
    Velocity,
    distance,
    target_point,
    min_distance,
    distance_to_segment,
    normalize_angle,
)
from navigation.planner import CostMap, Planner


class SensorDataError(RuntimeError):
    pass


def scan_points(lidar: dict) -> tuple[Point2D, ...]:
    """Convert one JSON-safe LaserScan snapshot to base-footprint points."""

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
    count = int(lidar["count"])
    ranges = lidar["ranges"]
    if count != CONFIG.navigation.lidar_count or len(ranges) != count:
        raise SensorDataError(
            f"LiDAR scan must contain {CONFIG.navigation.lidar_count} rays"
        )
    if int(lidar.get("invalid_count", 0)) != 0:
        raise SensorDataError("LiDAR scan contains NaN values")
    angle_min = float(lidar["angle_min"])
    angle_step = float(lidar["angle_step"])
    range_min = float(lidar["range_min"])
    range_max = float(lidar["range_max"])
    if not all(
        math.isfinite(value)
        for value in (angle_min, angle_step, range_min, range_max)
    ):
        raise SensorDataError("LiDAR metadata must be finite")
    if angle_step <= 0.0 or range_min <= 0.0 or range_max <= range_min:
        raise SensorDataError("LiDAR metadata is invalid")

    points = []
    sensor_yaw = CONFIG.navigation.lidar_sensor_yaw_rad
    for index, raw_range in enumerate(ranges):
        angle = angle_min + index * angle_step
        if raw_range is None:
            measured_range = range_max
            has_return = False
        else:
            measured_range = float(raw_range)
            if not math.isfinite(measured_range):
                raise SensorDataError("LiDAR ranges must be finite or null")
            measured_range = min(range_max, max(range_min, measured_range))
            has_return = measured_range < range_max - 1e-6
        if not has_return:
            continue
        ray_angle = angle + sensor_yaw
        points.append(
            Point2D(
                CONFIG.navigation.lidar_sensor_x_m
                + measured_range * math.cos(ray_angle),
                CONFIG.navigation.lidar_sensor_y_m
                + measured_range * math.sin(ray_angle),
            )
        )
    return tuple(points)


def _to_world_point(pose: Pose2D, point: Point2D) -> Point2D:
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return Point2D(
        pose.x + point.x * cosine - point.y * sine,
        pose.y + point.x * sine + point.y * cosine,
    )


def world_points(lidar: dict, pose: Pose2D) -> tuple[Point2D, ...]:
    return tuple(_to_world_point(pose, point) for point in scan_points(lidar))


def distance_to_path(point: Point2D, path: Sequence[Point2D]) -> float:
    if len(path) == 1:
        return distance(point, path[0])
    return min(
        distance_to_segment(point, start, end)
        for start, end in zip(path, path[1:])
    )


def _is_known_map_return(cost_map: CostMap, pose: Pose2D, point: Point2D) -> bool:
    return not cost_map.map.is_free(_to_world_point(pose, point))


class Controller:
    """Sample reachable velocity commands and execute the safest best one.

    Candidate arcs are simulated only in memory.  A new command is selected
    for every fresh LiDAR scan, so successive choices can form multi-step
    manoeuvres without hard-coded spin or backup recovery sequences.
    """

    def __init__(self, cost_map: CostMap, config: NavigationConfig = CONFIG.navigation):
        self.cost_map = cost_map
        self.config = config
        self._linear_velocity_cap = config.default_linear_velocity_mps
        if not (
            config.slow_linear_velocity_mps
            <= self._linear_velocity_cap
            <= config.fast_linear_velocity_mps
        ):
            raise ValueError("default speed must be within the configured speed range")

    @property
    def speed_mode(self) -> str:
        if self._linear_velocity_cap <= self.config.slow_linear_velocity_mps:
            return "slow"
        if self._linear_velocity_cap >= self.config.fast_linear_velocity_mps:
            return "fast"
        return "custom"

    @property
    def max_linear_velocity_mps(self) -> float:
        """Current forward-speed cap selected by the operator."""

        return self._linear_velocity_cap

    def set_speed_mps(self, speed_mps: float) -> None:
        """Set a bounded forward-speed cap selected by the operator."""

        if not math.isfinite(speed_mps):
            raise ValueError("speed must be finite")
        if not (
            self.config.slow_linear_velocity_mps
            <= speed_mps
            <= self.config.fast_linear_velocity_mps
        ):
            raise ValueError(
                "speed must be between "
                f"{self.config.slow_linear_velocity_mps:.2f} and "
                f"{self.config.fast_linear_velocity_mps:.2f} m/s"
            )
        self._linear_velocity_cap = float(speed_mps)

    def set_speed_mode(self, mode: str) -> None:
        """Select the conservative or brisk forward-speed profile."""

        if mode not in {"slow", "fast"}:
            raise ValueError("speed mode must be 'slow' or 'fast'")
        self.set_speed_mps(
            self.config.slow_linear_velocity_mps
            if mode == "slow"
            else self.config.fast_linear_velocity_mps
        )

    @staticmethod
    def _samples(low: float, high: float, count: int) -> tuple[float, ...]:
        values = {float(value) for value in np.linspace(low, high, count)}
        # Stop and rotate must remain available even outside acceleration bounds.
        values.add(0.0)
        return tuple(sorted(values))

    def _window(
        self, velocity: Velocity
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        dt = 1.0 / self.config.control_hz
        linear_low = max(
            self.config.min_linear_velocity_mps,
            velocity.linear - self.config.linear_acceleration_mps2 * dt,
        )
        linear_high = min(
            self.max_linear_velocity_mps,
            velocity.linear + self.config.linear_acceleration_mps2 * dt,
        )
        # If the operator lowers the cap while the robot is above it, allow a
        # bounded braking command instead of sampling an invalid interval.
        if linear_low > linear_high:
            linear_low = linear_high
        angular_low = max(
            -self.config.max_angular_velocity_rps,
            velocity.angular - self.config.angular_acceleration_rps2 * dt,
        )
        angular_high = min(
            self.config.max_angular_velocity_rps,
            velocity.angular + self.config.angular_acceleration_rps2 * dt,
        )
        return (
            self._samples(linear_low, linear_high, self.config.linear_samples),
            self._samples(angular_low, angular_high, self.config.angular_samples),
        )

    def _rollout(
        self, pose: Pose2D, linear: float, angular: float
    ) -> tuple[tuple[Pose2D, ...], tuple[Point2D, ...]]:
        steps = math.ceil(
            self.config.prediction_horizon_seconds / self.config.prediction_step_seconds
        )
        dt = self.config.prediction_step_seconds
        world_path, local_path = [pose], [Point2D(0.0, 0.0)]
        local = Pose2D(0.0, 0.0, 0.0)
        for _ in range(steps):
            pose = Pose2D(
                pose.x + linear * math.cos(pose.yaw) * dt,
                pose.y + linear * math.sin(pose.yaw) * dt,
                normalize_angle(pose.yaw + angular * dt),
            )
            local = Pose2D(
                local.x + linear * math.cos(local.yaw) * dt,
                local.y + linear * math.sin(local.yaw) * dt,
                normalize_angle(local.yaw + angular * dt),
            )
            world_path.append(pose)
            local_path.append(local.to_point())
        return tuple(world_path), tuple(local_path)

    def _is_trajectory_safe(
        self, trajectory: Sequence[Pose2D], current_clearance: float
    ) -> bool:
        # An inflated start is recoverable if every step stays outside raw
        # geometry and never moves closer to it.  From a free start, the full
        # inflated costmap remains a hard constraint.
        if not self.cost_map.is_raw_free(trajectory[0].to_point()):
            return False
        if self.cost_map.is_free(trajectory[0].to_point()):
            return all(self.cost_map.is_free(item.to_point()) for item in trajectory)
        return all(
            self.cost_map.is_raw_free(item.to_point())
            and self.cost_map.clearance_at(item.to_point()) + 1e-9 >= current_clearance
            for item in trajectory
        )

    @staticmethod
    def _field_progress(
        planner: Planner | None, start: Point2D, end: Point2D, scale: float
    ) -> float:
        if planner is None:
            return 0.0
        before = planner.cost_to_go(start)
        after = planner.cost_to_go(end)
        if math.isfinite(after) and not math.isfinite(before):
            return 1.0
        if not math.isfinite(before) or not math.isfinite(after):
            return -1.0
        # Planner costs are in grid steps; convert their reduction to metres.
        change_m = (before - after) * planner.map.resolution_m
        return max(-1.0, min(1.0, change_m / max(scale, 1e-9)))

    def command(
        self,
        *,
        pose: Pose2D,
        velocity: Velocity,
        path: Sequence[Point2D],
        lidar: dict,
        previous: Velocity,
        planner: Planner | None = None,
    ) -> Decision:
        if not path:
            raise ValueError("global path is empty")
        config = self.config
        obstacles = scan_points(lidar)
        # Static geometry is already represented by the map/costmap.  Do not
        # feed its own LiDAR returns back as temporary obstacles: near an
        # inflated pillar that feedback can make every forward arc look
        # unsafe and leave the robot rotating forever.  Unknown returns stay
        # active for dynamic-obstacle avoidance.
        obstacles = tuple(
            point
            for point in obstacles
            if not _is_known_map_return(self.cost_map, pose, point)
        )
        position = pose.to_point()
        start_is_free = self.cost_map.is_free(position)
        remaining = distance(position, path[-1])
        if remaining <= config.goal_tolerance_m:
            return Decision(
                Velocity(0.0, 0.0), (pose,), None, True, "goal_reached"
            )

        target = target_point(position, path, config.lookahead_distance_m)
        collision_radius = config.robot_radius_m + config.local_safety_margin_m
        center_clearance = min_distance(Point2D(0.0, 0.0), obstacles)
        admissible_lidar = min(collision_radius, center_clearance)
        current_map_clearance = self.cost_map.clearance_at(position)
        linear_samples, angular_samples = self._window(velocity)
        best = None
        best_forward = None
        best_progress = None
        best_turn = None
        min_turn_rate = 0.5 * config.angular_acceleration_rps2 / config.control_hz
        map_rejects = 0
        lidar_rejects = 0
        evaluated_count = 0

        for linear in linear_samples:
            for angular in angular_samples:
                evaluated_count += 1
                world_path, local_path = self._rollout(pose, linear, angular)
                if not self._is_trajectory_safe(world_path, current_map_clearance):
                    map_rejects += 1
                    continue
                arc_clearance = min(
                    min_distance(point, obstacles) for point in local_path
                )
                if arc_clearance + 1e-9 < admissible_lidar:
                    lidar_rejects += 1
                    continue

                end_pose = world_path[-1]
                end_local = local_path[-1]
                field_progress = self._field_progress(
                    planner,
                    position,
                    end_pose.to_point(),
                    config.lookahead_distance_m,
                )
                target_before = distance(position, target)
                target_progress = (
                    target_before - distance(end_pose.to_point(), target)
                ) / max(config.lookahead_distance_m, 1e-9)
                progress = max(-1.0, min(1.0, max(field_progress, target_progress)))
                target_heading = math.atan2(target.y - end_pose.y, target.x - end_pose.x)
                travel_heading = (
                    end_pose.yaw
                    if linear >= 0.0
                    else normalize_angle(end_pose.yaw + math.pi)
                )
                heading = (
                    math.cos(normalize_angle(target_heading - travel_heading)) + 1.0
                ) * 0.5
                endpoint_clearance = min(
                    min_distance(end_local, obstacles),
                    self.cost_map.clearance_at(end_pose.to_point()),
                )
                clearance = min(
                    1.0,
                    endpoint_clearance / max(config.astar_preferred_clearance_m, 1e-9),
                )
                max_linear_velocity = self.max_linear_velocity_mps
                speed = abs(linear) / max_linear_velocity
                path_deviation = min(
                    1.0,
                    distance_to_path(end_pose.to_point(), path)
                    / max(config.lookahead_distance_m, 1e-9),
                )
                smoothness = 0.5 * (
                    abs(linear - previous.linear)
                    / (max_linear_velocity - config.min_linear_velocity_mps)
                    + abs(angular - previous.angular)
                    / (2.0 * config.max_angular_velocity_rps)
                )
                stationary = (
                    0.35
                    if abs(linear) < 1e-9 and abs(angular) < 1e-9
                    else 0.0
                )
                score = (
                    config.score_progress * progress
                    + config.score_heading * heading
                    + config.score_clearance * clearance
                    + config.score_speed * speed
                    - config.score_path_deviation * path_deviation
                    - config.score_smoothness * smoothness
                    - (config.reverse_penalty if linear < 0.0 else 0.0)
                    - stationary
                )
                tie_break = (score, arc_clearance, progress, linear, -abs(angular))
                candidate = (tie_break, linear, angular, world_path, arc_clearance)
                if best is None or tie_break > best[0]:
                    best = candidate
                if linear >= 0.0 and (
                    best_forward is None or tie_break > best_forward[0]
                ):
                    best_forward = candidate
                if linear > 0.0 and progress > 1e-4 and (
                    best_progress is None or tie_break > best_progress[0]
                ):
                    best_progress = candidate
                if linear >= 0.0 and abs(angular) >= min_turn_rate and (
                    best_turn is None or tie_break > best_turn[0]
                ):
                    best_turn = candidate

        if best is None:
            reason = (
                "no_safe_map_trajectory"
                if map_rejects == evaluated_count
                else "no_safe_lidar_trajectory"
                if lidar_rejects
                else "no_safe_trajectory"
            )
            return Decision(
                Velocity(0.0, 0.0),
                (pose,),
                None
                if math.isinf(center_clearance)
                else max(0.0, center_clearance - config.robot_radius_m),
                False,
                reason,
            )

        # Reverse is an escape manoeuvre.  Once the robot is back in free map
        # space, prefer a safe forward/turning arc so it re-accelerates to the
        # selected default speed instead of backing along the whole global path.
        selected = (
            best
            if not start_is_free
            else best_progress or best_turn or best_forward or best
        )
        _, linear, angular, world_path, arc_clearance = selected
        clearance_m = (
            None
            if math.isinf(arc_clearance)
            else max(0.0, arc_clearance - config.robot_radius_m)
        )
        reason = "selected" if start_is_free else "escaping"
        return Decision(
            Velocity(linear, angular), world_path, clearance_m, True, reason
        )
