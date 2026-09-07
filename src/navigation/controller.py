"""LiDAR processing and compact dynamic-window local planning."""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from config import CONFIG, NavigationConfig
from navigation.core import (
    ControlResult,
    Point2D,
    Pose2D,
    Velocity2D,
    distance,
    local_target,
    minimum_distance,
    point_segment_distance,
    wrap_angle,
)
from navigation.planner import Costmap, GoalField


class SensorDataError(RuntimeError):
    pass


def lidar_points(lidar: dict) -> tuple[Point2D, ...]:
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


def _world_point(pose: Pose2D, point: Point2D) -> Point2D:
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return Point2D(
        pose.x + point.x * cosine - point.y * sine,
        pose.y + point.x * sine + point.y * cosine,
    )


def lidar_points_in_world(lidar: dict, pose: Pose2D) -> tuple[Point2D, ...]:
    return tuple(_world_point(pose, point) for point in lidar_points(lidar))


def _distance_to_path(point: Point2D, path: Sequence[Point2D]) -> float:
    if len(path) == 1:
        return distance(point, path[0])
    return min(
        point_segment_distance(point, start, end)
        for start, end in zip(path, path[1:])
    )


def _is_known_map_return(costmap: Costmap, pose: Pose2D, point: Point2D) -> bool:
    return not costmap.map.is_free(_world_point(pose, point))


class DWALocalPlanner:
    """Sample reachable velocity commands and execute the safest best one.

    Candidate arcs are simulated only in memory.  A new command is selected
    for every fresh LiDAR scan, so successive choices can form multi-step
    manoeuvres without hard-coded spin or backup recovery sequences.
    """

    def __init__(self, costmap: Costmap, config: NavigationConfig = CONFIG.navigation):
        self.costmap = costmap
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
        self, velocity: Velocity2D
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
        world, body = [pose], [Point2D(0.0, 0.0)]
        local = Pose2D(0.0, 0.0, 0.0)
        for _ in range(steps):
            pose = Pose2D(
                pose.x + linear * math.cos(pose.yaw) * dt,
                pose.y + linear * math.sin(pose.yaw) * dt,
                wrap_angle(pose.yaw + angular * dt),
            )
            local = Pose2D(
                local.x + linear * math.cos(local.yaw) * dt,
                local.y + linear * math.sin(local.yaw) * dt,
                wrap_angle(local.yaw + angular * dt),
            )
            world.append(pose)
            body.append(local.point())
        return tuple(world), tuple(body)

    def _map_is_safe(
        self, trajectory: Sequence[Pose2D], current_clearance: float
    ) -> bool:
        # An inflated start is recoverable if every step stays outside raw
        # geometry and never moves closer to it.  From a free start, the full
        # inflated costmap remains a hard constraint.
        if not self.costmap.is_raw_free(trajectory[0].point()):
            return False
        if self.costmap.is_free(trajectory[0].point()):
            return all(self.costmap.is_free(item.point()) for item in trajectory)
        return all(
            self.costmap.is_raw_free(item.point())
            and self.costmap.clearance_at(item.point()) + 1e-9 >= current_clearance
            for item in trajectory
        )

    @staticmethod
    def _field_progress(
        goal_field: GoalField | None, start: Point2D, end: Point2D, scale: float
    ) -> float:
        if goal_field is None:
            return 0.0
        before = goal_field.cost_to_go(start)
        after = goal_field.cost_to_go(end)
        if math.isfinite(after) and not math.isfinite(before):
            return 1.0
        if not math.isfinite(before) or not math.isfinite(after):
            return -1.0
        # GoalField costs are in grid steps; convert their reduction to metres.
        change_m = (before - after) * goal_field.map.resolution
        return max(-1.0, min(1.0, change_m / max(scale, 1e-9)))

    def choose(
        self,
        *,
        pose: Pose2D,
        velocity: Velocity2D,
        global_path: Sequence[Point2D],
        lidar: dict,
        previous_command: Velocity2D,
        goal_field: GoalField | None = None,
    ) -> ControlResult:
        if not global_path:
            raise ValueError("global path is empty")
        config = self.config
        obstacles = lidar_points(lidar)
        # Static geometry is already represented by the map/costmap.  Do not
        # feed its own LiDAR returns back as temporary obstacles: near an
        # inflated pillar that feedback can make every forward arc look
        # unsafe and leave the robot rotating forever.  Unknown returns stay
        # active for dynamic-obstacle avoidance.
        obstacles = tuple(
            point
            for point in obstacles
            if not _is_known_map_return(self.costmap, pose, point)
        )
        position = pose.point()
        start_is_free = self.costmap.is_free(position)
        remaining = distance(position, global_path[-1])
        if remaining <= config.goal_tolerance_m:
            return ControlResult(
                Velocity2D(0.0, 0.0), (pose,), None, True, "goal_reached"
            )

        target = local_target(position, global_path, config.lookahead_distance_m)
        collision_radius = config.robot_radius_m + config.local_safety_margin_m
        center_clearance = minimum_distance(Point2D(0.0, 0.0), obstacles)
        admissible_lidar = min(collision_radius, center_clearance)
        current_map_clearance = self.costmap.clearance_at(position)
        linear_values, angular_values = self._window(velocity)
        best = None
        best_non_reverse = None
        best_progressing = None
        best_turning = None
        minimum_turn_rate = 0.5 * config.angular_acceleration_rps2 / config.control_hz
        map_rejected = 0
        lidar_rejected = 0
        evaluated = 0

        for linear in linear_values:
            for angular in angular_values:
                evaluated += 1
                world, body = self._rollout(pose, linear, angular)
                if not self._map_is_safe(world, current_map_clearance):
                    map_rejected += 1
                    continue
                arc_clearance = min(
                    minimum_distance(point, obstacles) for point in body
                )
                if arc_clearance + 1e-9 < admissible_lidar:
                    lidar_rejected += 1
                    continue

                end = world[-1]
                end_body = body[-1]
                field_progress = self._field_progress(
                    goal_field,
                    position,
                    end.point(),
                    config.lookahead_distance_m,
                )
                target_before = distance(position, target)
                target_progress = (
                    target_before - distance(end.point(), target)
                ) / max(config.lookahead_distance_m, 1e-9)
                progress = max(-1.0, min(1.0, max(field_progress, target_progress)))
                target_heading = math.atan2(target.y - end.y, target.x - end.x)
                travel_heading = (
                    end.yaw if linear >= 0.0 else wrap_angle(end.yaw + math.pi)
                )
                heading = (
                    math.cos(wrap_angle(target_heading - travel_heading)) + 1.0
                ) * 0.5
                endpoint_clearance = min(
                    minimum_distance(end_body, obstacles),
                    self.costmap.clearance_at(end.point()),
                )
                clearance = min(
                    1.0,
                    endpoint_clearance / max(config.astar_preferred_clearance_m, 1e-9),
                )
                max_linear_velocity = self.max_linear_velocity_mps
                speed = abs(linear) / max_linear_velocity
                path_deviation = min(
                    1.0,
                    _distance_to_path(end.point(), global_path)
                    / max(config.lookahead_distance_m, 1e-9),
                )
                smoothness = 0.5 * (
                    abs(linear - previous_command.linear)
                    / (max_linear_velocity - config.min_linear_velocity_mps)
                    + abs(angular - previous_command.angular)
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
                candidate = (tie_break, linear, angular, world, arc_clearance)
                if best is None or tie_break > best[0]:
                    best = candidate
                if linear >= 0.0 and (
                    best_non_reverse is None or tie_break > best_non_reverse[0]
                ):
                    best_non_reverse = candidate
                if linear > 0.0 and progress > 1e-4 and (
                    best_progressing is None or tie_break > best_progressing[0]
                ):
                    best_progressing = candidate
                if linear >= 0.0 and abs(angular) >= minimum_turn_rate and (
                    best_turning is None or tie_break > best_turning[0]
                ):
                    best_turning = candidate

        if best is None:
            reason = (
                "no_safe_map_trajectory"
                if map_rejected == evaluated
                else "no_safe_lidar_trajectory"
                if lidar_rejected
                else "no_safe_trajectory"
            )
            return ControlResult(
                Velocity2D(0.0, 0.0),
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
            else best_progressing or best_turning or best_non_reverse or best
        )
        _, linear, angular, world, arc_clearance = selected
        minimum_clearance = (
            None
            if math.isinf(arc_clearance)
            else max(0.0, arc_clearance - config.robot_radius_m)
        )
        reason = "selected" if start_is_free else "escaping"
        return ControlResult(
            Velocity2D(linear, angular), world, minimum_clearance, True, reason
        )
