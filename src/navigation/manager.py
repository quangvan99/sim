"""Threaded navigation state machine joining planning and control."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

from config import CONFIG
from navigation.controller import (
    DWALocalPlanner,
    SensorDataError,
    lidar_points_in_world,
)
from navigation.core import (
    NavigationInput,
    NavigationState,
    PlanResult,
    Point2D,
    Pose2D,
    Velocity2D,
    distance,
)
from navigation.planner import Costmap, GoalField, NavigationMap, PlanningError


class NavigationConflict(RuntimeError):
    pass


class NavigationSensorError(RuntimeError):
    pass


@dataclass
class _Mission:
    """Mutable state for one autonomous mission; a new instance per plan/reset."""

    state: NavigationState = NavigationState.IDLE
    message: str = "Select goal"
    error: str | None = None
    start: Point2D | None = None
    goal: Point2D | None = None
    current_position: Point2D | None = None
    goal_field: GoalField | None = None
    plan: PlanResult | None = None
    local_trajectory: tuple[Pose2D, ...] = ()
    local_reason: str | None = None
    executed_path: list[Point2D] = field(default_factory=list)
    command: Velocity2D = field(default_factory=lambda: Velocity2D(0.0, 0.0))
    minimum_clearance: float | None = None
    initial_goal_distance: float = 0.0
    progress_anchor: Point2D | None = None
    last_progress_at: float = 0.0
    replan_count: int = 0
    last_lidar_sequence: int = 0


class NavigationManager:
    """Own autonomous command publication and expose a JSON-safe status.

    Every cycle: fold LiDAR into the costmap, rebuild the cost-to-go field if
    the costmap changed, read the path from the robot's cell, track it.
    Locking: one ``self._lock`` region per public method and per cycle
    (RLock, so helpers may re-enter); only the transport read runs unlocked.
    """

    _CONTROL_STATES = frozenset(
        {
            NavigationState.PLANNING,
            NavigationState.READY,
            NavigationState.NAVIGATING,
            NavigationState.PAUSED,
            NavigationState.REPLANNING,
        }
    )

    def __init__(
        self,
        transport,
        *,
        navigation_map: NavigationMap | None = None,
        clock=time.monotonic,
        auto_start: bool = True,
    ):
        self.transport = transport
        self.map = navigation_map or NavigationMap.load()
        self.costmap = Costmap(
            self.map,
            round(
                CONFIG.navigation.temporary_obstacle_ttl_seconds
                * CONFIG.navigation.control_hz
            ),
        )
        self.controller = DWALocalPlanner(self.costmap)
        self.clock = clock
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._closed = False
        self._mission = _Mission(last_progress_at=self.clock())
        self._thread = threading.Thread(
            target=self._run, name="amr-navigation", daemon=True
        )
        if auto_start:
            self._thread.start()

    @property
    def state(self) -> NavigationState:
        with self._lock:
            return self._mission.state

    @property
    def owns_control(self) -> bool:
        return self.state in self._CONTROL_STATES

    def _set_state(
        self, state: NavigationState, message: str, error: str | None = None
    ) -> None:
        mission = self._mission
        mission.state = state
        mission.message = message
        mission.error = error

    def _read_navigation_input(self, snapshot: dict) -> NavigationInput:
        """Validate and normalize one transport snapshot for navigation."""

        freshness = snapshot.get("freshness", {})
        for sensor in ("lidar", "odometry", "model_pose"):
            item = freshness.get(sensor, {})
            age = item.get("age_seconds")
            if not item.get("received") or age is None:
                raise NavigationSensorError(f"{sensor} has not been received")
            if float(age) > CONFIG.navigation.sensor_timeout_seconds:
                raise NavigationSensorError(f"{sensor} is stale")
        sensors = snapshot.get("sensors", {})
        odometry = sensors.get("odometry")
        model_pose = sensors.get("model_pose")
        lidar = sensors.get("lidar")
        if (
            not isinstance(odometry, dict)
            or not isinstance(model_pose, dict)
            or not isinstance(lidar, dict)
        ):
            raise NavigationSensorError("navigation sensors are unavailable")
        try:
            linear_velocity = float(odometry["linear_velocity"]["x"])
            angular_velocity = float(odometry["angular_velocity"]["z"])
            pose = Pose2D(
                float(model_pose["position"]["x"]),
                float(model_pose["position"]["y"]),
                float(model_pose["yaw"]),
            )
            if not all(
                math.isfinite(value)
                for value in (pose.x, pose.y, pose.yaw, linear_velocity, angular_velocity)
            ):
                raise ValueError
            input_data = NavigationInput(
                pose=pose,
                velocity=Velocity2D(linear_velocity, angular_velocity),
                lidar=lidar,
            )
        except (KeyError, TypeError, ValueError) as error:
            raise NavigationSensorError("model pose or odometry is invalid") from error
        try:
            lidar_points_in_world(lidar, input_data.pose)
        except SensorDataError as error:
            raise NavigationSensorError(str(error)) from error
        return input_data

    # -- public mission API -----------------------------------------------------

    def plan(self, goal: Point2D) -> dict:
        if not all(math.isfinite(value) for value in (goal.x, goal.y)):
            raise ValueError("goal coordinates must be finite")
        with self._lock:
            if self._mission.state in {
                NavigationState.NAVIGATING,
                NavigationState.PAUSED,
                NavigationState.REPLANNING,
                NavigationState.PLANNING,
            }:
                raise NavigationConflict("navigation mission is active")
            self._stop_robot()
            self._set_state(NavigationState.PLANNING, "Planning")
        try:
            snapshot = self.transport.telemetry_snapshot()
            start = self._read_navigation_input(snapshot).pose.point()
            self.costmap.clear()
            goal_field = GoalField(self.costmap, goal)
            result = goal_field.path_from(start)
        except Exception as error:
            with self._lock:
                self._set_state(NavigationState.ERROR, "Plan failed", str(error))
            raise
        with self._lock:
            initial_goal_distance = max(distance(start, goal), 1e-9)
            self._mission = _Mission(
                state=NavigationState.READY,
                message="Ready",
                start=start,
                goal=goal,
                current_position=start,
                goal_field=goal_field,
                plan=result,
                executed_path=[start],
                initial_goal_distance=initial_goal_distance,
                progress_anchor=start,
                last_progress_at=self.clock(),
            )
            return self.status()

    def validate_goal(self, goal: Point2D) -> dict:
        """Validate a browser goal without changing navigation state."""

        if not all(math.isfinite(value) for value in (goal.x, goal.y)):
            raise ValueError("goal coordinates must be finite")
        valid = self.map.is_free(goal)
        return {
            "valid": valid,
            "code": None if valid else "GOAL_BLOCKED",
            "message": (
                "B is valid"
                if valid
                else "B overlaps a pillar or is outside the ground; choose B again"
            ),
        }

    def start(self) -> dict:
        with self._lock:
            if self._mission.state != NavigationState.READY:
                raise NavigationConflict("a ready plan is required")
            self._set_state(NavigationState.NAVIGATING, "Moving")
            self._mission.last_progress_at = self.clock()
            return self.status()

    def pause(self) -> dict:
        with self._lock:
            if self._mission.state != NavigationState.NAVIGATING:
                raise NavigationConflict("navigation is not running")
            self._stop_robot()
            self._set_state(NavigationState.PAUSED, "Paused")
            return self.status()

    def resume(self) -> dict:
        with self._lock:
            if self._mission.state != NavigationState.PAUSED:
                raise NavigationConflict("navigation is not paused")
            # The next cycle re-reads the path from wherever the robot is.
            self._mission.last_progress_at = self.clock()
            self._set_state(NavigationState.NAVIGATING, "Moving")
            return self.status()

    def cancel(self) -> dict:
        with self._lock:
            self._stop_robot()
            self._set_state(NavigationState.CANCELLED, "Stopped")
            self._mission.local_trajectory = ()
            self._mission.local_reason = None
            return self.status()

    def set_speed_mode(self, mode: str) -> dict:
        """Change the DWA forward-speed profile without restarting a mission."""

        with self._lock:
            self.controller.set_speed_mode(mode)
            return self.status()

    def set_speed_mps(self, speed_mps: float) -> dict:
        """Change the DWA forward-speed cap without restarting a mission."""

        with self._lock:
            self.controller.set_speed_mps(speed_mps)
            return self.status()

    def reset(self) -> bool:
        self.cancel()
        result = self.transport.reset()
        with self._lock:
            self.costmap.clear()
            self._mission = _Mission(
                state=NavigationState.IDLE if result else NavigationState.ERROR,
                message="Select goal" if result else "Reset failed",
                error=None if result else "Gazebo reset failed",
                last_progress_at=self.clock(),
            )
        return result

    def status(self) -> dict:
        with self._lock:
            mission = self._mission
            pose, goal, plan = mission.current_position, mission.goal, mission.plan
            distance_to_goal = (
                distance(pose, goal) if pose is not None and goal is not None else None
            )
            progress = (
                max(0.0, min(1.0, 1.0 - distance_to_goal / mission.initial_goal_distance))
                if distance_to_goal is not None and mission.initial_goal_distance > 0.0
                else 0.0
            )
            return {
                "state": mission.state.value,
                "message": mission.message,
                "owns_control": mission.state in self._CONTROL_STATES,
                "start": None if mission.start is None else mission.start.as_dict(),
                "goal": None if goal is None else goal.as_dict(),
                "distance_to_goal_m": distance_to_goal,
                "progress": progress,
                "path_length_m": None if plan is None else plan.length_m,
                "waypoint_count": 0 if plan is None else len(plan.path),
                "raw_path": [] if plan is None else [p.as_dict() for p in plan.raw_path],
                "global_path": [] if plan is None else [p.as_dict() for p in plan.path],
                "local_trajectory": [
                    {"x": item.x, "y": item.y} for item in mission.local_trajectory
                ],
                "local_reason": mission.local_reason,
                "executed_path": [point.as_dict() for point in mission.executed_path],
                "command": mission.command.as_dict(),
                "speed_mode": self.controller.speed_mode,
                "speed_mps": self.controller.max_linear_velocity_mps,
                "max_linear_velocity_mps": self.controller.max_linear_velocity_mps,
                "minimum_clearance_m": mission.minimum_clearance,
                "localization_source": "gazebo_model_pose",
                "replan_count": mission.replan_count,
                "error": mission.error,
            }

    # -- helpers ----------------------------------------------------------------

    def _append_executed(self, point: Point2D) -> None:
        with self._lock:
            mission = self._mission
            mission.current_position = point
            if not mission.executed_path or distance(
                mission.executed_path[-1], point
            ) >= CONFIG.navigation.executed_path_spacing_m:
                mission.executed_path.append(point)

    def _stop_robot(self) -> None:
        self._mission.command = Velocity2D(0.0, 0.0)
        try:
            self.transport.set_velocity(0.0, 0.0)
        except RuntimeError:
            pass

    def _fail(self, message: str) -> None:
        with self._lock:
            self._stop_robot()
            self._set_state(
                NavigationState.ERROR,
                "Sensor error" if "stale" in message else "Error",
                message,
            )

    def _blocked(self, message: str) -> None:
        with self._lock:
            self._stop_robot()
            self._set_state(NavigationState.BLOCKED, "Blocked", message)

    def _succeed(self, point: Point2D) -> None:
        with self._lock:
            self._append_executed(point)
            self._stop_robot()
            self._set_state(NavigationState.SUCCEEDED, "Goal reached")

    # -- control thread ---------------------------------------------------------

    def _control_cycle(self) -> None:
        input_data = self._read_navigation_input(self.transport.telemetry_snapshot())
        pose, velocity, lidar = input_data.pose, input_data.velocity, input_data.lidar
        position = pose.point()
        with self._lock:
            mission = self._mission
            self._append_executed(position)
            if mission.state != NavigationState.NAVIGATING:
                return
            if mission.goal is None or mission.goal_field is None or mission.plan is None:
                raise RuntimeError("active navigation plan is incomplete")
            if distance(position, mission.goal) <= CONFIG.navigation.goal_tolerance_m:
                self._succeed(position)
                return
            lidar_sequence = int(lidar.get("sequence", 0))
            if lidar_sequence and lidar_sequence == mission.last_lidar_sequence:
                # Same scan as last cycle: re-publish, do not re-decide.
                self.transport.set_velocity(mission.command.linear, mission.command.angular)
                return
            mission.last_lidar_sequence = lidar_sequence
            now = self.clock()

            # 1. Fold this scan into the costmap; rebuild the field on change.
            # Near-contact returns can surround a pose already inside static
            # inflation. Recover from the static map while old marks keep aging.
            costmap_points = (
                lidar_points_in_world(lidar, pose)
                if self.costmap.is_free(position)
                else ()
            )
            if self.costmap.update(costmap_points):
                if not self.costmap.path_is_free(mission.plan.path):
                    mission.replan_count += 1
                try:
                    mission.goal_field = GoalField(self.costmap, mission.goal)
                except PlanningError as error:
                    self._hold(mission, now, error.code)
                    return
            # 2. Read the path from where the robot is.
            try:
                mission.plan = mission.goal_field.path_from(position)
            except PlanningError as error:
                self._hold(mission, now, error.code)
                return
            # Progress = cost-to-go dropped since the anchor, both evaluated on
            # the *current* field so rebuilds cannot fake or hide progress.
            field_now = mission.goal_field
            if (
                field_now.cost_to_go(mission.progress_anchor)
                - field_now.cost_to_go(position)
                >= CONFIG.navigation.minimum_progress_m
            ):
                mission.progress_anchor = position
                mission.last_progress_at = now
            # 3. Track it.
            result = self.controller.choose(
                pose=pose,
                velocity=velocity,
                global_path=mission.plan.path,
                lidar=lidar,
                previous_command=mission.command,
                goal_field=mission.goal_field,
            )
            mission.local_trajectory = result.trajectory
            mission.local_reason = result.reason
            mission.minimum_clearance = result.minimum_clearance_m
            mission.command = result.command
            self.transport.set_velocity(result.command.linear, result.command.angular)
            # Backing away and turning are active recovery, not stalled motion.
            if result.reason == "escaping" or abs(result.command.angular) > 0.05:
                mission.last_progress_at = now
            mission.message = (
                "Avoiding obstacle"
                if result.minimum_clearance_m is not None and result.minimum_clearance_m < 0.30
                else "Moving"
            )
            if now - mission.last_progress_at > CONFIG.navigation.no_progress_timeout_seconds:
                self._blocked("no progress")

    def _hold(self, mission: _Mission, now: float, code: str) -> None:
        """No path this cycle: stand still; marks expire, so retry until timeout."""

        self._stop_robot()
        mission.local_trajectory = ()
        mission.local_reason = code
        mission.message = "Avoiding obstacle"
        if now - mission.last_progress_at > CONFIG.navigation.no_progress_timeout_seconds:
            self._blocked(f"no collision-free path to goal ({code})")

    def _run(self) -> None:
        period = 1.0 / CONFIG.navigation.heartbeat_hz
        while not self._stop_event.is_set():
            started = self.clock()
            if self.state == NavigationState.NAVIGATING:
                try:
                    self._control_cycle()
                except Exception as error:
                    self._fail(str(error))
            elapsed = self.clock() - started
            self._stop_event.wait(max(0.0, period - elapsed))

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop_event.set()
        with self._lock:
            self._stop_robot()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
