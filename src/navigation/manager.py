"""Threaded navigation state machine joining planning and control."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

from config import CONFIG
from navigation.controller import (
    Controller,
    SensorDataError,
    world_points,
)
from navigation.core import (
    Input,
    State,
    Plan,
    Point2D,
    Pose2D,
    Velocity,
    distance,
)
from navigation.planner import CostMap, Planner, Map, PlanningError


class NavigationConflict(RuntimeError):
    pass


class NavigationSensorError(RuntimeError):
    pass


@dataclass
class _Mission:
    """Mutable state for one autonomous mission; a new instance per plan/reset."""

    state: State = State.IDLE
    message: str = "Select goal"
    error: str | None = None
    start: Point2D | None = None
    goal: Point2D | None = None
    position: Point2D | None = None
    planner: Planner | None = None
    plan: Plan | None = None
    trajectory: tuple[Pose2D, ...] = ()
    reason: str | None = None
    history: list[Point2D] = field(default_factory=list)
    command: Velocity = field(default_factory=lambda: Velocity(0.0, 0.0))
    clearance_m: float | None = None
    start_distance_m: float = 0.0
    anchor: Point2D | None = None
    last_progress: float = 0.0
    replans: int = 0
    last_scan: int = 0


class Navigator:
    """Own autonomous command publication and expose a JSON-safe status.

    Every cycle: fold LiDAR into the costmap, rebuild the cost-to-go field if
    the costmap changed, read the path from the robot's cell, track it.
    Locking: one ``self._lock`` region per public method and per cycle
    (RLock, so helpers may re-enter); only the transport read runs unlocked.
    """

    _CONTROL_STATES = frozenset(
        {
            State.PLANNING,
            State.READY,
            State.NAVIGATING,
            State.PAUSED,
            State.REPLANNING,
        }
    )

    def __init__(
        self,
        transport,
        *,
        static_map: Map | None = None,
        clock=time.monotonic,
        auto_start: bool = True,
    ):
        self.transport = transport
        self.map = static_map or Map.load()
        self.cost_map = CostMap(
            self.map,
            round(
                CONFIG.navigation.temporary_obstacle_ttl_seconds
                * CONFIG.navigation.control_hz
            ),
        )
        self.controller = Controller(self.cost_map)
        self.clock = clock
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._closed = False
        self._mission = _Mission(last_progress=self.clock())
        self._thread = threading.Thread(
            target=self._loop, name="amr-navigation", daemon=True
        )
        if auto_start:
            self._thread.start()

    @property
    def state(self) -> State:
        with self._lock:
            return self._mission.state

    @property
    def owns_control(self) -> bool:
        return self.state in self._CONTROL_STATES

    def _set_state(
        self, state: State, message: str, error: str | None = None
    ) -> None:
        mission = self._mission
        mission.state = state
        mission.message = message
        mission.error = error

    def _parse_input(self, snapshot: dict) -> Input:
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
            nav_input = Input(
                pose=pose,
                velocity=Velocity(linear_velocity, angular_velocity),
                lidar=lidar,
            )
        except (KeyError, TypeError, ValueError) as error:
            raise NavigationSensorError("model pose or odometry is invalid") from error
        try:
            world_points(lidar, nav_input.pose)
        except SensorDataError as error:
            raise NavigationSensorError(str(error)) from error
        return nav_input

    # -- public mission API -----------------------------------------------------

    def plan(self, goal: Point2D) -> dict:
        if not all(math.isfinite(value) for value in (goal.x, goal.y)):
            raise ValueError("goal coordinates must be finite")
        with self._lock:
            if self._mission.state in {
                State.NAVIGATING,
                State.PAUSED,
                State.REPLANNING,
                State.PLANNING,
            }:
                raise NavigationConflict("navigation mission is active")
            self._stop()
            self._set_state(State.PLANNING, "Planning")
        try:
            snapshot = self.transport.telemetry_snapshot()
            start = self._parse_input(snapshot).pose.to_point()
            self.cost_map.clear_dynamic_obstacles()
            planner = Planner(self.cost_map, goal)
            plan = planner.plan(start)
        except Exception as error:
            with self._lock:
                self._set_state(State.ERROR, "Plan failed", str(error))
            raise
        with self._lock:
            start_distance_m = max(distance(start, goal), 1e-9)
            self._mission = _Mission(
                state=State.READY,
                message="Ready",
                start=start,
                goal=goal,
                position=start,
                planner=planner,
                plan=plan,
                history=[start],
                start_distance_m=start_distance_m,
                anchor=start,
                last_progress=self.clock(),
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
            if self._mission.state != State.READY:
                raise NavigationConflict("a ready plan is required")
            self._set_state(State.NAVIGATING, "Moving")
            self._mission.last_progress = self.clock()
            return self.status()

    def pause(self) -> dict:
        with self._lock:
            if self._mission.state != State.NAVIGATING:
                raise NavigationConflict("navigation is not running")
            self._stop()
            self._set_state(State.PAUSED, "Paused")
            return self.status()

    def resume(self) -> dict:
        with self._lock:
            if self._mission.state != State.PAUSED:
                raise NavigationConflict("navigation is not paused")
            # The next cycle re-reads the path from wherever the robot is.
            self._mission.last_progress = self.clock()
            self._set_state(State.NAVIGATING, "Moving")
            return self.status()

    def cancel(self) -> dict:
        with self._lock:
            self._stop()
            self._set_state(State.CANCELLED, "Stopped")
            self._mission.trajectory = ()
            self._mission.reason = None
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
        reset_ok = self.transport.reset()
        with self._lock:
            self.cost_map.clear_dynamic_obstacles()
            self._mission = _Mission(
                state=State.IDLE if reset_ok else State.ERROR,
                message="Select goal" if reset_ok else "Reset failed",
                error=None if reset_ok else "Gazebo reset failed",
                last_progress=self.clock(),
            )
        return reset_ok

    def status(self) -> dict:
        with self._lock:
            mission = self._mission
            pose, goal, plan = mission.position, mission.goal, mission.plan
            distance_to_goal = (
                distance(pose, goal) if pose is not None and goal is not None else None
            )
            progress = (
                max(0.0, min(1.0, 1.0 - distance_to_goal / mission.start_distance_m))
                if distance_to_goal is not None and mission.start_distance_m > 0.0
                else 0.0
            )
            return {
                "state": mission.state.value,
                "message": mission.message,
                "owns_control": mission.state in self._CONTROL_STATES,
                "start": None if mission.start is None else mission.start.to_dict(),
                "goal": None if goal is None else goal.to_dict(),
                "distance_to_goal_m": distance_to_goal,
                "progress": progress,
                "path_length_m": None if plan is None else plan.length_m,
                "waypoint_count": 0 if plan is None else len(plan.path),
                "raw_path": [] if plan is None else [p.to_dict() for p in plan.raw],
                "global_path": [] if plan is None else [p.to_dict() for p in plan.path],
                "local_trajectory": [
                    {"x": item.x, "y": item.y} for item in mission.trajectory
                ],
                "local_reason": mission.reason,
                "executed_path": [point.to_dict() for point in mission.history],
                "command": mission.command.to_dict(),
                "speed_mode": self.controller.speed_mode,
                "speed_mps": self.controller.max_linear_velocity_mps,
                "max_linear_velocity_mps": self.controller.max_linear_velocity_mps,
                "minimum_clearance_m": mission.clearance_m,
                "localization_source": "gazebo_model_pose",
                "replan_count": mission.replans,
                "error": mission.error,
            }

    # -- helpers ----------------------------------------------------------------

    def _track(self, point: Point2D) -> None:
        with self._lock:
            mission = self._mission
            mission.position = point
            if not mission.history or distance(
                mission.history[-1], point
            ) >= CONFIG.navigation.executed_path_spacing_m:
                mission.history.append(point)

    def _stop(self) -> None:
        self._mission.command = Velocity(0.0, 0.0)
        try:
            self.transport.set_velocity(0.0, 0.0)
        except RuntimeError:
            pass

    def _fail(self, message: str) -> None:
        with self._lock:
            self._stop()
            self._set_state(
                State.ERROR,
                "Sensor error" if "stale" in message else "Error",
                message,
            )

    def _blocked(self, message: str) -> None:
        with self._lock:
            self._stop()
            self._set_state(State.BLOCKED, "Blocked", message)

    def _succeed(self, point: Point2D) -> None:
        with self._lock:
            self._track(point)
            self._stop()
            self._set_state(State.SUCCEEDED, "Goal reached")

    # -- control thread ---------------------------------------------------------

    def _step(self) -> None:
        nav_input = self._parse_input(self.transport.telemetry_snapshot())
        pose, velocity, lidar = nav_input.pose, nav_input.velocity, nav_input.lidar
        position = pose.to_point()
        with self._lock:
            mission = self._mission
            self._track(position)
            if mission.state != State.NAVIGATING:
                return
            if mission.goal is None or mission.planner is None or mission.plan is None:
                raise RuntimeError("active navigation plan is incomplete")
            if distance(position, mission.goal) <= CONFIG.navigation.goal_tolerance_m:
                self._succeed(position)
                return
            scan_sequence = int(lidar.get("sequence", 0))
            if scan_sequence and scan_sequence == mission.last_scan:
                # Same scan as last cycle: re-publish, do not re-decide.
                self.transport.set_velocity(mission.command.linear, mission.command.angular)
                return
            mission.last_scan = scan_sequence
            now = self.clock()

            # 1. Fold this scan into the costmap; rebuild the field on change.
            # Near-contact returns can surround a pose already inside static
            # inflation. Recover from the static map while old marks keep aging.
            obstacle_points = (
                world_points(lidar, pose)
                if self.cost_map.is_free(position)
                else ()
            )
            if self.cost_map.update_obstacles(obstacle_points):
                if not self.cost_map.path_is_free(mission.plan.path):
                    mission.replans += 1
                try:
                    mission.planner = Planner(self.cost_map, mission.goal)
                except PlanningError as error:
                    self._handle_no_path(mission, now, error.code)
                    return
            # 2. Read the path from where the robot is.
            try:
                mission.plan = mission.planner.plan(position)
            except PlanningError as error:
                self._handle_no_path(mission, now, error.code)
                return
            # Progress = cost-to-go dropped since the anchor, both evaluated on
            # the *current* field so rebuilds cannot fake or hide progress.
            current_planner = mission.planner
            if (
                current_planner.cost_to_go(mission.anchor)
                - current_planner.cost_to_go(position)
                >= CONFIG.navigation.minimum_progress_m
            ):
                mission.anchor = position
                mission.last_progress = now
            # 3. Track it.
            decision = self.controller.command(
                pose=pose,
                velocity=velocity,
                path=mission.plan.path,
                lidar=lidar,
                previous=mission.command,
                planner=mission.planner,
            )
            mission.trajectory = decision.path
            mission.reason = decision.reason
            mission.clearance_m = decision.clearance_m
            mission.command = decision.command
            self.transport.set_velocity(decision.command.linear, decision.command.angular)
            # Backing away and turning are active recovery, not stalled motion.
            if decision.reason == "escaping" or abs(decision.command.angular) > 0.05:
                mission.last_progress = now
            mission.message = (
                "Avoiding obstacle"
                if decision.clearance_m is not None and decision.clearance_m < 0.30
                else "Moving"
            )
            if now - mission.last_progress > CONFIG.navigation.no_progress_timeout_seconds:
                self._blocked("no progress")

    def _handle_no_path(self, mission: _Mission, now: float, code: str) -> None:
        """No path this cycle: stand still; marks expire, so retry until timeout."""

        self._stop()
        mission.trajectory = ()
        mission.reason = code
        mission.message = "Avoiding obstacle"
        if now - mission.last_progress > CONFIG.navigation.no_progress_timeout_seconds:
            self._blocked(f"no collision-free path to goal ({code})")

    def _loop(self) -> None:
        period = 1.0 / CONFIG.navigation.heartbeat_hz
        while not self._stop_event.is_set():
            started = self.clock()
            if self.state == State.NAVIGATING:
                try:
                    self._step()
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
            self._stop()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
