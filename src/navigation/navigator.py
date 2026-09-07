"""Threaded navigation state machine joining global and local planning."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Protocol

from config import CONFIG, NavigationConfig
from navigation.local_planner import LocalPlanner
from navigation.types import (
    DecisionReason,
    GoalValidation,
    GlobalPlan,
    NavigationStatus,
    Point2D,
    State,
    Velocity,
)
from navigation.helpers import (
    SensorDataError,
    distance,
    parse_navigation_input,
    transform_points,
)
from navigation.global_planner import (
    CostMap,
    GlobalPlanner,
    PlanningError,
    PlanningErrorCode,
    StaticMap,
)


class NavigationConflict(RuntimeError):
    pass


class NavigationTransport(Protocol):
    def telemetry_snapshot(self) -> Mapping[str, object]: ...

    def set_velocity(self, linear: float, angular: float) -> object: ...

    def reset(self) -> bool: ...


@dataclass
class _Mission:
    """Mutable state for one autonomous mission; a new instance per plan/reset."""

    state: State = State.IDLE
    message: str = "Select goal"
    error: str | None = None
    start: Point2D | None = None
    goal: Point2D | None = None
    position: Point2D | None = None
    planner: GlobalPlanner | None = None
    plan: GlobalPlan | None = None
    command: Velocity = field(default_factory=lambda: Velocity(0.0, 0.0))
    clearance_m: float | None = None
    anchor: Point2D | None = None
    last_progress: float = 0.0
    last_scan: int = 0


class Navigator:
    """Own autonomous command publication and expose a JSON-safe status.

    Every cycle: fold LiDAR into the costmap, rebuild the cost-to-go field if
    the costmap changed, read the path from the robot's cell, publish a command.
    Mission state is guarded by ``_lock``. Algorithms run under
    ``_algorithm_lock`` and commit only if the mission revision is unchanged.
    """

    _CONTROL_STATES = frozenset(
        {
            State.PLANNING,
            State.READY,
            State.NAVIGATING,
            State.PAUSED,
        }
    )

    def __init__(
        self,
        transport: NavigationTransport,
        *,
        static_map: StaticMap | None = None,
        config: NavigationConfig = CONFIG.navigation,
        clock: Callable[[], float] = time.monotonic,
        auto_start: bool = True,
    ):
        self.transport = transport
        self.config = config
        self.map = static_map or StaticMap.load(
            CONFIG.paths.navigation_map_path,
            config=config,
            world_name=CONFIG.simulation.world_name,
        )
        self.cost_map = CostMap(
            self.map, config.temporary_obstacle_ttl_seconds
        )
        self.local_planner = LocalPlanner(self.cost_map, config)
        self.clock = clock
        self._lock = threading.RLock()
        self._algorithm_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._closed = False
        self._revision = 0
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

    # -- public mission API -----------------------------------------------------

    def plan(self, goal: Point2D) -> NavigationStatus:
        if not all(math.isfinite(value) for value in (goal.x, goal.y)):
            raise ValueError("goal coordinates must be finite")
        with self._lock:
            if self._mission.state in {
                State.NAVIGATING,
                State.PAUSED,
                State.PLANNING,
                State.RESETTING,
            }:
                raise NavigationConflict("navigation mission is active")
            self._revision += 1
            revision = self._revision
            self._stop()
            self._set_state(State.PLANNING, "Planning")
        try:
            with self._algorithm_lock:
                nav_input = parse_navigation_input(
                    self.transport.telemetry_snapshot(), self.config
                )
                start = nav_input.pose.to_point()
                self.cost_map.clear_dynamic_obstacles()
                planner = GlobalPlanner(self.cost_map, goal, self.config)
                plan = planner.plan(start)
        except Exception as error:
            with self._lock:
                if revision == self._revision:
                    self._set_state(State.ERROR, "Plan failed", str(error))
            raise
        with self._lock:
            if revision != self._revision:
                raise NavigationConflict("planning was cancelled")
            self._mission = _Mission(
                state=State.READY,
                message="Ready",
                start=start,
                goal=goal,
                position=start,
                planner=planner,
                plan=plan,
                anchor=start,
                last_progress=self.clock(),
            )
            return self.status()

    def validate_goal(self, goal: Point2D) -> GoalValidation:
        """Validate a browser goal without changing navigation state."""

        if not all(math.isfinite(value) for value in (goal.x, goal.y)):
            raise ValueError("goal coordinates must be finite")
        valid = self.map.is_free(goal)
        return {
            "valid": valid,
            "code": None if valid else "GOAL_BLOCKED",
            "message": (
                "Goal is valid"
                if valid
                else "Goal is blocked or outside the map"
            ),
        }

    def start(self) -> NavigationStatus:
        with self._lock:
            if self._mission.state != State.READY:
                raise NavigationConflict("a ready plan is required")
            self._set_state(State.NAVIGATING, "Moving")
            self._mission.last_progress = self.clock()
            return self.status()

    def pause(self) -> NavigationStatus:
        with self._lock:
            if self._mission.state != State.NAVIGATING:
                raise NavigationConflict("navigation is not running")
            self._revision += 1
            self._stop()
            self._set_state(State.PAUSED, "Paused")
            return self.status()

    def resume(self) -> NavigationStatus:
        with self._lock:
            if self._mission.state != State.PAUSED:
                raise NavigationConflict("navigation is not paused")
            # The next cycle re-reads the path from wherever the robot is.
            self._mission.last_progress = self.clock()
            self._set_state(State.NAVIGATING, "Moving")
            return self.status()

    def cancel(self) -> NavigationStatus:
        with self._lock:
            if self._mission.state == State.RESETTING:
                raise NavigationConflict("reset is in progress")
            self._revision += 1
            self._stop()
            self._set_state(State.CANCELLED, "Stopped")
            self._mission.clearance_m = None
            return self.status()

    def set_speed_mode(self, mode: str) -> NavigationStatus:
        """Change the DWA forward-speed profile without restarting a mission."""

        with self._lock:
            self.local_planner.set_speed_mode(mode)
            return self.status()

    def set_speed_mps(self, speed_mps: float) -> NavigationStatus:
        """Change the DWA forward-speed cap without restarting a mission."""

        with self._lock:
            self.local_planner.set_speed_mps(speed_mps)
            return self.status()

    def reset(self) -> bool:
        with self._lock:
            if self._mission.state == State.RESETTING:
                raise NavigationConflict("reset is already in progress")
            self._revision += 1
            revision = self._revision
            self._stop()
            self._set_state(State.RESETTING, "Resetting")
        try:
            with self._algorithm_lock:
                reset_ok = self.transport.reset()
                self.cost_map.clear_dynamic_obstacles()
        except Exception as error:
            with self._lock:
                if revision == self._revision:
                    self._set_state(State.ERROR, "Reset failed", str(error))
            raise
        with self._lock:
            if revision != self._revision:
                return reset_ok
            self._mission = _Mission(
                state=State.IDLE if reset_ok else State.ERROR,
                message="Select goal" if reset_ok else "Reset failed",
                error=None if reset_ok else "Gazebo reset failed",
                last_progress=self.clock(),
            )
        return reset_ok

    def status(self) -> NavigationStatus:
        with self._lock:
            mission = self._mission
            pose, goal, plan = mission.position, mission.goal, mission.plan
            distance_to_goal = (
                distance(pose, goal) if pose is not None and goal is not None else None
            )
            return {
                "state": mission.state.value,
                "message": mission.message,
                "owns_control": mission.state in self._CONTROL_STATES,
                "start": None if mission.start is None else mission.start.to_dict(),
                "goal": None if goal is None else goal.to_dict(),
                "distance_to_goal_m": distance_to_goal,
                "path_length_m": None if plan is None else plan.length_m,
                "waypoint_count": 0 if plan is None else len(plan.path),
                "global_path": [] if plan is None else [p.to_dict() for p in plan.path],
                "command": mission.command.to_dict(),
                "speed_mode": self.local_planner.speed_mode,
                "speed_mps": self.local_planner.speed_mps,
                "minimum_clearance_m": mission.clearance_m,
                "error": mission.error,
            }

    # -- helpers ----------------------------------------------------------------

    def _stop(self) -> None:
        self._mission.command = Velocity(0.0, 0.0)
        try:
            self.transport.set_velocity(0.0, 0.0)
        except RuntimeError:
            pass

    def _fail(self, error: Exception) -> None:
        with self._lock:
            if self._mission.state != State.NAVIGATING:
                return
            self._revision += 1
            self._stop()
            self._set_state(
                State.ERROR,
                "Sensor error" if isinstance(error, SensorDataError) else "Error",
                str(error),
            )

    def _blocked(self, message: str) -> None:
        with self._lock:
            self._stop()
            self._set_state(State.BLOCKED, "Blocked", message)

    def _succeed(self, point: Point2D) -> None:
        with self._lock:
            self._mission.position = point
            self._stop()
            self._set_state(State.SUCCEEDED, "Goal reached")

    # -- control thread ---------------------------------------------------------

    def _step(self) -> None:
        nav_input = parse_navigation_input(
            self.transport.telemetry_snapshot(), self.config
        )
        pose = nav_input.pose
        position = pose.to_point()
        with self._lock:
            mission = self._mission
            mission.position = position
            if mission.state != State.NAVIGATING:
                return
            if mission.goal is None or mission.planner is None or mission.plan is None:
                raise RuntimeError("active navigation plan is incomplete")
            if distance(position, mission.goal) <= self.config.goal_tolerance_m:
                self._succeed(position)
                return
            scan_sequence = nav_input.scan_sequence
            if scan_sequence and scan_sequence == mission.last_scan:
                # Same scan as last cycle: re-publish, do not re-decide.
                self.transport.set_velocity(
                    mission.command.linear, mission.command.angular
                )
                return
            mission.last_scan = scan_sequence
            revision = self._revision
            planner = mission.planner
            previous = mission.command
            anchor = mission.anchor
        now = self.clock()

        # Costmap and planners are serialized, but mission controls
        # remain responsive while the algorithms run.
        with self._algorithm_lock:
            obstacle_points = (
                transform_points(nav_input.obstacles_robot, pose)
                if self.cost_map.is_free(position)
                else ()
            )
            try:
                self.cost_map.update_world_obstacles(obstacle_points, now)
                planner.refresh()
                plan = planner.plan(position)
            except PlanningError as error:
                self._handle_no_path(revision, now, error.code)
                return
            decision = self.local_planner.command(
                pose=pose,
                velocity=nav_input.velocity,
                path_world=plan.path,
                obstacles_robot=nav_input.obstacles_robot,
                previous=previous,
                progress_field=planner,
            )

        with self._lock:
            mission = self._mission
            if revision != self._revision or mission.state != State.NAVIGATING:
                return
            mission.position = position
            mission.plan = plan
            if (
                planner.progress_m(anchor, position)
                >= self.config.minimum_progress_m
            ):
                mission.anchor = position
                mission.last_progress = now
            mission.clearance_m = decision.clearance_m
            mission.command = (
                decision.command if decision.safe else Velocity(0.0, 0.0)
            )
            self.transport.set_velocity(
                mission.command.linear, mission.command.angular
            )
            # Backing away and turning are active recovery, not stalled motion.
            if (
                decision.reason is DecisionReason.ESCAPING
                or abs(decision.command.angular) > 0.05
            ):
                mission.last_progress = now
            mission.message = (
                "Avoiding obstacle"
                if decision.clearance_m is not None and decision.clearance_m < 0.30
                else "Moving"
            )
            if now - mission.last_progress > self.config.no_progress_timeout_seconds:
                self._blocked("no progress")

    def _handle_no_path(
        self, revision: int, now: float, code: PlanningErrorCode
    ) -> None:
        """No path this cycle: stand still; marks expire, so retry until timeout."""

        with self._lock:
            mission = self._mission
            if revision != self._revision or mission.state != State.NAVIGATING:
                return
            self._stop()
            mission.message = "Avoiding obstacle"
            mission.clearance_m = None
            if now - mission.last_progress > self.config.no_progress_timeout_seconds:
                self._set_state(
                    State.BLOCKED,
                    "Blocked",
                    f"no collision-free path to goal ({code})",
                )

    def _loop(self) -> None:
        period = 1.0 / self.config.heartbeat_hz
        while not self._stop_event.is_set():
            started = self.clock()
            if self.state == State.NAVIGATING:
                try:
                    self._step()
                except Exception as error:
                    self._fail(error)
            elapsed = self.clock() - started
            self._stop_event.wait(max(0.0, period - elapsed))

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop_event.set()
        with self._lock:
            self._revision += 1
            self._stop()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
