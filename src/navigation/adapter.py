"""Navigation backend contract shared by the HTTP server and adapters."""

from __future__ import annotations

from typing import Protocol

from navigation.types import NavigationStatus, Point2D


class NavigationAdapter(Protocol):
    """Common mission API exposed by every navigation backend."""

    @property
    def owns_control(self) -> bool: ...

    def plan(self, goal: Point2D) -> NavigationStatus: ...

    def is_goal_free(self, goal: Point2D) -> bool: ...

    def start(self) -> NavigationStatus: ...

    def cancel(self) -> NavigationStatus: ...

    def reset(self) -> NavigationStatus: ...

    def set_speed_mps(self, speed_mps: float) -> NavigationStatus: ...

    def status(self) -> NavigationStatus: ...

    def set_initial_pose(
        self, pose: Point2D, yaw: float = 0.0
    ) -> NavigationStatus: ...

    def close(self) -> None: ...
