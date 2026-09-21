"""Adapter exposing the in-process custom navigator through the common API."""

from __future__ import annotations

from navigation.custom.navigator import CustomNavigator
from navigation.types import NavigationStatus, Point2D


class CustomNavigationAdapter:
    """Keep the existing Dijkstra + DWA navigator behind the adapter contract."""

    def __init__(self, navigator: CustomNavigator):
        self.navigator = navigator

    @property
    def owns_control(self) -> bool:
        return self.navigator.owns_control

    def plan(self, goal: Point2D) -> NavigationStatus:
        return self.navigator.plan(goal)

    def navigate(self, goal: Point2D) -> NavigationStatus:
        self.navigator.plan(goal)
        return self.navigator.start()

    def is_goal_free(self, goal: Point2D) -> bool:
        return self.navigator.is_goal_free(goal)

    def start(self) -> NavigationStatus:
        return self.navigator.start()

    def cancel(self) -> NavigationStatus:
        return self.navigator.cancel()

    def reset(self) -> NavigationStatus:
        return self.navigator.reset()

    def set_speed_mps(self, speed_mps: float) -> NavigationStatus:
        return self.navigator.set_speed_mps(speed_mps)

    def status(self) -> NavigationStatus:
        return self.navigator.status()

    def close(self) -> None:
        self.navigator.close()
