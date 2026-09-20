"""HTTP adapter for the ROS 2/Nav2 gateway."""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from navigation.types import NavigationStatus, Point2D
from transport import AMRTransport


class Nav2GatewayError(RuntimeError):
    """Raised when the Nav2 gateway cannot satisfy an adapter request."""


class Nav2GatewayClient:
    """Small dependency-free JSON client used by the app container."""

    def __init__(self, base_url: str, timeout_seconds: float = 10.0):
        self.base_url = base_url.rstrip("/")
        self.timeout_seconds = timeout_seconds

    def get(self, path: str) -> dict:
        return self._request("GET", path)

    def post(self, path: str, payload: Mapping[str, object] | None = None) -> dict:
        return self._request("POST", path, payload)

    def _request(
        self,
        method: str,
        path: str,
        payload: Mapping[str, object] | None = None,
    ) -> dict:
        body = None
        headers = {"Accept": "application/json"}
        if payload is not None:
            body = json.dumps(payload).encode("utf-8")
            headers["Content-Type"] = "application/json"
        request = Request(
            f"{self.base_url}{path}",
            data=body,
            headers=headers,
            method=method,
        )
        try:
            with urlopen(request, timeout=self.timeout_seconds) as response:
                result = json.loads(response.read().decode("utf-8"))
        except HTTPError as error:
            detail = error.read().decode("utf-8", errors="replace")
            raise Nav2GatewayError(
                f"Nav2 gateway returned HTTP {error.code}: {detail}"
            ) from error
        except (URLError, TimeoutError, OSError, json.JSONDecodeError) as error:
            raise Nav2GatewayError(f"Nav2 gateway request failed: {error}") from error
        if not isinstance(result, dict):
            raise Nav2GatewayError("Nav2 gateway returned a non-object response")
        if result.get("ok") is False:
            raise Nav2GatewayError(str(result.get("error", "Nav2 request failed")))
        return result


def _status_payload(payload: Mapping[str, object]) -> NavigationStatus:
    """Normalize gateway responses to the existing Web UI schema."""

    nested = payload.get("status")
    source = nested if isinstance(nested, Mapping) else payload
    status: NavigationStatus = {
        "state": str(source.get("state", "ERROR")),
        "message": str(source.get("message", "")),
        "owns_control": bool(source.get("owns_control", False)),
        "start": source.get("start") if isinstance(source.get("start"), dict) else None,
        "goal": source.get("goal") if isinstance(source.get("goal"), dict) else None,
        "distance_to_goal_m": _optional_float(source.get("distance_to_goal_m")),
        "path_length_m": _optional_float(source.get("path_length_m")),
        "waypoint_count": int(source.get("waypoint_count", 0)),
        "global_path": list(source.get("global_path", [])),
        "command": source.get("command", {"linear": 0.0, "angular": 0.0}),
        "speed_mps": float(source.get("speed_mps", 0.0)),
        "minimum_clearance_m": _optional_float(source.get("minimum_clearance_m")),
        "error": source.get("error") if source.get("error") is None else str(source.get("error")),
    }
    return status


def _optional_float(value: object) -> float | None:
    if value is None:
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


class Nav2NavigationAdapter:
    """Keep FastAPI independent from ROS 2 and Nav2 Python dependencies."""

    def __init__(
        self,
        transport: AMRTransport,
        base_url: str,
        *,
        timeout_seconds: float = 10.0,
        gateway: Nav2GatewayClient | None = None,
    ):
        self.transport = transport
        self.gateway = gateway or Nav2GatewayClient(base_url, timeout_seconds)

    @property
    def owns_control(self) -> bool:
        try:
            return self.status()["owns_control"]
        except Nav2GatewayError:
            return False

    def plan(self, goal: Point2D) -> NavigationStatus:
        return _status_payload(
            self.gateway.post("/plan", {"goal": goal.to_dict()})
        )

    def is_goal_free(self, goal: Point2D) -> bool:
        response = self.gateway.post("/validate-goal", {"goal": goal.to_dict()})
        return bool(response.get("valid", False))

    def start(self) -> NavigationStatus:
        return _status_payload(self.gateway.post("/start", {}))

    def cancel(self) -> NavigationStatus:
        return _status_payload(self.gateway.post("/cancel", {}))

    def reset(self) -> NavigationStatus:
        self.gateway.post("/cancel", {})
        if not self.transport.reset():
            raise RuntimeError("Gazebo reset failed")
        return _status_payload(self.gateway.post("/reset", {}))

    def set_speed_mps(self, speed_mps: float) -> NavigationStatus:
        return _status_payload(
            self.gateway.post("/speed", {"speed_mps": speed_mps})
        )

    def status(self) -> NavigationStatus:
        return _status_payload(self.gateway.get("/status"))

    def set_initial_pose(self, pose: Point2D, yaw: float = 0.0) -> NavigationStatus:
        return _status_payload(
            self.gateway.post(
                "/initialpose",
                {"x": pose.x, "y": pose.y, "yaw": yaw},
            )
        )

    def close(self) -> None:
        try:
            self.gateway.post("/cancel", {})
        except Nav2GatewayError:
            pass
