"""Expose the small navigation contract used by the FastAPI container."""

from __future__ import annotations

import json
import logging
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node

try:
    from nav2_msgs.srv import SpeedLimit
except ImportError:  # Older Nav2 releases may not provide this service.
    SpeedLimit = None

try:
    import tf2_ros
    from rclpy.time import Time
except ImportError:  # Keep the module easy to syntax-check without ROS.
    tf2_ros = None
    Time = None


LOGGER = logging.getLogger("nav2_gateway")


class GatewayRequestError(RuntimeError):
    """Raised for an invalid or currently unavailable Nav2 operation."""


def _wait_future(future, timeout_seconds: float):
    done = threading.Event()
    result: list[Any] = []
    failure: list[BaseException] = []

    def complete(done_future):
        try:
            result.append(done_future.result())
        except BaseException as error:  # Keep the HTTP thread from hanging.
            failure.append(error)
        finally:
            done.set()

    future.add_done_callback(complete)
    if not done.wait(timeout_seconds):
        raise GatewayRequestError("ROS 2 action or service timed out")
    if failure:
        raise GatewayRequestError(str(failure[0])) from failure[0]
    return result[0] if result else None


def _distance(first: dict[str, float] | None, second: dict[str, float] | None):
    if first is None or second is None:
        return None
    return math.hypot(first["x"] - second["x"], first["y"] - second["y"])


def _yaw_error(first: dict[str, float] | None, second: dict[str, float] | None):
    if first is None or second is None or second.get("yaw") is None:
        return None
    return abs((first["yaw"] - second["yaw"] + math.pi) % (2.0 * math.pi) - math.pi)


class Nav2Gateway(Node):
    """ROS node and thread-safe HTTP facade for Nav2 actions."""

    def __init__(self):
        super().__init__("amr_nav2_gateway")
        self.declare_parameter("http_host", "0.0.0.0")
        self.declare_parameter("http_port", 8010)
        self.declare_parameter("action_timeout_seconds", 10.0)
        self.declare_parameter("goal_xy_tolerance", 0.005)
        self.declare_parameter("goal_yaw_tolerance", 0.02)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("initial_pose_x", -2.0)
        self.declare_parameter("initial_pose_y", -0.5)
        self.declare_parameter("initial_pose_yaw", 0.0)
        host = str(self.get_parameter("http_host").value)
        port = int(self.get_parameter("http_port").value)
        self._timeout = float(self.get_parameter("action_timeout_seconds").value)
        self._goal_xy_tolerance = float(
            self.get_parameter("goal_xy_tolerance").value
        )
        self._goal_yaw_tolerance = float(
            self.get_parameter("goal_yaw_tolerance").value
        )
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._lock = threading.Lock()
        self._generation = 0
        self._goal_handle = None
        self._settling_generation: int | None = None
        self._settled_cycles = 0
        self._current_pose: dict[str, float] | None = None
        self._planned_goal: dict[str, float] | None = None
        self._path: list[dict[str, float]] = []
        self._status = self._new_status()
        self._nav2_ready = False
        self._odom_ready = False
        self._tf_ready = False
        self._map_to_odom: dict[str, float] | None = None
        self._reanchor_localization = True
        self._startup_pose = {
            "x": float(self.get_parameter("initial_pose_x").value),
            "y": float(self.get_parameter("initial_pose_y").value),
            "yaw": float(self.get_parameter("initial_pose_yaw").value),
        }
        self._desired_pose = dict(self._startup_pose)

        self._compute_client = ActionClient(
            self, ComputePathToPose, "/compute_path_to_pose"
        )
        self._navigate_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self._global_clear_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
        )
        self._local_clear_client = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
        )
        self._speed_client = (
            self.create_client(SpeedLimit, "/controller_server/set_speed_limit")
            if SpeedLimit is not None
            else None
        )
        self._speed_service_type = SpeedLimit
        self._zero_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(
            Path, "/received_global_plan", self._on_global_plan, 10
        )
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self._tf_buffer = tf2_ros.Buffer() if tf2_ros is not None else None
        self._tf_listener = (
            tf2_ros.TransformListener(self._tf_buffer, self)
            if self._tf_buffer is not None
            else None
        )
        self.create_timer(1.0, self._refresh_health)
        self.create_timer(1.0, self._ensure_startup_localization)
        self.create_timer(0.05, self._check_goal_settled)
        handler = type("GatewayHandler", (_GatewayHandler,), {"gateway": self})
        self._http = ThreadingHTTPServer((host, port), handler)
        self._http_thread = threading.Thread(
            target=self._http.serve_forever,
            name="nav2-gateway-http",
            daemon=True,
        )
        self._http_thread.start()
        LOGGER.info("Nav2 gateway listening on %s:%d", host, port)

    def destroy_node(self):
        self._http.shutdown()
        self._http.server_close()
        self._http_thread.join(timeout=2.0)
        return super().destroy_node()

    # -- HTTP request handling -------------------------------------------------

    def handle_http(self, method: str, path: str, payload: dict | None):
        route = path.split("?", 1)[0]
        try:
            if method == "GET" and route == "/health":
                return 200, self.health()
            if method == "GET" and route == "/status":
                return 200, self.status()
            if method == "POST" and route == "/plan":
                return 200, self.plan(payload)
            if method == "POST" and route == "/validate-goal":
                return 200, self.validate_goal(payload)
            if method == "POST" and route == "/start":
                return 200, self.start()
            if method == "POST" and route == "/cancel":
                return 200, self.cancel()
            if method == "POST" and route == "/reset":
                return 200, self.reset()
            if method == "POST" and route == "/initialpose":
                return 200, self.set_initial_pose(payload)
            if method == "POST" and route == "/speed":
                return 200, self.set_speed(payload)
            return 404, {"ok": False, "error": "endpoint not found"}
        except GatewayRequestError as error:
            return 409, {"ok": False, "error": str(error)}
        except (KeyError, TypeError, ValueError) as error:
            return 422, {"ok": False, "error": str(error)}
        except Exception as error:  # Keep one bad request from killing the node.
            LOGGER.exception("Nav2 gateway request failed")
            return 500, {"ok": False, "error": str(error)}

    def health(self):
        return {
            "ok": True,
            "ready": self._nav2_ready and self._odom_ready and self._tf_ready,
            "nav2_ready": self._nav2_ready,
            "odom_ready": self._odom_ready,
            "tf_ready": self._tf_ready,
        }

    def status(self):
        with self._lock:
            status = dict(self._status)
            status["global_path"] = list(self._path)
            status["start"] = self._current_pose
            status["distance_to_goal_m"] = _distance(
                self._current_pose, self._planned_goal
            )
            status["yaw_error_to_goal_rad"] = _yaw_error(
                self._current_pose, self._planned_goal
            )
            return {"ok": True, **status}

    def plan(self, payload: dict | None):
        goal = self._goal(payload)
        with self._lock:
            if self._status["state"] in {"PLANNING", "NAVIGATING"}:
                raise GatewayRequestError("navigation is still active")
            self._set_state_locked("PLANNING", "Computing Nav2 path")
        try:
            path = self._compute_path(goal)
        except Exception as error:
            with self._lock:
                self._set_state_locked("ERROR", "Nav2 planning failed", str(error))
            raise
        with self._lock:
            self._planned_goal = goal
            self._path = path
            self._set_state_locked("READY", "Plan ready")
            self._status["goal"] = goal
            self._status["path_length_m"] = self._path_length(path)
            self._status["waypoint_count"] = len(path)
        return self.status()

    def validate_goal(self, payload: dict | None):
        goal = self._goal(payload)
        try:
            path = self._compute_path(goal)
        except Exception as error:
            return {
                "ok": True,
                "valid": False,
                "code": "GOAL_BLOCKED",
                "message": str(error),
            }
        return {
            "ok": True,
            "valid": bool(path),
            "code": None if path else "GOAL_BLOCKED",
            "message": "Goal is valid" if path else "Goal is blocked",
        }

    def start(self):
        with self._lock:
            if self._status["state"] != "READY" or self._planned_goal is None:
                raise GatewayRequestError("a ready plan is required")
            goal = dict(self._planned_goal)
            self._generation += 1
            generation = self._generation
            self._set_state_locked("NAVIGATING", "Nav2 goal active", owns_control=True)
        if not self._navigate_client.wait_for_server(timeout_sec=self._timeout):
            self._set_error(generation, "NavigateToPose action server is unavailable")
            raise GatewayRequestError("NavigateToPose action server is unavailable")
        request = NavigateToPose.Goal()
        request.pose = self._pose(self._controller_goal(goal))
        future = self._navigate_client.send_goal_async(
            request,
            feedback_callback=lambda message: self._on_feedback(message, generation),
        )
        future.add_done_callback(
            lambda done: self._on_goal_response(done, generation)
        )
        return self.status()

    def cancel(self):
        with self._lock:
            self._generation += 1
            self._settling_generation = None
            self._settled_cycles = 0
            handle = self._goal_handle
            self._set_state_locked("CANCELLED", "Navigation cancelled")
        self._publish_zero()
        if handle is not None:
            _wait_future(handle.cancel_goal_async(), self._timeout)
            with self._lock:
                if self._goal_handle is handle:
                    self._goal_handle = None
        return self.status()

    def reset(self):
        self.cancel()
        for client, client_type, service_name in (
            (self._global_clear_client, ClearEntireCostmap, "global costmap"),
            (self._local_clear_client, ClearEntireCostmap, "local costmap"),
        ):
            if not client.wait_for_service(timeout_sec=self._timeout):
                raise GatewayRequestError(f"{service_name} clear service unavailable")
            _wait_future(client.call_async(client_type.Request()), self._timeout)
        with self._lock:
            self._settling_generation = None
            self._planned_goal = None
            self._path = []
            self._current_pose = None
            self._map_to_odom = None
            self._desired_pose = dict(self._startup_pose)
            self._reanchor_localization = True
            self._set_state_locked("IDLE", "Ready")
            self._status["start"] = None
            self._status["goal"] = None
            self._status["path_length_m"] = None
            self._status["waypoint_count"] = 0
        return self.status()

    def set_initial_pose(self, payload: dict | None):
        pose = self._initial_pose(payload)
        with self._lock:
            self._desired_pose = {"x": pose[0], "y": pose[1], "yaw": pose[2]}
            self._reanchor_localization = True
        return self.status()

    def set_speed(self, payload: dict | None):
        if payload is None or "speed_mps" not in payload:
            raise ValueError("speed_mps is required")
        speed = float(payload["speed_mps"])
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError("speed_mps must be a positive finite number")
        if self._speed_client is not None:
            if not self._speed_client.wait_for_service(timeout_sec=self._timeout):
                raise GatewayRequestError("Nav2 speed limit service is unavailable")
            request = self._speed_service_type.Request()
            request.speed_limit = speed
            request.percentage = False
            _wait_future(self._speed_client.call_async(request), self._timeout)
        with self._lock:
            self._status["speed_mps"] = speed
        return self.status()

    # -- ROS actions, subscriptions and state ---------------------------------

    def _compute_path(self, goal: dict[str, float]) -> list[dict[str, float]]:
        if not self._compute_client.wait_for_server(timeout_sec=self._timeout):
            raise GatewayRequestError("ComputePathToPose action server is unavailable")
        request = ComputePathToPose.Goal()
        request.goal = self._pose(goal)
        if hasattr(request, "planner_id"):
            request.planner_id = "GridBased"
        result = _wait_future(
            self._compute_client.send_goal_async(request), self._timeout
        )
        if not result.accepted:
            raise GatewayRequestError("ComputePathToPose goal was rejected")
        response = _wait_future(result.get_result_async(), self._timeout)
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            raise GatewayRequestError(
                f"ComputePathToPose ended with status {response.status}"
            )
        path = response.result
        poses = getattr(getattr(path, "path", None), "poses", [])
        return [
            {"x": float(item.pose.position.x), "y": float(item.pose.position.y)}
            for item in poses
        ]

    def _on_goal_response(self, future, generation: int):
        try:
            handle = future.result()
        except Exception as error:
            self._set_error(generation, str(error))
            return
        if not handle.accepted:
            self._set_error(generation, "NavigateToPose goal was rejected")
            return
        with self._lock:
            if generation != self._generation:
                return
            self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self._on_navigation_result(done, generation)
        )

    def _on_navigation_result(self, future, generation: int):
        try:
            status = future.result().status
        except Exception as error:
            self._set_error(generation, str(error))
            return
        state = {
            GoalStatus.STATUS_SUCCEEDED: ("SUCCEEDED", "Goal reached"),
            GoalStatus.STATUS_CANCELED: ("CANCELLED", "Navigation cancelled"),
            GoalStatus.STATUS_ABORTED: ("BLOCKED", "Nav2 could not reach the goal"),
        }.get(status, ("ERROR", f"Nav2 action ended with status {status}"))
        publish_zero = True
        with self._lock:
            if generation != self._generation:
                return
            self._goal_handle = None
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._settling_generation = generation
                self._settled_cycles = 0
                self._set_state_locked(
                    "NAVIGATING", "Settling at exact goal", owns_control=True
                )
            else:
                self._settling_generation = None
                self._settled_cycles = 0
                self._set_state_locked(state[0], state[1], owns_control=False)
        if publish_zero:
            self._publish_zero()

    def _check_goal_settled(self):
        settle_command = None
        with self._lock:
            generation = self._settling_generation
            if generation is None or generation != self._generation:
                return
            if self._current_pose is None or self._planned_goal is None:
                return
            distance_error = _distance(self._current_pose, self._planned_goal)
            yaw_error = _yaw_error(self._current_pose, self._planned_goal)
            if distance_error is None:
                self._settled_cycles = 0
                return
            if distance_error > self._goal_xy_tolerance:
                self._settled_cycles = 0
                target_heading = math.atan2(
                    self._planned_goal["y"] - self._current_pose["y"],
                    self._planned_goal["x"] - self._current_pose["x"],
                )
                heading_error = (
                    target_heading - self._current_pose["yaw"] + math.pi
                ) % (2.0 * math.pi) - math.pi
                direction = 1.0
                if abs(heading_error) > math.pi / 2.0:
                    direction = -1.0
                    heading_error = (
                        target_heading + math.pi - self._current_pose["yaw"]
                        + math.pi
                    ) % (2.0 * math.pi) - math.pi
                if abs(heading_error) > 0.25:
                    linear = 0.0
                else:
                    linear = direction * min(
                        0.06, max(0.015, distance_error * 0.8)
                    )
                angular = max(-0.35, min(0.35, heading_error * 1.2))
                settle_command = (linear, angular)
                self._status["command"] = {
                    "linear": linear,
                    "angular": angular,
                }
            elif yaw_error is not None and yaw_error > self._goal_yaw_tolerance:
                self._settled_cycles = 0
                yaw_delta = (
                    self._planned_goal["yaw"]
                    - self._current_pose["yaw"]
                    + math.pi
                ) % (2.0 * math.pi) - math.pi
                direction = 1.0 if yaw_delta > 0.0 else -1.0
                angular = direction * min(0.50, max(0.02, yaw_error * 0.8))
                settle_command = (0.0, angular)
                self._status["command"] = {"linear": 0.0, "angular": angular}
            else:
                self._settled_cycles += 1
                if self._settled_cycles >= 5:
                    self._settling_generation = None
                    self._set_state_locked(
                        "SUCCEEDED", "Goal reached", owns_control=False
                    )
        if settle_command is not None:
            command = Twist()
            command.linear.x = settle_command[0]
            command.angular.z = settle_command[1]
            self._zero_publisher.publish(command)
            return
        self._publish_zero()

    def _goal_reached_locked(self) -> bool:
        if self._current_pose is None or self._planned_goal is None:
            return False
        if _distance(self._current_pose, self._planned_goal) > self._goal_xy_tolerance:
            return False
        yaw_error = _yaw_error(self._current_pose, self._planned_goal)
        return yaw_error is None or yaw_error <= self._goal_yaw_tolerance

    def _on_feedback(self, message, generation: int):
        feedback = getattr(message, "feedback", message)
        with self._lock:
            if generation != self._generation:
                return
            remaining = getattr(feedback, "distance_remaining", None)
            if remaining is not None:
                self._status["distance_to_goal_m"] = float(remaining)
            recoveries = getattr(feedback, "number_of_recoveries", None)
            if recoveries is not None:
                self._status["recovery_count"] = int(recoveries)

    def _on_odom(self, message: Odometry):
        position = message.pose.pose.position
        odom_pose = {
            "x": float(position.x),
            "y": float(position.y),
            "yaw": self._yaw(message.pose.pose.orientation),
        }
        with self._lock:
            if self._map_to_odom is None or self._reanchor_localization:
                self._map_to_odom = self._localization_transform_locked(odom_pose)
                self._reanchor_localization = False
            map_pose = self._compose(self._map_to_odom, odom_pose)
            self._current_pose = {
                "x": map_pose["x"],
                "y": map_pose["y"],
                "yaw": map_pose["yaw"],
            }
            self._status["start"] = self._current_pose
            self._odom_ready = True
            map_to_odom = dict(self._map_to_odom)
        self._tf_broadcaster.sendTransform(
            self._transform_message(message.header.stamp, map_to_odom)
        )

    def _on_cmd_vel(self, message: Twist):
        with self._lock:
            self._status["command"] = {
                "linear": float(message.linear.x),
                "angular": float(message.angular.z),
            }

    def _on_global_plan(self, message: Path):
        with self._lock:
            map_to_odom = dict(self._map_to_odom) if self._map_to_odom else None
            current_pose = dict(self._current_pose) if self._current_pose else None
        path = []
        for item in message.poses:
            x = float(item.pose.position.x)
            y = float(item.pose.position.y)
            if message.header.frame_id == "odom":
                if map_to_odom is None:
                    return
                odom_x = x
                odom_y = y
                cosine = math.cos(map_to_odom["yaw"])
                sine = math.sin(map_to_odom["yaw"])
                x = map_to_odom["x"] + cosine * odom_x - sine * odom_y
                y = map_to_odom["y"] + sine * odom_x + cosine * odom_y
            elif message.header.frame_id == self._base_frame:
                if current_pose is None:
                    return
                base_x = x
                base_y = y
                cosine = math.cos(current_pose["yaw"])
                sine = math.sin(current_pose["yaw"])
                x = current_pose["x"] + cosine * base_x - sine * base_y
                y = current_pose["y"] + sine * base_x + cosine * base_y
            path.append({"x": x, "y": y})
        if not path:
            return
        with self._lock:
            self._path = path
            self._status["path_length_m"] = self._path_length(path)
            self._status["waypoint_count"] = len(path)

    @staticmethod
    def _yaw(orientation) -> float:
        return math.atan2(
            2.0 * orientation.w * orientation.z,
            1.0 - 2.0 * orientation.z * orientation.z,
        )

    def _localization_transform_locked(
        self, odom_pose: dict[str, float]
    ) -> dict[str, float]:
        desired = self._desired_pose
        yaw = desired["yaw"] - odom_pose["yaw"]
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return {
            "x": desired["x"] - cosine * odom_pose["x"] + sine * odom_pose["y"],
            "y": desired["y"] - sine * odom_pose["x"] - cosine * odom_pose["y"],
            "yaw": yaw,
        }

    @staticmethod
    def _compose(
        first: dict[str, float], second: dict[str, float]
    ) -> dict[str, float]:
        cosine = math.cos(first["yaw"])
        sine = math.sin(first["yaw"])
        return {
            "x": first["x"] + cosine * second["x"] - sine * second["y"],
            "y": first["y"] + sine * second["x"] + cosine * second["y"],
            "yaw": first["yaw"] + second["yaw"],
        }

    def _transform_message(self, stamp, transform: dict[str, float]):
        message = TransformStamped()
        message.header.stamp = stamp
        message.header.frame_id = self._map_frame
        message.child_frame_id = "odom"
        message.transform.translation.x = transform["x"]
        message.transform.translation.y = transform["y"]
        message.transform.rotation.z = math.sin(transform["yaw"] / 2.0)
        message.transform.rotation.w = math.cos(transform["yaw"] / 2.0)
        return message

    def _refresh_health(self):
        self._nav2_ready = (
            self._compute_client.server_is_ready()
            and self._navigate_client.server_is_ready()
        )
        if self._tf_buffer is None or Time is None:
            self._tf_ready = False
            return
        try:
            self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time()
            )
            self._tf_ready = True
        except Exception:
            self._tf_ready = False

    def _set_error(self, generation: int, error: str):
        with self._lock:
            if generation == self._generation:
                self._set_state_locked("ERROR", "Nav2 action failed", error)
        self._publish_zero()

    def _set_state_locked(
        self,
        state: str,
        message: str,
        error: str | None = None,
        *,
        owns_control: bool | None = None,
    ):
        self._status["state"] = state
        self._status["message"] = message
        self._status["error"] = error
        if owns_control is not None:
            self._status["owns_control"] = owns_control
        else:
            self._status["owns_control"] = state == "NAVIGATING"

    def _publish_zero(self):
        # Update the HTTP-visible command before publishing. The ROS command
        # echo arrives asynchronously, so exposing the old command here can
        # make a just-finished NavigateToPose look unsafe to callers.
        with self._lock:
            self._status["command"] = {"linear": 0.0, "angular": 0.0}
        self._zero_publisher.publish(Twist())

    def _ensure_startup_localization(self):
        if self._map_to_odom is None:
            with self._lock:
                self._reanchor_localization = True

    @staticmethod
    def _initial_pose(payload: dict | None) -> tuple[float, float, float]:
        if not isinstance(payload, dict):
            raise ValueError("initial pose {x, y, yaw} is required")
        try:
            x = float(payload["x"])
            y = float(payload["y"])
            yaw = float(payload.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("initial pose {x, y, yaw} is required") from error
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("initial pose values must be finite")
        return x, y, yaw

    def _pose(self, point: dict[str, float]) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point["x"]
        pose.pose.position.y = point["y"]
        with self._lock:
            current = self._current_pose
        requested_yaw = point.get("yaw")
        if requested_yaw is not None:
            yaw = requested_yaw
        elif current is None:
            yaw = 0.0
        else:
            yaw = math.atan2(point["y"] - current["y"], point["x"] - current["x"])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _controller_goal(self, goal: dict[str, float]) -> dict[str, float]:
        """Use path tangent for Nav2, while gateway settles the requested yaw."""

        if goal.get("yaw") is None:
            return goal
        with self._lock:
            path = list(self._path)
        if len(path) < 2:
            return goal
        previous = path[-2]
        final = path[-1]
        if math.hypot(final["x"] - previous["x"], final["y"] - previous["y"]) < 1e-6:
            return goal
        controller_goal = dict(goal)
        controller_goal["yaw"] = math.atan2(
            final["y"] - previous["y"], final["x"] - previous["x"]
        )
        return controller_goal

    @staticmethod
    def _goal(payload: dict | None) -> dict[str, float]:
        if not isinstance(payload, dict) or not isinstance(payload.get("goal"), dict):
            raise ValueError("goal {x, y} is required")
        goal = payload["goal"]
        x = float(goal["x"])
        y = float(goal["y"])
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("goal coordinates must be finite")
        result = {"x": x, "y": y}
        if goal.get("yaw") is not None:
            yaw = float(goal["yaw"])
            if not math.isfinite(yaw):
                raise ValueError("goal yaw must be finite")
            result["yaw"] = yaw
        return result

    @staticmethod
    def _path_length(path: list[dict[str, float]]) -> float:
        return sum(
            math.hypot(item["x"] - previous["x"], item["y"] - previous["y"])
            for previous, item in zip(path, path[1:])
        )

    @staticmethod
    def _new_status() -> dict[str, Any]:
        return {
            "state": "IDLE",
            "message": "Ready",
            "owns_control": False,
            "start": None,
            "goal": None,
            "distance_to_goal_m": None,
            "path_length_m": None,
            "waypoint_count": 0,
            "global_path": [],
            "command": {"linear": 0.0, "angular": 0.0},
            "speed_mps": 0.5,
            "minimum_clearance_m": None,
            "error": None,
        }


class _GatewayHandler(BaseHTTPRequestHandler):
    gateway: Nav2Gateway

    def do_GET(self):
        self._handle("GET", None)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length)
        try:
            payload = json.loads(raw.decode("utf-8")) if raw else {}
        except json.JSONDecodeError:
            payload = None
        self._handle("POST", payload)

    def _handle(self, method: str, payload: dict | None):
        status_code, response = self.gateway.handle_http(method, self.path, payload)
        body = json.dumps(response).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args):
        LOGGER.info("%s - %s", self.address_string(), format % args)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Gateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
