"""Small HTTP facade around Nav2's NavigateToPose action."""

from __future__ import annotations

import json
import logging
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

try:
    from nav2_msgs.srv import SpeedLimit
except ImportError:  # Older Nav2 releases may not provide this service.
    SpeedLimit = None


LOGGER = logging.getLogger("nav2_gateway")


class GatewayRequestError(RuntimeError):
    """Raised for an invalid or currently unavailable Nav2 operation."""


def _wait_future(future, timeout: float):
    done = threading.Event()
    result: list[Any] = []
    failure: list[BaseException] = []

    def complete(done_future):
        try:
            result.append(done_future.result())
        except BaseException as error:
            failure.append(error)
        finally:
            done.set()

    future.add_done_callback(complete)
    if not done.wait(timeout):
        raise GatewayRequestError("ROS 2 action or service timed out")
    if failure:
        raise GatewayRequestError(str(failure[0])) from failure[0]
    return result[0] if result else None


def _distance(first, second):
    if first is None or second is None:
        return None
    return math.hypot(first["x"] - second["x"], first["y"] - second["y"])


def _yaw_error(first, second):
    if first is None or second is None or second.get("yaw") is None:
        return None
    return abs((first["yaw"] - second["yaw"] + math.pi) % (2 * math.pi) - math.pi)


class Nav2Gateway(Node):
    """Thread-safe HTTP facade for one Nav2 NavigateToPose goal."""

    def __init__(self):
        super().__init__("amr_nav2_gateway")
        self.declare_parameter("http_host", "0.0.0.0")
        self.declare_parameter("http_port", 8010)
        self.declare_parameter("action_timeout_seconds", 10.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("initial_pose_x", -2.0)
        self.declare_parameter("initial_pose_y", -0.5)
        self.declare_parameter("initial_pose_yaw", 0.0)

        self._timeout = float(self.get_parameter("action_timeout_seconds").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._lock = threading.Lock()
        self._generation = 0
        self._goal_handle = None
        self._goal: dict[str, float] | None = None
        self._current_pose: dict[str, float] | None = None
        self._map_to_odom: dict[str, float] | None = None
        self._reanchor = True
        self._desired_pose = {
            "x": float(self.get_parameter("initial_pose_x").value),
            "y": float(self.get_parameter("initial_pose_y").value),
            "yaw": float(self.get_parameter("initial_pose_yaw").value),
        }
        self._status = {
            "state": "IDLE",
            "message": "Ready",
            "owns_control": False,
            "start": None,
            "goal": None,
            "speed_mps": 0.5,
            "error": None,
        }

        self._navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._global_clear = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap"
        )
        self._local_clear = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap"
        )
        self._speed_client = (
            self.create_client(SpeedLimit, "/controller_server/set_speed_limit")
            if SpeedLimit is not None
            else None
        )
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        host = str(self.get_parameter("http_host").value)
        port = int(self.get_parameter("http_port").value)
        self._http = GatewayHTTPServer((host, port), _GatewayHandler, self)
        self._http_thread = threading.Thread(
            target=self._http.serve_forever, name="nav2-gateway-http", daemon=True
        )
        self._http_thread.start()
        LOGGER.info("Nav2 gateway listening on %s:%d", host, port)

    def destroy_node(self):
        self._http.shutdown()
        self._http.server_close()
        self._http_thread.join(timeout=2.0)
        return super().destroy_node()

    # HTTP API ---------------------------------------------------------------

    def handle_http(self, method: str, path: str, payload: dict | None):
        route = path.split("?", 1)[0]
        try:
            if method == "GET":
                if route == "/health":
                    return 200, self.health()
                if route == "/status":
                    return 200, self.status()
            if method == "POST":
                handlers = {
                    "/navigate": self.navigate,
                    "/cancel": lambda _: self.cancel(),
                    "/reset": lambda _: self.reset(),
                    "/initialpose": self.set_initial_pose,
                    "/speed": self.set_speed,
                }
                handler = handlers.get(route)
                if handler is not None:
                    return 200, handler(payload)
            return 404, {"ok": False, "error": "endpoint not found"}
        except GatewayRequestError as error:
            return 409, {"ok": False, "error": str(error)}
        except (KeyError, TypeError, ValueError) as error:
            return 422, {"ok": False, "error": str(error)}
        except Exception as error:
            LOGGER.exception("Nav2 gateway request failed")
            return 500, {"ok": False, "error": str(error)}

    def health(self):
        nav2_ready = self._navigate_client.server_is_ready()
        with self._lock:
            odom_ready = self._current_pose is not None
            tf_ready = self._map_to_odom is not None
        return {
            "ok": True,
            "ready": nav2_ready and odom_ready and tf_ready,
            "nav2_ready": nav2_ready,
            "odom_ready": odom_ready,
            "tf_ready": tf_ready,
        }

    def status(self):
        with self._lock:
            status = dict(self._status)
            status["start"] = self._current_pose
            status["distance_to_goal_m"] = _distance(self._current_pose, self._goal)
            status["yaw_error_to_goal_rad"] = _yaw_error(self._current_pose, self._goal)
            return {"ok": True, **status}

    def navigate(self, payload: dict | None):
        goal = self._goal_from_payload(payload)
        with self._lock:
            if self._status["state"] == "NAVIGATING":
                raise GatewayRequestError("navigation is still active")
            self._goal = goal
            self._status["goal"] = goal
            self._generation += 1
            generation = self._generation
            self._set_state("NAVIGATING", "Nav2 goal active", owns_control=True)

        if not self._navigate_client.wait_for_server(timeout_sec=self._timeout):
            self._set_error(generation, "NavigateToPose action server is unavailable")
            raise GatewayRequestError("NavigateToPose action server is unavailable")
        request = NavigateToPose.Goal()
        request.pose = self._pose(goal)
        future = self._navigate_client.send_goal_async(request)
        future.add_done_callback(lambda done: self._on_goal_response(done, generation))
        return self.status()

    def cancel(self):
        with self._lock:
            self._generation += 1
            handle = self._goal_handle
            self._set_state("CANCELLED", "Navigation cancelled")
        if handle is not None:
            _wait_future(handle.cancel_goal_async(), self._timeout)
            with self._lock:
                if self._goal_handle is handle:
                    self._goal_handle = None
        return self.status()

    def reset(self):
        with self._lock:
            active = self._status["state"] == "NAVIGATING"
        if active:
            self.cancel()
        for client, name in (
            (self._global_clear, "global costmap"),
            (self._local_clear, "local costmap"),
        ):
            if not client.wait_for_service(timeout_sec=self._timeout):
                raise GatewayRequestError(f"{name} clear service unavailable")
            _wait_future(client.call_async(ClearEntireCostmap.Request()), self._timeout)
        with self._lock:
            self._goal = None
            self._current_pose = None
            self._map_to_odom = None
            self._desired_pose = {
                "x": float(self.get_parameter("initial_pose_x").value),
                "y": float(self.get_parameter("initial_pose_y").value),
                "yaw": float(self.get_parameter("initial_pose_yaw").value),
            }
            self._reanchor = True
            self._set_state("IDLE", "Ready")
            self._status["goal"] = None
        return self.status()

    def set_initial_pose(self, payload: dict | None):
        pose = self._initial_pose(payload)
        with self._lock:
            self._desired_pose = dict(zip(("x", "y", "yaw"), pose))
            self._reanchor = True
        return self.status()

    def set_speed(self, payload: dict | None):
        if not isinstance(payload, dict) or "speed_mps" not in payload:
            raise ValueError("speed_mps is required")
        speed = float(payload["speed_mps"])
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError("speed_mps must be a positive finite number")
        if self._speed_client is not None:
            if not self._speed_client.wait_for_service(timeout_sec=self._timeout):
                raise GatewayRequestError("Nav2 speed limit service is unavailable")
            request = SpeedLimit.Request()
            request.speed_limit = speed
            request.percentage = False
            _wait_future(self._speed_client.call_async(request), self._timeout)
        with self._lock:
            self._status["speed_mps"] = speed
        return self.status()

    # ROS callbacks ----------------------------------------------------------

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
                handle.cancel_goal_async()
                return
            self._goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(lambda done: self._on_result(done, generation))

    def _on_result(self, future, generation: int):
        try:
            result = future.result().status
        except Exception as error:
            self._set_error(generation, str(error))
            return
        state, message = {
            GoalStatus.STATUS_SUCCEEDED: ("SUCCEEDED", "Goal reached"),
            GoalStatus.STATUS_CANCELED: ("CANCELLED", "Navigation cancelled"),
            GoalStatus.STATUS_ABORTED: ("BLOCKED", "Nav2 could not reach the goal"),
        }.get(result, ("ERROR", f"Nav2 action ended with status {result}"))
        with self._lock:
            if generation == self._generation:
                self._goal_handle = None
                self._set_state(state, message)

    def _on_odom(self, message: Odometry):
        position = message.pose.pose.position
        odom = {
            "x": float(position.x),
            "y": float(position.y),
            "yaw": self._yaw(message.pose.pose.orientation),
        }
        with self._lock:
            if self._map_to_odom is None or self._reanchor:
                self._map_to_odom = self._reanchor_transform(odom)
                self._reanchor = False
            self._current_pose = self._compose(self._map_to_odom, odom)
            self._status["start"] = self._current_pose
            transform = dict(self._map_to_odom)
        self._tf_broadcaster.sendTransform(
            self._transform_message(message.header.stamp, transform)
        )

    # State and pose helpers -------------------------------------------------

    def _set_state(
        self, state: str, message: str, *, owns_control: bool | None = None, error=None
    ):
        self._status.update(
            state=state,
            message=message,
            error=error,
            owns_control=state == "NAVIGATING" if owns_control is None else owns_control,
        )

    def _set_error(self, generation: int, error: str):
        with self._lock:
            if generation == self._generation:
                self._set_state("ERROR", "Nav2 action failed", error=error)

    @staticmethod
    def _goal_from_payload(payload: dict | None):
        if not isinstance(payload, dict) or not isinstance(payload.get("goal"), dict):
            raise ValueError("goal {x, y} is required")
        goal = payload["goal"]
        result = {"x": float(goal["x"]), "y": float(goal["y"])}
        if goal.get("yaw") is not None:
            result["yaw"] = float(goal["yaw"])
        if not all(math.isfinite(value) for value in result.values()):
            raise ValueError("goal values must be finite")
        return result

    @staticmethod
    def _initial_pose(payload: dict | None):
        if not isinstance(payload, dict):
            raise ValueError("initial pose {x, y, yaw} is required")
        try:
            values = (
                float(payload["x"]),
                float(payload["y"]),
                float(payload.get("yaw", 0.0)),
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("initial pose {x, y, yaw} is required") from error
        if not all(math.isfinite(value) for value in values):
            raise ValueError("initial pose values must be finite")
        return values

    def _pose(self, goal: dict[str, float]):
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = goal["x"], goal["y"]
        with self._lock:
            current = self._current_pose
        yaw = goal.get("yaw")
        if yaw is None:
            yaw = 0.0 if current is None else math.atan2(
                goal["y"] - current["y"], goal["x"] - current["x"]
            )
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _reanchor_transform(self, odom):
        desired = self._desired_pose
        yaw = desired["yaw"] - odom["yaw"]
        cosine, sine = math.cos(yaw), math.sin(yaw)
        return {
            "x": desired["x"] - cosine * odom["x"] + sine * odom["y"],
            "y": desired["y"] - sine * odom["x"] - cosine * odom["y"],
            "yaw": yaw,
        }

    @staticmethod
    def _compose(first, second):
        cosine, sine = math.cos(first["yaw"]), math.sin(first["yaw"])
        return {
            "x": first["x"] + cosine * second["x"] - sine * second["y"],
            "y": first["y"] + sine * second["x"] + cosine * second["y"],
            "yaw": first["yaw"] + second["yaw"],
        }

    @staticmethod
    def _yaw(orientation):
        return math.atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )

    def _transform_message(self, stamp, transform):
        message = TransformStamped()
        message.header.stamp = stamp
        message.header.frame_id = self._map_frame
        message.child_frame_id = "odom"
        message.transform.translation.x = transform["x"]
        message.transform.translation.y = transform["y"]
        message.transform.rotation.z = math.sin(transform["yaw"] / 2)
        message.transform.rotation.w = math.cos(transform["yaw"] / 2)
        return message


class SimulatorTF(Node):
    """Bridge simulator odometry to the TF frames expected by Nav2."""

    def __init__(self):
        super().__init__("amr_simulator_tf")
        self._dynamic = tf2_ros.TransformBroadcaster(self)
        self._static = tf2_ros.StaticTransformBroadcaster(self)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._publish_static()

    def _publish_static(self):
        self._static.sendTransform(
            [
                self._static_transform("base_footprint", "base_link", 0.0, 0.0, 0.0),
                self._static_transform("base_link", "base_scan", -0.032, 0.0, 0.171),
            ]
        )

    def _on_odom(self, message: Odometry):
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self._dynamic.sendTransform(transform)

    def _static_transform(self, parent, child, x, y, z):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.w = 1.0
        return transform


class GatewayHTTPServer(ThreadingHTTPServer):
    def __init__(self, address, handler, gateway):
        self.gateway = gateway
        super().__init__(address, handler)


class _GatewayHandler(BaseHTTPRequestHandler):
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

    def _handle(self, method, payload):
        status_code, response = self.server.gateway.handle_http(method, self.path, payload)
        body = json.dumps(response).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
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


def main_tf(args=None):
    rclpy.init(args=args)
    node = SimulatorTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
