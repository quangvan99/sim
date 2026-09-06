"""Thread-safe Gazebo Transport bridge for AMR control and sensors."""

from __future__ import annotations

import copy
import math
import numbers
import os
import subprocess
import threading
import time
from collections.abc import Callable

from gz.msgs.imu_pb2 import IMU
from gz.msgs.laserscan_pb2 import LaserScan
from gz.msgs.model_pb2 import Model
from gz.msgs.odometry_pb2 import Odometry
from gz.msgs.pose_v_pb2 import Pose_V
from gz.msgs.twist_pb2 import Twist
from gz.transport import Node

from amr.config import CONFIG


def _finite(value: float) -> float | None:
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def _components(value, *names: str) -> dict[str, float | None]:
    return {name: _finite(getattr(value, name)) for name in names}


def _yaw(quaternion) -> float:
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def _request_world_reset() -> bool:
    """Call the Gazebo control service outside the subscribed Python node.

    Gazebo Transport's Python binding can time out on a blocking service call
    after the same process has started high-rate sensor subscriptions. The
    Gazebo CLI uses a short-lived transport process and avoids that deadlock.
    """

    try:
        result = subprocess.run(
            [
                "gz",
                "service",
                "-s",
                CONFIG.simulation.world_control_service,
                "--reqtype",
                "gz.msgs.WorldControl",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                str(CONFIG.control.reset_timeout_ms),
                "--req",
                "reset: {all: true}",
            ],
            env=os.environ.copy(),
            capture_output=True,
            text=True,
            timeout=CONFIG.control.reset_timeout_ms / 1_000 + 1.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise RuntimeError(f"Gazebo reset request failed: {error}") from error
    return result.returncode == 0 and "data: true" in result.stdout


class AMRTransport:
    """Publish safe velocity commands and retain latest sensor snapshots."""

    _SENSOR_NAMES = (
        "lidar",
        "imu",
        "odometry",
        "model_pose",
        "joint_states",
    )

    def __init__(
        self,
        *,
        node=None,
        reset_client: Callable[[], bool] = _request_world_reset,
        clock: Callable[[], float] = time.monotonic,
        auto_start: bool = True,
    ):
        self.node = node or Node()
        self._reset_client = reset_client
        self._clock = clock
        self._lock = threading.Lock()
        self._reset_lock = threading.Lock()
        self._stopped = threading.Event()
        self._publishing_paused = threading.Event()
        self._resetting = threading.Event()
        self._closed = False
        self._linear = 0.0
        self._angular = 0.0
        self._last_command_at = self._clock()
        self._sensor_data: dict[str, dict | None] = {
            name: None for name in self._SENSOR_NAMES
        }
        self._sensor_received_at: dict[str, float | None] = {
            name: None for name in self._SENSOR_NAMES
        }
        self._sensor_sequence: dict[str, int] = {
            name: 0 for name in self._SENSOR_NAMES
        }

        self._command_publisher = self.node.advertise(
            CONFIG.simulation.command_topic, Twist
        )
        subscriptions = (
            (
                LaserScan,
                CONFIG.simulation.lidar_topic,
                self._on_lidar,
            ),
            (IMU, CONFIG.simulation.imu_topic, self._on_imu),
            (
                Odometry,
                CONFIG.simulation.odometry_topic,
                self._on_odometry,
            ),
            (
                Pose_V,
                CONFIG.simulation.dynamic_pose_topic,
                self._on_model_pose,
            ),
            (
                Model,
                CONFIG.simulation.joint_state_topic,
                self._on_joint_states,
            ),
        )
        self._subscription_topics = []
        for message_type, topic, callback in subscriptions:
            if self.node.subscribe(message_type, topic, callback) is False:
                raise RuntimeError(f"Failed to subscribe to {topic}")
            self._subscription_topics.append(topic)

        self._thread = threading.Thread(
            target=self._publish_loop,
            name="amr-command-publisher",
            daemon=True,
        )
        if auto_start:
            self._thread.start()

    def set_velocity(self, linear: float, angular: float) -> dict[str, float]:
        """Clamp and immediately publish one manual velocity command."""

        linear_value = self._clamp_velocity(
            "linear", linear, CONFIG.control.max_linear_velocity_mps
        )
        angular_value = self._clamp_velocity(
            "angular", angular, CONFIG.control.max_angular_velocity_rps
        )
        if self._resetting.is_set() and (
            linear_value != 0.0 or angular_value != 0.0
        ):
            raise RuntimeError("reset in progress")
        with self._lock:
            self._linear = linear_value
            self._angular = angular_value
            self._last_command_at = self._clock()
        self._publish(linear_value, angular_value)
        return {"linear": linear_value, "angular": angular_value}

    def telemetry_snapshot(self) -> dict:
        """Return JSON-safe latest sensor values and freshness metadata."""

        now = self._clock()
        linear, angular, command_age = self._effective_command()
        with self._lock:
            sensor_data = copy.deepcopy(self._sensor_data)
            received_at = dict(self._sensor_received_at)
            sequences = dict(self._sensor_sequence)

        freshness = {}
        for name in self._SENSOR_NAMES:
            timestamp = received_at[name]
            age = None if timestamp is None else max(0.0, now - timestamp)
            freshness[name] = {
                "received": timestamp is not None,
                "fresh": (
                    age is not None
                    and age <= CONFIG.control.sensor_timeout_seconds
                ),
                "age_seconds": age,
                "sequence": sequences[name],
            }

        return {
            "command": {
                "linear": linear,
                "angular": angular,
                "age_seconds": command_age,
                "active": linear != 0.0 or angular != 0.0,
            },
            "sensors": sensor_data,
            "freshness": freshness,
            "ready": all(item["fresh"] for item in freshness.values()),
        }

    def reset(self) -> bool:
        """Stop the robot and request a complete world reset."""

        with self._reset_lock:
            self._resetting.set()
            self._publishing_paused.set()
            try:
                self.set_velocity(0.0, 0.0)
                time.sleep(0.05)
                return self._reset_client()
            finally:
                time.sleep(0.1)
                self._resetting.clear()
                self._publishing_paused.clear()

    def close(self) -> None:
        """Publish zero velocity and stop the heartbeat thread."""

        if self._closed:
            return
        self._closed = True
        self._stopped.set()
        self._publish(0.0, 0.0)
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        for topic in self._subscription_topics:
            self.node.unsubscribe(topic)

    @staticmethod
    def _validated_number(name: str, value: float) -> float:
        if isinstance(value, bool) or not isinstance(value, numbers.Real):
            raise ValueError(f"{name} must be a number")
        parsed = float(value)
        if not math.isfinite(parsed):
            raise ValueError(f"{name} must be finite")
        return parsed

    @classmethod
    def _clamp_velocity(cls, name: str, value: float, limit: float) -> float:
        value = cls._validated_number(name, value)
        return max(-limit, min(limit, value))

    def _effective_command(self) -> tuple[float, float, float]:
        with self._lock:
            age = max(0.0, self._clock() - self._last_command_at)
            if age > CONFIG.control.command_timeout_seconds:
                self._linear = 0.0
                self._angular = 0.0
            return self._linear, self._angular, age

    def _publish(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self._command_publisher.publish(message)

    def _publish_loop(self) -> None:
        period = 1.0 / CONFIG.control.publish_hz
        was_active = False
        while not self._stopped.is_set():
            if self._publishing_paused.is_set():
                self._stopped.wait(period)
                continue
            started = self._clock()
            linear, angular, _ = self._effective_command()
            active = linear != 0.0 or angular != 0.0
            if active or was_active:
                self._publish(linear, angular)
            was_active = active
            elapsed = self._clock() - started
            self._stopped.wait(max(0.0, period - elapsed))

    def _store_sensor(self, name: str, data: dict) -> None:
        with self._lock:
            self._sensor_sequence[name] += 1
            data["sequence"] = self._sensor_sequence[name]
            self._sensor_data[name] = data
            self._sensor_received_at[name] = self._clock()

    def _on_lidar(self, message: LaserScan) -> None:
        ranges = []
        invalid_count = 0
        range_min = float(message.range_min)
        for value in message.ranges:
            parsed = float(value)
            if math.isnan(parsed):
                invalid_count += 1
                ranges.append(None)
            elif math.isinf(parsed):
                ranges.append(None if parsed > 0.0 else range_min)
            else:
                ranges.append(parsed)
        self._store_sensor(
            "lidar",
            {
                "frame": message.frame,
                "angle_min": _finite(message.angle_min),
                "angle_max": _finite(message.angle_max),
                "angle_step": _finite(message.angle_step),
                "range_min": _finite(message.range_min),
                "range_max": _finite(message.range_max),
                "count": int(message.count),
                "ranges": ranges,
                "invalid_count": invalid_count,
            },
        )

    def _on_imu(self, message: IMU) -> None:
        self._store_sensor(
            "imu",
            {
                "entity_name": message.entity_name,
                "orientation": _components(
                    message.orientation, "x", "y", "z", "w"
                ),
                "angular_velocity": _components(
                    message.angular_velocity, "x", "y", "z"
                ),
                "linear_acceleration": _components(
                    message.linear_acceleration, "x", "y", "z"
                ),
            },
        )

    def _on_odometry(self, message: Odometry) -> None:
        self._store_sensor(
            "odometry",
            {
                "position": _components(
                    message.pose.position, "x", "y", "z"
                ),
                "orientation": _components(
                    message.pose.orientation, "x", "y", "z", "w"
                ),
                "yaw": _finite(_yaw(message.pose.orientation)),
                "linear_velocity": _components(
                    message.twist.linear, "x", "y", "z"
                ),
                "angular_velocity": _components(
                    message.twist.angular, "x", "y", "z"
                ),
            },
        )

    def _on_model_pose(self, message: Pose_V) -> None:
        for pose in message.pose:
            if pose.name != CONFIG.simulation.model_name:
                continue
            self._store_sensor(
                "model_pose",
                {
                    "name": pose.name,
                    "position": _components(pose.position, "x", "y", "z"),
                    "orientation": _components(
                        pose.orientation, "x", "y", "z", "w"
                    ),
                    "yaw": _finite(_yaw(pose.orientation)),
                },
            )
            return

    def _on_joint_states(self, message: Model) -> None:
        joints = []
        for joint in message.joint:
            joints.append(
                {
                    "name": joint.name,
                    "position": _finite(joint.axis1.position),
                    "velocity": _finite(joint.axis1.velocity),
                    "force": _finite(joint.axis1.force),
                }
            )
        self._store_sensor(
            "joint_states",
            {"model": message.name, "joints": joints},
        )
