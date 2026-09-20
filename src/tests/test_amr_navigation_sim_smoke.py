"""Opt-in end-to-end Dijkstra cost-to-go + DWA-lite navigation smoke test."""

from __future__ import annotations

import math
import os
import shlex
import signal
import subprocess
import tempfile
import time
import unittest
import uuid

from config import CONFIG
from navigation.types import Point2D
from navigation.navigator import Navigator


@unittest.skipUnless(
    os.environ.get("RUN_GZ_SIM_SMOKE") == "1",
    "set RUN_GZ_SIM_SMOKE=1 to run Gazebo integration",
)
class AMRNavigationSimulationSmokeTest(unittest.TestCase):
    def setUp(self):
        self.previous_partition = os.environ.get("GZ_PARTITION")
        self.previous_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH")
        self.environment = os.environ.copy()
        self.environment["GZ_PARTITION"] = f"amr_nav_{uuid.uuid4().hex}"
        self.environment["GZ_SIM_RESOURCE_PATH"] = str(
            CONFIG.paths.models_root
        )
        os.environ["GZ_PARTITION"] = self.environment["GZ_PARTITION"]
        os.environ["GZ_SIM_RESOURCE_PATH"] = self.environment[
            "GZ_SIM_RESOURCE_PATH"
        ]
        self.log = tempfile.TemporaryFile(mode="w+")
        simulator_command = shlex.split(
            os.environ.get("GZ_SIM_COMMAND", "gz sim -s")
        )
        self.process = subprocess.Popen(
            [
                *simulator_command,
                "-v",
                "2",
                "-r",
                str(CONFIG.paths.navigation_test_world_path),
            ],
            env=self.environment,
            stdout=self.log,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self.transport = None
        self.navigator = None
        self._wait_for_topics()

    def tearDown(self):
        if self.navigator is not None:
            self.navigator.close()
        if self.transport is not None:
            self.transport.close()
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.log.close()
        if self.previous_partition is None:
            os.environ.pop("GZ_PARTITION", None)
        else:
            os.environ["GZ_PARTITION"] = self.previous_partition
        if self.previous_resource_path is None:
            os.environ.pop("GZ_SIM_RESOURCE_PATH", None)
        else:
            os.environ["GZ_SIM_RESOURCE_PATH"] = self.previous_resource_path

    def _topics(self):
        return subprocess.run(
            ["gz", "topic", "-l"],
            env=self.environment,
            capture_output=True,
            text=True,
            timeout=3,
            check=True,
        ).stdout.splitlines()

    def _wait_for_topics(self):
        required = {
            CONFIG.simulation.command_topic,
            CONFIG.simulation.odometry_topic,
            CONFIG.simulation.lidar_topic,
            CONFIG.simulation.dynamic_pose_topic,
        }
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                self.log.seek(0)
                self.fail(f"Gazebo exited early:\n{self.log.read()}")
            if required.issubset(self._topics()):
                return
            time.sleep(0.2)
        self.log.seek(0)
        self.fail(f"navigation topics did not appear:\n{self.log.read()}")

    def _wait_for_navigation_sensors(self):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            snapshot = self.transport.telemetry_snapshot()
            if all(
                snapshot["freshness"][name]["fresh"]
                for name in ("lidar", "odometry", "model_pose")
            ):
                return
            time.sleep(0.05)
        self.fail("navigation sensors did not become ready")

    def test_robot_reaches_a_clear_goal_and_stops(self):
        self._run_goal(Point2D(-2.0, 0.5), timeout=25)

    def test_robot_follows_planned_detour_around_static_obstacle(self):
        self._run_goal(Point2D(-0.55, 0.55), timeout=35)

    def test_robot_reaches_reported_diagonal_goal(self):
        self._run_goal(Point2D(-0.44, 0.43), timeout=35)

    def test_robot_reaches_goal_near_center_pillar(self):
        self._run_goal(Point2D(0.19, -0.35), timeout=45)

    def test_robot_reaches_left_boundary_goal(self):
        self._run_goal(Point2D(-2.45, 0.0), timeout=25)

    def test_robot_reaches_upper_boundary_goal(self):
        self._run_goal(Point2D(0.0, 2.0), timeout=45)

    def test_robot_reaches_lower_boundary_goal(self):
        self._run_goal(Point2D(0.0, -2.0), timeout=45)

    def test_robot_reaches_narrow_center_corridor_goal(self):
        self._run_goal(Point2D(0.0, 0.55), timeout=45)

    def test_robot_reaches_far_right_boundary_goal(self):
        self._run_goal(Point2D(2.15, 0.55), timeout=60)

    def test_robot_reaches_open_ground_beyond_pillar_field(self):
        self._run_goal(Point2D(1.70, 3.99), timeout=75)

    def test_robot_reaches_goal_when_model_pose_differs_from_odometry(self):
        self._set_robot_pose(-1.54, -1.45, math.radians(-49.1))
        self._run_goal(Point2D(-0.28, 0.55), timeout=50)

    def test_robot_escapes_reported_blocked_start(self):
        self._set_robot_pose(-1.29, -0.03, math.radians(-4.2))
        self._run_goal(Point2D(0.56, -1.50), timeout=60)

    def _set_robot_pose(self, x: float, y: float, yaw: float) -> None:
        result = subprocess.run(
            [
                "gz",
                "service",
                "-s",
                f"/world/{CONFIG.simulation.world_name}/set_pose",
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "5000",
                "--req",
                (
                    f'name: "{CONFIG.simulation.model_name}", '
                    f"position: {{x: {x}, y: {y}, z: 0.01}}, "
                    f"orientation: {{z: {math.sin(yaw / 2.0)}, "
                    f"w: {math.cos(yaw / 2.0)}}}"
                ),
            ],
            env=self.environment,
            capture_output=True,
            text=True,
            timeout=7,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("data: true", result.stdout)
        time.sleep(0.5)

    def _run_goal(self, goal, *, timeout):
        from transport import AMRTransport

        self.transport = AMRTransport()
        self._wait_for_navigation_sensors()
        self.navigator = Navigator(self.transport)
        plan = self.navigator.plan(goal)
        if goal == Point2D(-0.55, 0.55):
            self.assertGreater(plan["path_length_m"], 1.6)
        self.navigator.start()

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.navigator.status()
            if status["state"] in {"SUCCEEDED", "BLOCKED", "ERROR"}:
                break
            time.sleep(0.1)
        else:
            self.fail(f"navigation timed out: {self.navigator.status()}")

        status = self.navigator.status()
        self.assertEqual(status["state"], "SUCCEEDED", status)
        self.assertLessEqual(
            status["distance_to_goal_m"], CONFIG.navigation.goal_tolerance_m
        )
        self.assertEqual(status["command"], {"linear": 0.0, "angular": 0.0})
        if status["minimum_clearance_m"] is not None:
            self.assertGreaterEqual(
                status["minimum_clearance_m"],
                0.0,
            )


if __name__ == "__main__":
    unittest.main()
