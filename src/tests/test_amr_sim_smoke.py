"""Opt-in end-to-end smoke test for the Gazebo AMR world."""

from __future__ import annotations

import os
import re
import signal
import subprocess
import tempfile
import time
import unittest
import uuid

from amr.config import CONFIG


@unittest.skipUnless(
    os.environ.get("RUN_GZ_SIM_SMOKE") == "1",
    "set RUN_GZ_SIM_SMOKE=1 to run Gazebo integration",
)
class AMRSimulationSmokeTest(unittest.TestCase):
    def setUp(self):
        self.environment = os.environ.copy()
        self.environment["GZ_PARTITION"] = f"amr_smoke_{uuid.uuid4().hex}"
        self.environment["GZ_SIM_RESOURCE_PATH"] = str(
            CONFIG.paths.models_root
        )
        self.log = tempfile.TemporaryFile(mode="w+")
        self.process = subprocess.Popen(
            [
                "gz",
                "sim",
                "-v",
                "2",
                "-r",
                "-s",
                str(CONFIG.paths.world_path),
            ],
            env=self.environment,
            stdout=self.log,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self._wait_for_topics()

    def tearDown(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.log.close()

    def _topic(self, *arguments, timeout=8):
        return subprocess.run(
            ["gz", "topic", *arguments],
            env=self.environment,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=True,
        ).stdout

    def _wait_for_topics(self):
        required = {
            CONFIG.simulation.command_topic,
            CONFIG.simulation.odometry_topic,
            CONFIG.simulation.lidar_topic,
            CONFIG.simulation.imu_topic,
            CONFIG.simulation.joint_state_topic,
        }
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                self.log.seek(0)
                self.fail(f"Gazebo exited early:\n{self.log.read()}")
            topics = set(self._topic("-l", timeout=3).splitlines())
            if required.issubset(topics):
                return
            time.sleep(0.2)
        self.log.seek(0)
        self.fail(f"AMR topics did not appear:\n{self.log.read()}")

    @staticmethod
    def _position_x(odometry: str) -> float:
        match = re.search(
            r"position\s*\{[^}]*?x:\s*([-+0-9.eE]+)", odometry, re.DOTALL
        )
        return 0.0 if match is None else float(match.group(1))

    def test_sensors_publish_and_velocity_changes_odometry(self):
        for topic in (
            CONFIG.simulation.lidar_topic,
            CONFIG.simulation.imu_topic,
            CONFIG.simulation.joint_state_topic,
        ):
            with self.subTest(topic=topic):
                message = self._topic("-e", "-t", topic, "-n", "1")
                self.assertTrue(message.strip())

        before = self._topic(
            "-e", "-t", CONFIG.simulation.odometry_topic, "-n", "1"
        )
        self._topic(
            "-t",
            CONFIG.simulation.command_topic,
            "-m",
            "gz.msgs.Twist",
            "-p",
            "linear: {x: 0.12}",
        )
        time.sleep(1.0)
        after = self._topic(
            "-e", "-t", CONFIG.simulation.odometry_topic, "-n", "1"
        )
        self._topic(
            "-t",
            CONFIG.simulation.command_topic,
            "-m",
            "gz.msgs.Twist",
            "-p",
            "linear: {x: 0.0} angular: {z: 0.0}",
        )

        self.assertGreater(
            abs(self._position_x(after) - self._position_x(before)), 0.02
        )


if __name__ == "__main__":
    unittest.main()
