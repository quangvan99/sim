"""Static contract tests for the self-contained AMR simulation assets."""

from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from amr.config import CONFIG


class AMRContractTest(unittest.TestCase):
    def test_frozen_contract(self):
        self.assertEqual(CONFIG.simulation.world_name, "amr_world")
        self.assertEqual(CONFIG.simulation.model_name, "amr")
        self.assertEqual(CONFIG.simulation.command_topic, "/amr/cmd_vel")
        self.assertEqual(CONFIG.simulation.odometry_topic, "/amr/odom")
        self.assertEqual(CONFIG.simulation.lidar_topic, "/amr/scan")
        self.assertEqual(CONFIG.simulation.imu_topic, "/amr/imu")
        self.assertEqual(
            CONFIG.simulation.joint_state_topic, "/amr/joint_states"
        )
        self.assertEqual(CONFIG.server.http_port, 8010)
        self.assertEqual(CONFIG.server.websocket_port, 9003)

    def test_required_assets_exist_and_xml_is_valid(self):
        required = (
            CONFIG.paths.world_path,
            CONFIG.paths.models_root / "robot" / "model.sdf",
            CONFIG.paths.models_root / "robot" / "model.config",
            CONFIG.paths.models_root
            / "common"
            / "meshes"
            / "bases"
            / "base.stl",
            CONFIG.paths.models_root
            / "common"
            / "meshes"
            / "sensors"
            / "lidar.stl",
            CONFIG.paths.models_root
            / "common"
            / "meshes"
            / "wheels"
            / "left_wheel.stl",
            CONFIG.paths.models_root
            / "common"
            / "meshes"
            / "wheels"
            / "right_wheel.stl",
            CONFIG.paths.models_root / "world" / "model.sdf",
            CONFIG.paths.models_root
            / "world"
            / "meshes"
            / "hexagon.dae",
            CONFIG.paths.models_root
            / "world"
            / "meshes"
            / "wall.dae",
        )
        for path in required:
            with self.subTest(path=path):
                self.assertTrue(path.is_file())

        for path in CONFIG.paths.sim_root.rglob("*"):
            if path.suffix in {".sdf", ".config"}:
                with self.subTest(xml=path):
                    ET.parse(path)

    def test_world_contains_robot_and_required_systems(self):
        root = ET.parse(CONFIG.paths.world_path).getroot()
        world = root.find("world")
        self.assertIsNotNone(world)
        self.assertEqual(world.attrib["name"], CONFIG.simulation.world_name)

        plugins = {plugin.attrib["name"] for plugin in world.findall("plugin")}
        self.assertTrue(
            {
                "gz::sim::systems::Physics",
                "gz::sim::systems::UserCommands",
                "gz::sim::systems::SceneBroadcaster",
                "gz::sim::systems::Sensors",
                "gz::sim::systems::Imu",
                "gz::sim::systems::WebsocketServer",
            }.issubset(plugins)
        )
        includes = {
            include.findtext("name"): include.findtext("uri")
            for include in world.findall("include")
        }
        self.assertEqual(includes["amr"], "model://robot")
        self.assertEqual(
            includes["environment"], "model://world"
        )

    def test_model_declares_all_transport_topics(self):
        model_path = (
            CONFIG.paths.models_root / "robot" / "model.sdf"
        )
        text = model_path.read_text()
        for topic in (
            CONFIG.simulation.command_topic,
            CONFIG.simulation.odometry_topic,
            CONFIG.simulation.lidar_topic,
            CONFIG.simulation.imu_topic,
            CONFIG.simulation.joint_state_topic,
        ):
            with self.subTest(topic=topic):
                self.assertIn(f">{topic}<", text)

    def test_amr_has_no_runtime_dependency_on_cartpole(self):
        for path in CONFIG.paths.sim_root.parent.rglob("*"):
            with self.subTest(path=path):
                self.assertFalse(path.is_symlink())
            if path.suffix not in {".py", ".sdf", ".html", ".js", ".css"}:
                continue
            text = path.read_text(errors="ignore")
            self.assertNotIn("src/cartpole", text)
            self.assertNotIn("from cartpole", text)
            self.assertNotIn("import cartpole", text)


if __name__ == "__main__":
    unittest.main()
