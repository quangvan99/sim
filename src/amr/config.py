"""Central configuration for the standalone AMR simulation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class PathsConfig:
    """Project and packaged simulation asset locations."""

    project_root: Path
    sim_root: Path
    world_path: Path
    models_root: Path
    visualize_root: Path


@dataclass(frozen=True)
class SimulationConfig:
    """Names and Gazebo Transport topics shared by all AMR components."""

    world_name: str = "amr_world"
    model_name: str = "amr"
    command_topic: str = "/amr/cmd_vel"
    odometry_topic: str = "/amr/odom"
    lidar_topic: str = "/amr/scan"
    imu_topic: str = "/amr/imu"
    joint_state_topic: str = "/amr/joint_states"

    @property
    def world_control_service(self) -> str:
        return f"/world/{self.world_name}/control"

    @property
    def dynamic_pose_topic(self) -> str:
        return f"/world/{self.world_name}/dynamic_pose/info"


@dataclass(frozen=True)
class ControlConfig:
    """Manual velocity control and telemetry safety limits."""

    max_linear_velocity_mps: float = 0.22
    max_angular_velocity_rps: float = 2.84
    command_timeout_seconds: float = 0.3
    publish_hz: float = 20.0
    sensor_timeout_seconds: float = 2.0
    reset_timeout_ms: int = 5_000


@dataclass(frozen=True)
class ServerConfig:
    """HTTP, WebSocket, and Gazebo partition settings."""

    host: str = "0.0.0.0"
    http_port: int = 8010
    websocket_port: int = 9003
    partition: str = "amr_sim"


@dataclass(frozen=True)
class AppConfig:
    """Immutable AMR application configuration."""

    paths: PathsConfig
    simulation: SimulationConfig
    control: ControlConfig
    server: ServerConfig


_PACKAGE_ROOT = Path(__file__).resolve().parent
_PROJECT_ROOT = _PACKAGE_ROOT.parent.parent
_SIM_ROOT = _PACKAGE_ROOT / "sim"

CONFIG = AppConfig(
    paths=PathsConfig(
        project_root=_PROJECT_ROOT,
        sim_root=_SIM_ROOT,
        world_path=_SIM_ROOT / "worlds" / "main.sdf",
        models_root=_SIM_ROOT / "models",
        visualize_root=_SIM_ROOT / "visualize",
    ),
    simulation=SimulationConfig(),
    control=ControlConfig(),
    server=ServerConfig(),
)
