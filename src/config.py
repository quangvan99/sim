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
    navigation_map_path: Path
    navigation_test_world_path: Path


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

    max_linear_velocity_mps: float = 0.90
    max_angular_velocity_rps: float = 2.84
    command_timeout_seconds: float = 0.3
    publish_hz: float = 20.0
    sensor_timeout_seconds: float = 2.0
    reset_timeout_ms: int = 5_000


@dataclass(frozen=True)
class ServerConfig:
    """HTTP, WebSocket, and Gazebo partition settings."""

    host: str = "0.0.0.0"
    http_port: int = 8009
    websocket_port: int = 9002
    partition: str = "amr_sim"


@dataclass(frozen=True)
class NavigationConfig:
    """Deterministic A* and DWA-lite defaults."""

    map_version: str = "amr-world-nav-v1"
    robot_radius_m: float = 0.13
    safety_margin_m: float = 0.05
    local_safety_margin_m: float = 0.03
    goal_tolerance_m: float = 0.04
    waypoint_spacing_m: float = 0.25
    astar_preferred_clearance_m: float = 0.38
    astar_clearance_cost_weight: float = 2.0
    lookahead_distance_m: float = 0.45
    min_linear_velocity_mps: float = -0.06
    slow_linear_velocity_mps: float = 0.30
    default_linear_velocity_mps: float = 0.50
    fast_linear_velocity_mps: float = 0.90
    max_angular_velocity_rps: float = 1.50
    linear_acceleration_mps2: float = 0.50
    angular_acceleration_rps2: float = 2.00
    control_hz: float = 5.0
    heartbeat_hz: float = 10.0
    prediction_horizon_seconds: float = 2.0
    prediction_step_seconds: float = 0.10
    linear_samples: int = 7
    angular_samples: int = 21
    lidar_count: int = 360
    lidar_sensor_x_m: float = -0.032
    lidar_sensor_y_m: float = 0.0
    lidar_sensor_yaw_rad: float = 0.0
    sensor_timeout_seconds: float = 0.60
    no_progress_timeout_seconds: float = 8.0
    minimum_progress_m: float = 0.05
    temporary_obstacle_ttl_seconds: float = 5.0
    score_progress: float = 2.4
    score_heading: float = 1.5
    score_clearance: float = 2.8
    score_speed: float = 0.6
    score_path_deviation: float = 1.8
    score_smoothness: float = 0.25
    reverse_penalty: float = 0.35


@dataclass(frozen=True)
class AppConfig:
    """Immutable AMR application configuration."""

    paths: PathsConfig
    simulation: SimulationConfig
    control: ControlConfig
    server: ServerConfig
    navigation: NavigationConfig


_PACKAGE_ROOT = Path(__file__).resolve().parent
_PROJECT_ROOT = _PACKAGE_ROOT.parent
_SIM_ROOT = _PACKAGE_ROOT / "sim"

CONFIG = AppConfig(
    paths=PathsConfig(
        project_root=_PROJECT_ROOT,
        sim_root=_SIM_ROOT,
        world_path=_SIM_ROOT / "worlds" / "main.sdf",
        models_root=_SIM_ROOT / "models",
        visualize_root=_SIM_ROOT / "visualize",
        navigation_map_path=_SIM_ROOT / "maps" / "amr_world.json",
        navigation_test_world_path=_SIM_ROOT
        / "worlds"
        / "navigation_test.sdf",
    ),
    simulation=SimulationConfig(),
    control=ControlConfig(),
    server=ServerConfig(),
    navigation=NavigationConfig(),
)
