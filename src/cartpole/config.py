"""Central configuration for the CartPole application."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class PathsConfig:
    """Project and packaged asset locations."""

    project_root: Path
    sim_root: Path
    world_path: Path
    training_world_path: Path
    visualize_root: Path
    checkpoint_path: Path


@dataclass(frozen=True)
class CartPoleConfig:
    """Simulation and RL contract values."""

    world_name: str = "cart_pole"
    model_name: str = "vehicle_green"
    pole_link_name: str = "pole"
    chassis_link_name: str = "chassis"

    action_force_n: float = 2000.0
    physics_steps_per_action: int = 5
    physics_step_seconds: float = 0.001

    pole_limit_rad: float = 0.48
    cart_limit_m: float = 4.8
    max_episode_steps: int = 6000

    disturbance_force_n: float = 3000.0
    disturbance_control_steps: int = 10
    max_disturbance_force_n: float = 3000.0

    observation_position_limit_m: float = 10.0
    observation_angle_limit_rad: float = 0.96

    @property
    def control_step_seconds(self) -> float:
        return self.physics_steps_per_action * self.physics_step_seconds

    @property
    def observation_low(self) -> tuple[float, float, float, float]:
        float_max = float(np.finfo(np.float32).max)
        return (
            -self.observation_position_limit_m,
            -np.inf,
            -self.observation_angle_limit_rad,
            -float_max,
        )

    @property
    def observation_high(self) -> tuple[float, float, float, float]:
        float_max = float(np.finfo(np.float32).max)
        return (
            self.observation_position_limit_m,
            np.inf,
            self.observation_angle_limit_rad,
            float_max,
        )


@dataclass(frozen=True)
class ServerConfig:
    """HTTP bridge and Gazebo transport settings."""

    command_topic: str = "/model/vehicle_green/force_cmd"
    entity_name: str = "vehicle_green::chassis"
    max_force_n: float = 2000.0
    command_timeout_seconds: float = 0.2
    publish_hz: float = 50.0
    websocket_port: int = 9002
    host: str = "0.0.0.0"
    http_port: int = 8009


@dataclass(frozen=True)
class TrainingConfig:
    """PPO training and evaluation settings."""

    output_path: Path
    total_steps: int = 25_000
    seed: int = 123
    disturbance_at_step: int = 1_000
    ppo_rollout_steps: int = 2_048
    ppo_batch_size: int = 64
    ppo_gamma: float = 0.999
    balancing_prior: tuple[float, float, float, float] = (
        -0.01,
        -0.005,
        1.0,
        0.2,
    )
    prior_logit_scale: float = 100.0


@dataclass(frozen=True)
class AppConfig:
    """Immutable application configuration grouped by responsibility."""

    paths: PathsConfig
    cartpole: CartPoleConfig
    server: ServerConfig
    training: TrainingConfig


_PACKAGE_ROOT = Path(__file__).resolve().parent
_PROJECT_ROOT = _PACKAGE_ROOT.parent.parent
_SOURCE_ROOT = _PROJECT_ROOT / "src"
_SIM_ROOT = _PACKAGE_ROOT / "sim"

_PATHS = PathsConfig(
    project_root=_PROJECT_ROOT,
    sim_root=_SIM_ROOT,
    world_path=_SIM_ROOT / "worlds" / "main.sdf",
    training_world_path=_SIM_ROOT / "worlds" / "train.sdf",
    visualize_root=_SIM_ROOT / "visualize",
    checkpoint_path=_SOURCE_ROOT / "checkpoints" / "ppo_cartpole.zip",
)
_CARTPOLE = CartPoleConfig()
_SERVER = ServerConfig(max_force_n=_CARTPOLE.action_force_n)
_TRAINING = TrainingConfig(output_path=_PATHS.checkpoint_path)

CONFIG = AppConfig(
    paths=_PATHS,
    cartpole=_CARTPOLE,
    server=_SERVER,
    training=_TRAINING,
)
