"""Shared Gymnasium environment for the Gazebo CartPole model.

This module contains no training loop and never starts a Gazebo desktop GUI.
Both offline training and runtime inference must use this implementation so
that their observation and action contracts remain identical.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import ClassVar, Protocol

import gymnasium as gym
import numpy as np
from gz.math import Vector3d
from gz.sim import Link, Model, TestFixture, World, world_entity

from cartpole.contract import (
    CHASSIS_LINK_NAME,
    CONTROL_STEP_SECONDS,
    DEFAULT_DISTURBANCE_CONTROL_STEPS,
    DEFAULT_DISTURBANCE_FORCE_N,
    DEFAULT_MAX_EPISODE_STEPS,
    MAX_DISTURBANCE_FORCE_N,
    MODEL_NAME,
    OBSERVATION_HIGH,
    OBSERVATION_LOW,
    PHYSICS_STEPS_PER_ACTION,
    POLE_LINK_NAME,
    action_force,
    physical_termination_reason,
)

PACKAGE_ROOT = Path(__file__).resolve().parent
SIM_ROOT = PACKAGE_ROOT / "sim"
DEFAULT_WORLD_PATH = SIM_ROOT / "worlds" / "main.sdf"


def _configure_gazebo_resource_path() -> None:
    """Make package models resolvable by Gazebo's ``<uri>`` lookup."""

    sim_root = str(SIM_ROOT)
    configured = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    paths = [path for path in configured.split(os.pathsep) if path]
    if sim_root not in paths:
        os.environ["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
            [sim_root, *paths]
        )


_configure_gazebo_resource_path()


class CartPoleBackend(Protocol):
    """Minimal simulation interface consumed by :class:`CartPoleEnv`."""

    def reset(self) -> np.ndarray:
        """Reset the simulation and return its initial observation."""

    def step(self, action: int) -> tuple[np.ndarray, bool, str | None]:
        """Apply one action and return observation, failure, and reason."""

    def queue_disturbance(self, force_n: float, control_steps: int) -> None:
        """Apply an external force during upcoming control intervals."""


class GazeboCartPoleBackend:
    """Drive and observe CartPole through a Gazebo ``TestFixture``."""

    def __init__(self, world_path: str | Path = DEFAULT_WORLD_PATH):
        self.world_path = Path(world_path).expanduser().resolve()
        if not self.world_path.is_file():
            raise FileNotFoundError(f"CartPole world not found: {self.world_path}")

        self._command: int | None = None
        self._state: np.ndarray | None = None
        self._terminated = False
        self._termination_reason: str | None = None
        self._disturbance_force_n = 0.0
        self._disturbance_steps_remaining = 0
        self._pole: Link | None = None
        self._chassis: Link | None = None

        self._fixture = TestFixture(str(self.world_path))
        self._fixture.on_pre_update(self._on_pre_update)
        self._fixture.on_post_update(self._on_post_update)
        self._fixture.finalize()
        self._server = self._fixture.server()

    def _on_pre_update(self, info, ecm) -> None:
        if info.paused:
            return

        world = World(world_entity(ecm))
        model = Model(world.model_by_name(ecm, MODEL_NAME))
        self._pole = Link(model.link_by_name(ecm, POLE_LINK_NAME))
        self._chassis = Link(model.link_by_name(ecm, CHASSIS_LINK_NAME))
        self._pole.enable_velocity_checks(ecm)
        self._chassis.enable_velocity_checks(ecm)

        if self._command is not None:
            self._chassis.add_world_force(
                ecm, Vector3d(action_force(self._command), 0, 0)
            )

        if self._disturbance_steps_remaining > 0:
            self._chassis.add_world_force(
                ecm, Vector3d(self._disturbance_force_n, 0, 0)
            )

    def _on_post_update(self, info, ecm) -> None:
        if info.paused or self._pole is None or self._chassis is None:
            return

        pole_velocity = self._pole.world_angular_velocity(ecm)
        cart_velocity = self._chassis.world_linear_velocity(ecm)
        pole_angular_velocity = 0.0 if pole_velocity is None else pole_velocity.y()
        cart_linear_velocity = 0.0 if cart_velocity is None else cart_velocity.x()

        pole_pitch = self._pole.world_pose(ecm).rot().euler().y()
        cart_position = self._chassis.world_pose(ecm).pos().x()
        self._state = np.array(
            [cart_position, cart_linear_velocity, pole_pitch, pole_angular_velocity],
            dtype=np.float32,
        )

        if not self._terminated:
            self._termination_reason = physical_termination_reason(
                cart_position, pole_pitch
            )
            self._terminated = self._termination_reason is not None

    def _run_control_interval(self) -> None:
        completed = self._server.run(True, PHYSICS_STEPS_PER_ACTION, False)
        if completed is False:
            raise RuntimeError("Gazebo failed to run the CartPole control interval")
        if self._state is None:
            raise RuntimeError("Gazebo did not produce a CartPole observation")

    def reset(self) -> np.ndarray:
        self._server.reset_all()
        self._command = None
        self._state = None
        self._terminated = False
        self._termination_reason = None
        self._disturbance_force_n = 0.0
        self._disturbance_steps_remaining = 0
        self._run_control_interval()
        # reset_all() may emit an update containing the previous episode's
        # state before the reset state is observed.  Do not let that transient
        # update leave the failure latch set for the new episode.
        self._termination_reason = physical_termination_reason(
            float(self._state[0]), float(self._state[2])
        )
        self._terminated = self._termination_reason is not None
        return self._state.copy()

    def step(self, action: int) -> tuple[np.ndarray, bool, str | None]:
        self._command = action
        self._run_control_interval()
        if self._disturbance_steps_remaining > 0:
            self._disturbance_steps_remaining -= 1
            if self._disturbance_steps_remaining == 0:
                self._disturbance_force_n = 0.0
        return self._state.copy(), self._terminated, self._termination_reason

    def queue_disturbance(self, force_n: float, control_steps: int) -> None:
        self._disturbance_force_n = force_n
        self._disturbance_steps_remaining = control_steps


class CartPoleEnv(gym.Env[np.ndarray, int]):
    """Gymnasium wrapper with the frozen CartPole V2 contract."""

    metadata: ClassVar[dict[str, list[str]]] = {"render_modes": []}

    def __init__(
        self,
        world_path: str | Path = DEFAULT_WORLD_PATH,
        *,
        backend: CartPoleBackend | None = None,
        max_episode_steps: int = DEFAULT_MAX_EPISODE_STEPS,
        terminate_on_pole_limit: bool = True,
        terminate_on_cart_limit: bool = True,
    ):
        super().__init__()
        if max_episode_steps <= 0:
            raise ValueError("max_episode_steps must be positive")

        self.action_space = gym.spaces.Discrete(2)
        self.observation_space = gym.spaces.Box(
            low=OBSERVATION_LOW.copy(),
            high=OBSERVATION_HIGH.copy(),
            dtype=np.float32,
        )
        self.max_episode_steps = max_episode_steps
        self.terminate_on_pole_limit = terminate_on_pole_limit
        self.terminate_on_cart_limit = terminate_on_cart_limit
        self._backend = backend or GazeboCartPoleBackend(world_path)
        self._episode_steps = 0
        self._episode_done = False

    @staticmethod
    def _as_observation(value: np.ndarray) -> np.ndarray:
        observation = np.asarray(value, dtype=np.float32)
        if observation.shape != (4,):
            raise RuntimeError(
                f"CartPole observation must have shape (4,), got {observation.shape}"
            )
        return observation

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict | None = None,
    ) -> tuple[np.ndarray, dict]:
        super().reset(seed=seed)
        del options
        self._episode_steps = 0
        self._episode_done = False
        observation = self._as_observation(self._backend.reset())
        return observation, {"episode_step": 0, "simulation_time": 0.0}

    def step(self, action: int) -> tuple[np.ndarray, float, bool, bool, dict]:
        if self._episode_done:
            raise RuntimeError("Episode is complete; call reset() before step()")
        if not self.action_space.contains(action):
            raise ValueError(f"Invalid CartPole action: {action!r}")

        observation, terminated, reason = self._backend.step(int(action))
        observation = self._as_observation(observation)
        self._episode_steps += 1
        terminated = bool(terminated)
        if (
            reason == "pole_limit" and not self.terminate_on_pole_limit
        ) or (
            reason == "cart_limit" and not self.terminate_on_cart_limit
        ):
            terminated = False
            reason = None
        truncated = (
            not terminated and self._episode_steps >= self.max_episode_steps
        )
        reward = 0.0 if terminated else 1.0
        if truncated:
            reason = "time_limit"

        self._episode_done = terminated or truncated
        info = {
            "episode_step": self._episode_steps,
            "simulation_time": self._episode_steps * CONTROL_STEP_SECONDS,
            "termination_reason": reason,
        }
        return observation, reward, terminated, truncated, info

    def apply_disturbance(
        self,
        direction: str,
        *,
        force_n: float = DEFAULT_DISTURBANCE_FORCE_N,
        control_steps: int = DEFAULT_DISTURBANCE_CONTROL_STEPS,
    ) -> None:
        """Queue a bounded external push for the active episode."""

        if self._episode_done:
            raise RuntimeError("Cannot disturb a completed episode")
        if direction not in {"left", "right"}:
            raise ValueError("direction must be 'left' or 'right'")
        if not np.isfinite(force_n) or force_n <= 0:
            raise ValueError("force_n must be a positive finite number")
        if control_steps <= 0:
            raise ValueError("control_steps must be positive")

        bounded_force = min(float(force_n), MAX_DISTURBANCE_FORCE_N)
        signed_force = -bounded_force if direction == "left" else bounded_force
        self._backend.queue_disturbance(signed_force, int(control_steps))
