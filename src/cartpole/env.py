"""Shared Gymnasium environment for the Gazebo CartPole model.

This module contains no training loop and never starts a Gazebo desktop GUI.
Both offline training and runtime inference must use this implementation so
that their observation and action contracts remain identical.
"""

from __future__ import annotations

import os
from pathlib import Path

import gymnasium as gym
import numpy as np
from gz.math import Vector3d
from gz.sim import Link, Model, TestFixture, World, world_entity

from cartpole.config import CONFIG

os.environ["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
    filter(None, (str(CONFIG.paths.sim_root), os.environ.get("GZ_SIM_RESOURCE_PATH")))
)


class GazeboCartPoleBackend:
    """Drive and observe CartPole through a Gazebo ``TestFixture``."""

    def __init__(self, world_path: str | Path = CONFIG.paths.world_path):
        resolved_world_path = Path(world_path).expanduser().resolve()
        if not resolved_world_path.is_file():
            raise FileNotFoundError(
                f"CartPole world not found: {resolved_world_path}"
            )

        self._command: int | None = None
        self._state: np.ndarray | None = None
        self._disturbance_force_n = 0.0
        self._disturbance_steps_remaining = 0
        self._pole: Link | None = None
        self._chassis: Link | None = None

        self._fixture = TestFixture(str(resolved_world_path))
        self._fixture.on_pre_update(self._on_pre_update)
        self._fixture.on_post_update(self._on_post_update)
        self._fixture.finalize()
        self._server = self._fixture.server()

    def _on_pre_update(self, info, ecm) -> None:
        if info.paused:
            return

        world = World(world_entity(ecm))
        model = Model(world.model_by_name(ecm, CONFIG.cartpole.model_name))
        self._pole = Link(
            model.link_by_name(ecm, CONFIG.cartpole.pole_link_name)
        )
        self._chassis = Link(
            model.link_by_name(ecm, CONFIG.cartpole.chassis_link_name)
        )
        self._pole.enable_velocity_checks(ecm)
        self._chassis.enable_velocity_checks(ecm)

        if self._command is not None:
            command_sign = -1.0 if self._command == 0 else 1.0
            self._chassis.add_world_force(
                ecm,
                Vector3d(
                    command_sign * CONFIG.cartpole.action_force_n,
                    0,
                    0,
                ),
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

    def _run_control_interval(self) -> None:
        completed = self._server.run(
            True, CONFIG.cartpole.physics_steps_per_action, False
        )
        if completed is False:
            raise RuntimeError("Gazebo failed to run the CartPole control interval")
        if self._state is None:
            raise RuntimeError("Gazebo did not produce a CartPole observation")

    def reset(self) -> np.ndarray:
        self._server.reset_all()
        self._command = None
        self._state = None
        self._disturbance_force_n = 0.0
        self._disturbance_steps_remaining = 0
        self._run_control_interval()
        return self._state.copy()

    def step(self, action: int) -> np.ndarray:
        self._command = action
        self._run_control_interval()
        if self._disturbance_steps_remaining > 0:
            self._disturbance_steps_remaining -= 1
            if self._disturbance_steps_remaining == 0:
                self._disturbance_force_n = 0.0
        return self._state.copy()

    def queue_disturbance(self, force_n: float, control_steps: int) -> None:
        self._disturbance_force_n = force_n
        self._disturbance_steps_remaining = control_steps


class CartPoleEnv(gym.Env[np.ndarray, int]):
    """Gymnasium wrapper for the Gazebo CartPole simulation."""

    def __init__(
        self,
        world_path: str | Path = CONFIG.paths.world_path,
        *,
        backend=None,
        max_episode_steps: int = CONFIG.cartpole.max_episode_steps,
        terminate_on_limits: bool = True,
    ):
        super().__init__()
        if max_episode_steps <= 0:
            raise ValueError("max_episode_steps must be positive")

        self.action_space = gym.spaces.Discrete(2)
        self.observation_space = gym.spaces.Box(
            low=np.asarray(CONFIG.cartpole.observation_low, dtype=np.float32),
            high=np.asarray(CONFIG.cartpole.observation_high, dtype=np.float32),
            dtype=np.float32,
        )
        self.max_episode_steps = max_episode_steps
        self.terminate_on_limits = terminate_on_limits
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
        self._episode_steps = 0
        self._episode_done = False
        observation = self._as_observation(self._backend.reset())
        return observation, {"episode_step": 0, "simulation_time": 0.0}

    def step(self, action: int) -> tuple[np.ndarray, float, bool, bool, dict]:
        if self._episode_done:
            raise RuntimeError("Episode is complete; call reset() before step()")
        if not self.action_space.contains(action):
            raise ValueError(f"Invalid CartPole action: {action!r}")

        observation = self._as_observation(self._backend.step(int(action)))
        self._episode_steps += 1
        reason = None
        if abs(float(observation[2])) > CONFIG.cartpole.pole_limit_rad:
            reason = "pole_limit"
        elif abs(float(observation[0])) > CONFIG.cartpole.cart_limit_m:
            reason = "cart_limit"
        terminated = reason is not None
        if reason is not None and not self.terminate_on_limits:
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
            "simulation_time": (
                self._episode_steps * CONFIG.cartpole.control_step_seconds
            ),
            "termination_reason": reason,
        }
        return observation, reward, terminated, truncated, info

    def apply_disturbance(
        self,
        direction: str,
        *,
        force_n: float = CONFIG.cartpole.disturbance_force_n,
        control_steps: int = CONFIG.cartpole.disturbance_control_steps,
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

        bounded_force = min(
            float(force_n), CONFIG.cartpole.max_disturbance_force_n
        )
        signed_force = -bounded_force if direction == "left" else bounded_force
        self._backend.queue_disturbance(signed_force, int(control_steps))
