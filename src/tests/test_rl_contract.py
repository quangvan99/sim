"""Contract tests for the shared CartPole Gymnasium environment."""

from __future__ import annotations

import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path

import numpy as np

from cartpole.contract import (
    ACTION_FORCE_N,
    CART_LIMIT_M,
    CONTROL_STEP_SECONDS,
    DEFAULT_DISTURBANCE_CONTROL_STEPS,
    DEFAULT_DISTURBANCE_FORCE_N,
    DEFAULT_MAX_EPISODE_STEPS,
    MAX_DISTURBANCE_FORCE_N,
    PHYSICS_STEP_SECONDS,
    PHYSICS_STEPS_PER_ACTION,
    POLE_LIMIT_RAD,
    action_force,
    physical_termination_reason,
    validate_policy_contract,
)
from cartpole.env import DEFAULT_WORLD_PATH, CartPoleEnv
from cartpole.runtime import (
    BALANCING,
    ERROR,
    IDLE,
    SUCCESS,
    RLRuntime,
    RuntimeCommandError,
)
from cartpole.server import ensure_tcp_port_available
from cartpole.train import SymmetryAugmentedEnv


class FakeBackend:
    def __init__(self):
        self.state = np.zeros(4, dtype=np.float32)
        self.terminated = False
        self.reason = None
        self.actions: list[int] = []
        self.disturbances: list[tuple[float, int]] = []
        self.reset_count = 0
        self.thread_ids: list[int] = []

    def _record_thread(self) -> None:
        self.thread_ids.append(threading.get_ident())

    def reset(self) -> np.ndarray:
        self._record_thread()
        self.reset_count += 1
        self.state = np.zeros(4, dtype=np.float32)
        self.terminated = False
        self.reason = None
        return self.state.copy()

    def step(self, action: int) -> tuple[np.ndarray, bool, str | None]:
        self._record_thread()
        self.actions.append(action)
        return self.state.copy(), self.terminated, self.reason

    def queue_disturbance(self, force_n: float, control_steps: int) -> None:
        self._record_thread()
        self.disturbances.append((force_n, control_steps))


class FakePolicy:
    def __init__(self, env: CartPoleEnv):
        self.observation_space = env.observation_space
        self.action_space = env.action_space
        self.predict_calls: list[tuple[int, bool]] = []

    def predict(self, observation, deterministic=False):
        del observation
        self.predict_calls.append((threading.get_ident(), deterministic))
        return np.int64(1), None


class CartPoleContractTest(unittest.TestCase):
    def setUp(self):
        self.backend = FakeBackend()
        self.env = CartPoleEnv(backend=self.backend)

    def test_frozen_constants(self):
        self.assertEqual(ACTION_FORCE_N, 2000.0)
        self.assertEqual(PHYSICS_STEP_SECONDS, 0.001)
        self.assertEqual(PHYSICS_STEPS_PER_ACTION, 5)
        self.assertEqual(CONTROL_STEP_SECONDS, 0.005)
        self.assertEqual(POLE_LIMIT_RAD, 0.48)
        self.assertEqual(CART_LIMIT_M, 4.8)
        self.assertEqual(DEFAULT_MAX_EPISODE_STEPS, 6000)
        self.assertEqual(DEFAULT_DISTURBANCE_FORCE_N, 3000.0)
        self.assertEqual(MAX_DISTURBANCE_FORCE_N, 3000.0)
        self.assertTrue(Path(DEFAULT_WORLD_PATH).is_file())

    def test_spaces_match_contract(self):
        self.assertEqual(self.env.action_space.n, 2)
        self.assertEqual(self.env.observation_space.shape, (4,))
        self.assertEqual(self.env.observation_space.dtype, np.dtype(np.float32))
        np.testing.assert_array_equal(
            self.env.observation_space.low[[0, 2]],
            np.array([-10.0, -0.96], dtype=np.float32),
        )
        np.testing.assert_array_equal(
            self.env.observation_space.high[[0, 2]],
            np.array([10.0, 0.96], dtype=np.float32),
        )

    def test_reset_returns_float32_observation_and_does_not_count_step(self):
        observation, info = self.env.reset(seed=123)

        self.assertEqual(self.backend.reset_count, 1)
        self.assertEqual(observation.shape, (4,))
        self.assertEqual(observation.dtype, np.float32)
        self.assertEqual(info, {"episode_step": 0, "simulation_time": 0.0})

    def test_actions_are_forwarded_without_remapping(self):
        self.env.reset()

        for action in (0, 1):
            _, reward, terminated, truncated, info = self.env.step(action)

            self.assertEqual(reward, 1.0)
            self.assertFalse(terminated)
            self.assertFalse(truncated)

        self.assertEqual(self.backend.actions, [0, 1])
        self.assertEqual(info["episode_step"], 2)
        self.assertEqual(info["simulation_time"], 0.01)

    def test_action_force_mapping(self):
        self.assertEqual(action_force(0), -2000.0)
        self.assertEqual(action_force(1), 2000.0)
        with self.assertRaises(ValueError):
            action_force(2)

    def test_invalid_action_is_rejected(self):
        self.env.reset()

        with self.assertRaises(ValueError):
            self.env.step(2)

    def test_physical_failure_terminates_with_zero_reward(self):
        self.env.reset()
        self.backend.state[2] = np.float32(POLE_LIMIT_RAD + 0.01)
        self.backend.terminated = True
        self.backend.reason = "pole_limit"

        _, reward, terminated, truncated, info = self.env.step(1)

        self.assertEqual(reward, 0.0)
        self.assertTrue(terminated)
        self.assertFalse(truncated)
        self.assertEqual(info["termination_reason"], "pole_limit")
        with self.assertRaises(RuntimeError):
            self.env.step(1)

    def test_physical_limits_can_be_ignored_for_visual_demo(self):
        env = CartPoleEnv(
            backend=self.backend,
            terminate_on_pole_limit=False,
            terminate_on_cart_limit=False,
        )
        env.reset()
        self.backend.terminated = True
        self.backend.reason = "cart_limit"

        _, reward, terminated, truncated, info = env.step(1)

        self.assertEqual(reward, 1.0)
        self.assertFalse(terminated)
        self.assertFalse(truncated)
        self.assertIsNone(info["termination_reason"])

    def test_physical_limits_are_strict_and_report_a_reason(self):
        self.assertIsNone(
            physical_termination_reason(CART_LIMIT_M, POLE_LIMIT_RAD)
        )
        self.assertEqual(
            physical_termination_reason(
                0.0, np.nextafter(POLE_LIMIT_RAD, np.inf)
            ),
            "pole_limit",
        )
        self.assertEqual(
            physical_termination_reason(
                np.nextafter(CART_LIMIT_M, np.inf), 0.0
            ),
            "cart_limit",
        )

    def test_time_limit_truncates_a_valid_episode(self):
        env = CartPoleEnv(backend=self.backend, max_episode_steps=2)
        env.reset()

        env.step(0)
        _, reward, terminated, truncated, info = env.step(1)

        self.assertEqual(reward, 1.0)
        self.assertFalse(terminated)
        self.assertTrue(truncated)
        self.assertEqual(info["termination_reason"], "time_limit")

    def test_physical_failure_wins_on_final_time_step(self):
        env = CartPoleEnv(backend=self.backend, max_episode_steps=1)
        env.reset()
        self.backend.terminated = True
        self.backend.reason = "cart_limit"

        _, reward, terminated, truncated, info = env.step(0)

        self.assertEqual(reward, 0.0)
        self.assertTrue(terminated)
        self.assertFalse(truncated)
        self.assertEqual(info["termination_reason"], "cart_limit")

    def test_disturbance_direction_duration_and_clamp(self):
        self.env.reset()

        self.env.apply_disturbance("left")
        self.env.apply_disturbance("right", force_n=9999, control_steps=3)

        self.assertEqual(
            self.backend.disturbances,
            [
                (-DEFAULT_DISTURBANCE_FORCE_N, DEFAULT_DISTURBANCE_CONTROL_STEPS),
                (MAX_DISTURBANCE_FORCE_N, 3),
            ],
        )

    def test_invalid_disturbance_is_rejected(self):
        self.env.reset()

        with self.assertRaises(ValueError):
            self.env.apply_disturbance("up")
        with self.assertRaises(ValueError):
            self.env.apply_disturbance("left", force_n=0)
        with self.assertRaises(ValueError):
            self.env.apply_disturbance("left", control_steps=0)

    def test_training_symmetry_mirrors_observation_and_action(self):
        training_env = SymmetryAugmentedEnv(self.env, seed=123)
        training_env.reset()
        training_env._mirrored = True
        self.backend.state = np.array([1.0, 2.0, 0.1, 0.2], dtype=np.float32)

        observation, *_ = training_env.step(0)

        self.assertEqual(self.backend.actions[-1], 1)
        np.testing.assert_array_equal(observation, -self.backend.state)

    def test_generated_checkpoint_matches_runtime_contract(self):
        checkpoint = (
            Path(__file__).resolve().parents[1]
            / "checkpoints"
            / "ppo_cartpole.zip"
        )
        self.assertTrue(checkpoint.is_file())

        from stable_baselines3 import PPO

        model = PPO.load(checkpoint)
        validate_policy_contract(model, self.env)


class RLRuntimeTest(unittest.TestCase):
    checkpoint = (
        Path(__file__).resolve().parents[1]
        / "checkpoints"
        / "ppo_cartpole.zip"
    )

    def wait_for_phase(self, runtime: RLRuntime, phase: str, timeout: float = 2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            snapshot = runtime.snapshot()
            if snapshot["phase"] == phase:
                return snapshot
            time.sleep(0.001)
        self.fail(f"Runtime did not reach {phase}: {runtime.snapshot()}")

    def make_runtime(self, *, max_episode_steps=1000, control_period=0.001):
        backend = FakeBackend()
        created = {}

        def env_factory():
            env = CartPoleEnv(
                backend=backend,
                max_episode_steps=max_episode_steps,
            )
            created["env"] = env
            return env

        def model_loader(checkpoint):
            self.assertEqual(checkpoint, self.checkpoint)
            model = FakePolicy(created["env"])
            created["model"] = model
            return model

        runtime = RLRuntime(
            self.checkpoint,
            env_factory=env_factory,
            model_loader=model_loader,
            control_period=control_period,
        )
        return runtime, backend, created

    def test_runtime_runs_deterministic_policy_to_success(self):
        runtime, backend, created = self.make_runtime(
            max_episode_steps=3,
            control_period=0,
        )
        try:
            initialized = runtime.wait_until_initialized(2.0)
            self.assertEqual(initialized["phase"], IDLE)
            self.assertTrue(initialized["ready"])

            runtime.start_episode()
            result = self.wait_for_phase(runtime, SUCCESS)

            self.assertEqual(result["episode_time"], 0.015)
            self.assertEqual(result["action"], 1)
            self.assertEqual(result["force"], ACTION_FORCE_N)
            self.assertEqual(result["result"], "Simulation ended after 30.0s")
            self.assertEqual(backend.actions, [1, 1, 1])
            self.assertTrue(
                all(deterministic for _, deterministic in created["model"].predict_calls)
            )
            worker_ids = set(backend.thread_ids)
            worker_ids.update(
                thread_id for thread_id, _ in created["model"].predict_calls
            )
            self.assertEqual(len(worker_ids), 1)
            self.assertNotIn(threading.get_ident(), worker_ids)
        finally:
            runtime.close()

    def test_runtime_queues_disturbance_and_reset(self):
        runtime, backend, _ = self.make_runtime()
        try:
            runtime.wait_until_initialized(2.0)
            with self.assertRaises(RuntimeCommandError):
                runtime.apply_disturbance("left")

            runtime.start_episode()
            self.wait_for_phase(runtime, BALANCING)
            runtime.apply_disturbance("left")

            deadline = time.monotonic() + 2.0
            while not backend.disturbances and time.monotonic() < deadline:
                time.sleep(0.001)
            self.assertEqual(
                backend.disturbances,
                [(-DEFAULT_DISTURBANCE_FORCE_N, DEFAULT_DISTURBANCE_CONTROL_STEPS)],
            )

            runtime.reset()
            reset = self.wait_for_phase(runtime, IDLE)
            self.assertEqual(reset["episode_time"], 0.0)
            self.assertIsNone(reset["action"])
            self.assertGreaterEqual(backend.reset_count, 3)
        finally:
            runtime.close()

    def test_missing_checkpoint_transitions_to_error_without_gazebo(self):
        env_created = False

        def env_factory():
            nonlocal env_created
            env_created = True
            return CartPoleEnv(backend=FakeBackend())

        with tempfile.TemporaryDirectory() as directory:
            missing = Path(directory) / "missing.zip"
            runtime = RLRuntime(missing, env_factory=env_factory)
            try:
                snapshot = runtime.wait_until_initialized(2.0)
                self.assertEqual(snapshot["phase"], ERROR)
                self.assertFalse(snapshot["ready"])
                self.assertIn("Checkpoint not found", snapshot["error"])
                self.assertFalse(env_created)
            finally:
                runtime.close()

    def test_rl_startup_rejects_occupied_webview_port(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.bind(("127.0.0.1", 0))
            listener.listen()

            with self.assertRaisesRegex(RuntimeError, "already in use"):
                ensure_tcp_port_available(listener.getsockname()[1])


if __name__ == "__main__":
    unittest.main()
