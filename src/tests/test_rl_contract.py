"""Contract tests for the shared CartPole Gymnasium environment."""

from __future__ import annotations

import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from cartpole.config import CONFIG
from cartpole.env import CartPoleEnv
from cartpole.runtime import Phase, RLRuntime
from cartpole.server import ensure_port_available
from cartpole.train import SymmetryAugmentedEnv


class FakeBackend:
    def __init__(self):
        self.state = np.zeros(4, dtype=np.float32)
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
        return self.state.copy()

    def step(self, action: int) -> np.ndarray:
        self._record_thread()
        self.actions.append(action)
        return self.state.copy()

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
        self.assertEqual(CONFIG.cartpole.action_force_n, 2000.0)
        self.assertEqual(CONFIG.cartpole.physics_step_seconds, 0.001)
        self.assertEqual(CONFIG.cartpole.physics_steps_per_action, 5)
        self.assertEqual(CONFIG.cartpole.control_step_seconds, 0.005)
        self.assertEqual(CONFIG.cartpole.pole_limit_rad, 0.48)
        self.assertEqual(CONFIG.cartpole.cart_limit_m, 4.8)
        self.assertEqual(CONFIG.cartpole.max_episode_steps, 6000)
        self.assertEqual(CONFIG.cartpole.runtime_ground_limit_m, 48.0)
        self.assertEqual(CONFIG.cartpole.disturbance_force_n, 3000.0)
        self.assertEqual(CONFIG.cartpole.max_disturbance_force_n, 3000.0)
        self.assertTrue(Path(CONFIG.paths.world_path).is_file())
        self.assertTrue(Path(CONFIG.paths.training_world_path).is_file())

    @patch("cartpole.server.socket.socket")
    def test_server_rejects_occupied_websocket_port(self, socket_factory):
        socket_factory.return_value.__enter__.return_value.bind.side_effect = (
            OSError("address already in use")
        )

        with self.assertRaisesRegex(RuntimeError, r"Port 9002.*already in use"):
            ensure_port_available(9002)

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

    def test_invalid_action_is_rejected(self):
        self.env.reset()

        with self.assertRaises(ValueError):
            self.env.step(2)

    def test_physical_failure_terminates_with_zero_reward(self):
        self.env.reset()
        self.backend.state[2] = np.float32(
            CONFIG.cartpole.pole_limit_rad + 0.01
        )

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
            terminate_on_limits=False,
        )
        env.reset()
        self.backend.state[0] = np.float32(CONFIG.cartpole.cart_limit_m + 0.01)

        _, reward, terminated, truncated, info = env.step(1)

        self.assertEqual(reward, 1.0)
        self.assertFalse(terminated)
        self.assertFalse(truncated)
        self.assertIsNone(info["termination_reason"])

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
        self.backend.state[0] = np.float32(CONFIG.cartpole.cart_limit_m + 0.01)

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
                (
                    -CONFIG.cartpole.disturbance_force_n,
                    CONFIG.cartpole.disturbance_control_steps,
                ),
                (CONFIG.cartpole.max_disturbance_force_n, 3),
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

    def test_generated_checkpoint_can_be_loaded(self):
        checkpoint = (
            Path(__file__).resolve().parents[1]
            / "checkpoints"
            / "ppo_cartpole.zip"
        )
        self.assertTrue(checkpoint.is_file())

        from stable_baselines3 import PPO

        model = PPO.load(checkpoint)
        self.assertIsNotNone(model)


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

    def make_runtime(
        self, *, max_episode_steps=1000, control_period=0.001, backend=None
    ):
        backend = backend or FakeBackend()
        created = {}

        def env_factory():
            env = CartPoleEnv(
                backend=backend,
                max_episode_steps=max_episode_steps,
                terminate_on_limits=False,
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
            self.assertEqual(initialized["phase"], Phase.IDLE)
            self.assertTrue(initialized["ready"])

            runtime.start_episode()
            result = self.wait_for_phase(runtime, Phase.SUCCESS)

            self.assertEqual(result["episode_time"], 0.015)
            self.assertEqual(result["action"], 1)
            self.assertEqual(result["force"], CONFIG.cartpole.action_force_n)
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

    def test_runtime_stops_at_ground_edge(self):
        class EscapingBackend(FakeBackend):
            def step(self, action: int) -> np.ndarray:
                super().step(action)
                self.state[0] = np.float32(49.0)
                return self.state.copy()

        runtime, backend, _ = self.make_runtime(
            control_period=0,
            backend=EscapingBackend(),
        )
        try:
            runtime.wait_until_initialized(2.0)
            runtime.start_episode()
            result = self.wait_for_phase(runtime, Phase.FAILED)

            self.assertEqual(result["termination_reason"], "ground_limit")
            self.assertEqual(len(backend.actions), 1)
        finally:
            runtime.close()

    def test_runtime_queues_disturbance_and_reset(self):
        runtime, backend, _ = self.make_runtime()
        try:
            runtime.wait_until_initialized(2.0)
            with self.assertRaises(RuntimeError):
                runtime.apply_disturbance("left")

            runtime.start_episode()
            self.wait_for_phase(runtime, Phase.BALANCING)
            runtime.apply_disturbance("left")

            deadline = time.monotonic() + 2.0
            while not backend.disturbances and time.monotonic() < deadline:
                time.sleep(0.001)
            self.assertEqual(
                backend.disturbances,
                [
                    (
                        -CONFIG.cartpole.disturbance_force_n,
                        CONFIG.cartpole.disturbance_control_steps,
                    )
                ],
            )

            runtime.reset()
            reset = self.wait_for_phase(runtime, Phase.IDLE)
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
                self.assertEqual(snapshot["phase"], Phase.ERROR)
                self.assertFalse(snapshot["ready"])
                self.assertIn("Checkpoint not found", snapshot["error"])
                self.assertFalse(env_created)
            finally:
                runtime.close()

if __name__ == "__main__":
    unittest.main()
