"""Load-only PPO runtime for the CartPole V2 browser demo."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass
from enum import StrEnum
from pathlib import Path
from queue import Empty, Queue
from typing import Self

import numpy as np

from cartpole.config import CONFIG
from cartpole.env import CartPoleEnv


class Phase(StrEnum):
    LOADING = "LOADING"
    IDLE = "IDLE"
    BALANCING = "BALANCING"
    FAILED = "FAILED"
    SUCCESS = "SUCCESS"
    ERROR = "ERROR"


@dataclass(frozen=True)
class RuntimeSnapshot:
    phase: Phase = Phase.LOADING
    ready: bool = False
    pole_angle_deg: float = 0.0
    action: int | None = None
    force: float = 0.0
    episode_time: float = 0.0
    result: str | None = None
    termination_reason: str | None = None
    error: str | None = None

    def as_dict(self) -> dict:
        return asdict(self)


class RLRuntime:
    """Own Gazebo and deterministic PPO inference on one worker thread."""

    def __init__(
        self,
        checkpoint: str | Path = CONFIG.paths.checkpoint_path,
        *,
        env_factory: Callable[[], CartPoleEnv] | None = None,
        model_loader: Callable[[Path], object] | None = None,
        control_period: float = CONFIG.cartpole.control_step_seconds,
    ):
        if control_period < 0:
            raise ValueError("control_period must not be negative")

        checkpoint_path = Path(checkpoint).expanduser()
        if not checkpoint_path.is_absolute():
            checkpoint_path = CONFIG.paths.project_root / checkpoint_path
        if checkpoint_path.suffix != ".zip":
            checkpoint_path = checkpoint_path.with_suffix(".zip")

        self.checkpoint = checkpoint_path.resolve()
        self._env_factory = env_factory
        self._model_loader = model_loader
        self._control_period = control_period
        self._commands: Queue[tuple[str, str | None]] = Queue()
        self._snapshot_lock = threading.Lock()
        self._snapshot = RuntimeSnapshot()
        self._initialized = threading.Event()
        self._stopped = threading.Event()
        self._thread = threading.Thread(
            target=self._worker,
            name="cartpole-rl-worker",
            daemon=True,
        )
        self._thread.start()

    def snapshot(self) -> dict:
        with self._snapshot_lock:
            return self._snapshot.as_dict()

    def wait_until_initialized(self, timeout: float | None = None) -> dict:
        self._initialized.wait(timeout)
        return self.snapshot()

    def start_episode(self) -> None:
        self._enqueue_for_phases(
            "start", {Phase.IDLE, Phase.FAILED, Phase.SUCCESS}
        )

    def reset(self) -> None:
        self._enqueue_for_phases(
            "reset", {Phase.IDLE, Phase.BALANCING, Phase.FAILED, Phase.SUCCESS}
        )

    def apply_disturbance(self, direction: str) -> None:
        if direction not in {"left", "right"}:
            raise ValueError("direction must be 'left' or 'right'")
        self._enqueue_for_phases("disturbance", {Phase.BALANCING}, direction)

    def close(self, timeout: float = 5.0) -> None:
        if self._stopped.is_set():
            return
        self._stopped.set()
        self._commands.put(("stop", None))
        self._thread.join(timeout)
        if self._thread.is_alive():
            raise RuntimeError("RL worker did not stop")

    def __enter__(self) -> Self:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def _enqueue_for_phases(
        self,
        name: str,
        allowed_phases: set[Phase],
        value: str | None = None,
    ) -> None:
        with self._snapshot_lock:
            phase = self._snapshot.phase
        if phase not in allowed_phases:
            raise RuntimeError(f"{name} is not allowed while phase={phase}")
        self._commands.put((name, value))

    def _set_snapshot(self, snapshot: RuntimeSnapshot) -> None:
        with self._snapshot_lock:
            self._snapshot = snapshot

    def _worker(self) -> None:
        env = None
        try:
            if not self.checkpoint.is_file():
                raise FileNotFoundError(f"Checkpoint not found: {self.checkpoint}")

            # Construct Gazebo before loading PPO; native libraries on macOS
            # can abort when initialized in the opposite order.
            env = (
                self._env_factory()
                if self._env_factory is not None
                else CartPoleEnv(terminate_on_limits=False)
            )
            if self._model_loader is None:
                # Keep PyTorch / SB3 lazy: Gazebo's TestFixture must be
                # initialized first on macOS or native libraries can abort.
                from stable_baselines3 import PPO

                model = PPO.load(str(self.checkpoint))
            else:
                model = self._model_loader(self.checkpoint)
            observation, _ = env.reset()
            self._set_ready_phase(Phase.IDLE, observation)
            self._initialized.set()

            next_step_at = time.monotonic()
            while not self._stopped.is_set():
                if self.snapshot()["phase"] != Phase.BALANCING:
                    try:
                        command = self._commands.get(timeout=0.1)
                    except Empty:
                        continue
                    command_name, command_value = command
                    if command_name == "stop":
                        break
                    observation = self._handle_command(
                        command_name, command_value, env, observation
                    )
                    next_step_at = time.monotonic()
                    continue

                observation = self._drain_commands(env, observation)
                if (
                    self._stopped.is_set()
                    or self.snapshot()["phase"] != Phase.BALANCING
                ):
                    continue

                action_value, _ = model.predict(observation, deterministic=True)
                action = int(np.asarray(action_value).item())
                observation, _, terminated, truncated, info = env.step(action)
                self._set_balancing_or_result(
                    observation, action, terminated, truncated, info
                )

                if self._control_period > 0:
                    next_step_at += self._control_period
                    remaining = next_step_at - time.monotonic()
                    if remaining > 0:
                        self._stopped.wait(remaining)
                    elif remaining < -self._control_period:
                        next_step_at = time.monotonic()
        except Exception as error:  # noqa: BLE001 - expose runtime failures
            self._set_snapshot(
                RuntimeSnapshot(phase=Phase.ERROR, error=str(error))
            )
        finally:
            self._initialized.set()
            if env is not None:
                env.close()

    def _drain_commands(self, env, observation: np.ndarray) -> np.ndarray:
        while True:
            try:
                command = self._commands.get_nowait()
            except Empty:
                return observation
            command_name, command_value = command
            if command_name == "stop":
                self._stopped.set()
                return observation
            observation = self._handle_command(
                command_name, command_value, env, observation
            )

    def _handle_command(
        self,
        command_name: str,
        command_value: str | None,
        env,
        observation: np.ndarray,
    ) -> np.ndarray:
        if command_name == "start":
            observation, _ = env.reset()
            self._set_ready_phase(Phase.BALANCING, observation)
        elif command_name == "reset":
            observation, _ = env.reset()
            self._set_ready_phase(Phase.IDLE, observation)
        elif command_name == "disturbance":
            # A command accepted near the final step may become stale before
            # the worker dequeues it.  Dropping it is safer than disturbing a
            # completed episode and moving the whole runtime to ERROR.
            if self.snapshot()["phase"] == Phase.BALANCING:
                env.apply_disturbance(command_value)
        else:
            raise RuntimeError(f"Unknown RL command: {command_name}")
        return observation

    def _set_ready_phase(self, phase: Phase, observation: np.ndarray) -> None:
        self._set_snapshot(
            RuntimeSnapshot(
                phase=phase,
                ready=True,
                pole_angle_deg=math.degrees(float(observation[2])),
            )
        )

    def _set_balancing_or_result(
        self,
        observation: np.ndarray,
        action: int,
        terminated: bool,
        truncated: bool,
        info: dict,
    ) -> None:
        reason = info.get("termination_reason")
        phase = Phase.BALANCING
        result = None
        if terminated:
            phase = Phase.FAILED
            result = "Episode failed"
        elif truncated:
            phase = Phase.SUCCESS
            result = "Simulation ended after 30.0s"

        self._set_snapshot(
            RuntimeSnapshot(
                phase=phase,
                ready=True,
                pole_angle_deg=math.degrees(float(observation[2])),
                action=action,
                force=CONFIG.cartpole.action_force_n * (
                    -1.0 if action == 0 else 1.0
                ),
                episode_time=float(info["simulation_time"]),
                result=result,
                termination_reason=reason,
            )
        )
