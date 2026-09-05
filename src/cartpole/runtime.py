"""Load-only PPO runtime for the CartPole V2 browser demo."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass
from pathlib import Path
from queue import Empty, Queue
from typing import Self

import numpy as np

from cartpole.contract import (
    CONTROL_STEP_SECONDS,
    action_force,
    validate_policy_contract,
)
from cartpole.env import CartPoleEnv

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SOURCE_ROOT = PROJECT_ROOT / "src"
DEFAULT_CHECKPOINT = SOURCE_ROOT / "checkpoints" / "ppo_cartpole.zip"

LOADING = "LOADING"
IDLE = "IDLE"
BALANCING = "BALANCING"
FAILED = "FAILED"
SUCCESS = "SUCCESS"
ERROR = "ERROR"


class RuntimeCommandError(RuntimeError):
    """Raised when the current runtime phase rejects a browser command."""


@dataclass(frozen=True)
class RuntimeSnapshot:
    phase: str = LOADING
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


@dataclass(frozen=True)
class _Command:
    name: str
    value: str | None = None


def normalized_checkpoint_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    if path.suffix != ".zip":
        path = path.with_suffix(".zip")
    return path.resolve()


def _load_ppo(checkpoint: Path):
    # Keep PyTorch / SB3 lazy: Gazebo's TestFixture must be initialized first
    # on macOS or the native libraries can abort while creating their mutexes.
    from stable_baselines3 import PPO

    return PPO.load(str(checkpoint))


def _make_runtime_env() -> CartPoleEnv:
    """Create the viewer environment and let the pole fall naturally."""

    return CartPoleEnv(
        terminate_on_pole_limit=False,
        terminate_on_cart_limit=False,
    )


class RLRuntime:
    """Own Gazebo and deterministic PPO inference on one worker thread."""

    def __init__(
        self,
        checkpoint: str | Path = DEFAULT_CHECKPOINT,
        *,
        env_factory: Callable[[], CartPoleEnv] = _make_runtime_env,
        model_loader: Callable[[Path], object] = _load_ppo,
        control_period: float = CONTROL_STEP_SECONDS,
    ):
        if control_period < 0:
            raise ValueError("control_period must not be negative")

        self.checkpoint = normalized_checkpoint_path(checkpoint)
        self._env_factory = env_factory
        self._model_loader = model_loader
        self._control_period = control_period
        self._commands: Queue[_Command] = Queue()
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
        self._enqueue_for_phases("start", {IDLE, FAILED, SUCCESS})

    def reset(self) -> None:
        self._enqueue_for_phases("reset", {IDLE, BALANCING, FAILED, SUCCESS})

    def apply_disturbance(self, direction: str) -> None:
        if direction not in {"left", "right"}:
            raise ValueError("direction must be 'left' or 'right'")
        self._enqueue_for_phases("disturbance", {BALANCING}, direction)

    def close(self, timeout: float = 5.0) -> None:
        if self._stopped.is_set():
            return
        self._stopped.set()
        self._commands.put(_Command("stop"))
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
        allowed_phases: set[str],
        value: str | None = None,
    ) -> None:
        with self._snapshot_lock:
            phase = self._snapshot.phase
        if phase not in allowed_phases:
            raise RuntimeCommandError(f"{name} is not allowed while phase={phase}")
        self._commands.put(_Command(name, value))

    def _set_snapshot(self, snapshot: RuntimeSnapshot) -> None:
        with self._snapshot_lock:
            self._snapshot = snapshot

    def _worker(self) -> None:
        env = None
        try:
            if not self.checkpoint.is_file():
                raise FileNotFoundError(f"Checkpoint not found: {self.checkpoint}")

            # Construct Gazebo before loading PPO; see _load_ppo().
            env = self._env_factory()
            model = self._model_loader(self.checkpoint)
            validate_policy_contract(model, env)
            observation, _ = env.reset()
            self._set_idle(observation)
            self._initialized.set()

            next_step_at = time.monotonic()
            while not self._stopped.is_set():
                if self.snapshot()["phase"] != BALANCING:
                    try:
                        command = self._commands.get(timeout=0.1)
                    except Empty:
                        continue
                    if command.name == "stop":
                        break
                    observation = self._handle_command(command, env, observation)
                    next_step_at = time.monotonic()
                    continue

                observation = self._drain_commands(env, observation)
                if self._stopped.is_set() or self.snapshot()["phase"] != BALANCING:
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
            self._set_snapshot(RuntimeSnapshot(phase=ERROR, error=str(error)))
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
            if command.name == "stop":
                self._stopped.set()
                return observation
            observation = self._handle_command(command, env, observation)

    def _handle_command(self, command, env, observation: np.ndarray) -> np.ndarray:
        if command.name == "start":
            observation, _ = env.reset()
            self._set_snapshot(
                RuntimeSnapshot(
                    phase=BALANCING,
                    ready=True,
                    pole_angle_deg=math.degrees(float(observation[2])),
                )
            )
        elif command.name == "reset":
            observation, _ = env.reset()
            self._set_idle(observation)
        elif command.name == "disturbance":
            # A command accepted near the final step may become stale before
            # the worker dequeues it.  Dropping it is safer than disturbing a
            # completed episode and moving the whole runtime to ERROR.
            if self.snapshot()["phase"] == BALANCING:
                env.apply_disturbance(command.value)
        else:
            raise RuntimeError(f"Unknown RL command: {command.name}")
        return observation

    def _set_idle(self, observation: np.ndarray) -> None:
        self._set_snapshot(
            RuntimeSnapshot(
                phase=IDLE,
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
        phase = BALANCING
        result = None
        if terminated:
            phase = FAILED
            result = "Episode failed"
        elif truncated:
            phase = SUCCESS
            result = "Simulation ended after 30.0s"

        self._set_snapshot(
            RuntimeSnapshot(
                phase=phase,
                ready=True,
                pole_angle_deg=math.degrees(float(observation[2])),
                action=action,
                force=action_force(action),
                episode_time=float(info["simulation_time"]),
                result=result,
                termination_reason=reason,
            )
        )
