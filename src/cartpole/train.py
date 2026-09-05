"""Train and validate the PPO checkpoint used by CartPole V2 runtime."""

from __future__ import annotations

import argparse
import os
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

# Isolate offline simulation from a running V1 control bridge. Gazebo reads
# this setting when its Python bindings create transport and server objects.
os.environ.setdefault("GZ_PARTITION", f"amr_cartpole_train_{os.getpid()}")

import gymnasium as gym
import numpy as np

# Gazebo bindings must load before PyTorch / Stable-Baselines3 on macOS. Loading
# them in the opposite order can abort the process with a native mutex error.
from cartpole.contract import (
    DEFAULT_DISTURBANCE_CONTROL_STEPS,
    DEFAULT_DISTURBANCE_FORCE_N,
    DEFAULT_MAX_EPISODE_STEPS,
    validate_policy_contract,
)
from cartpole.env import CartPoleEnv

if TYPE_CHECKING:
    from stable_baselines3 import PPO


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SOURCE_ROOT = PROJECT_ROOT / "src"
DEFAULT_OUTPUT = SOURCE_ROOT / "checkpoints" / "ppo_cartpole.zip"
DEFAULT_TRAINING_STEPS = 25_000
DEFAULT_SEED = 123
DISTURBANCE_AT_STEP = 1_000
PPO_ROLLOUT_STEPS = 2_048
PPO_BATCH_SIZE = 64
PPO_GAMMA = 0.999
BALANCING_PRIOR = np.array([-0.01, -0.005, 1.0, 0.2], dtype=np.float32)
PRIOR_LOGIT_SCALE = 100.0


@dataclass(frozen=True)
class EvaluationResult:
    scenario: str
    passed: bool
    episode_steps: int
    reason: str | None


class SymmetryAugmentedEnv(gym.Wrapper):
    """Expose mirrored episodes so PPO cannot learn a left/right bias."""

    def __init__(self, env: CartPoleEnv, *, seed: int):
        super().__init__(env)
        self._rng = np.random.default_rng(seed)
        self._mirrored = False

    def reset(self, **kwargs):
        observation, info = self.env.reset(**kwargs)
        self._mirrored = bool(self._rng.integers(2))
        if self._mirrored:
            observation = -observation
        return observation, info

    def step(self, action):
        backend_action = 1 - int(action) if self._mirrored else int(action)
        observation, reward, terminated, truncated, info = self.env.step(
            backend_action
        )
        if self._mirrored:
            observation = -observation
        return observation, reward, terminated, truncated, info


def normalized_checkpoint_path(value: str | Path) -> Path:
    """Return the concrete ``.zip`` path written by Stable-Baselines3."""

    path = Path(value).expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    if path.suffix != ".zip":
        path = path.with_suffix(".zip")
    return path.resolve()


def initialize_balancing_prior(model: PPO) -> None:
    """Initialize the linear PPO actor with a symmetric stabilizing prior."""

    import torch

    action_net = model.policy.action_net
    if tuple(action_net.weight.shape) != (2, 4):
        raise RuntimeError("Balancing prior requires a linear 4-to-2 actor")

    prior = torch.as_tensor(
        BALANCING_PRIOR * PRIOR_LOGIT_SCALE,
        dtype=action_net.weight.dtype,
        device=action_net.weight.device,
    )
    with torch.no_grad():
        action_net.weight[0].copy_(-0.5 * prior)
        action_net.weight[1].copy_(0.5 * prior)
        action_net.bias.zero_()


def remove_action_bias(model: PPO) -> None:
    """Preserve the left/right symmetry of the deterministic policy."""

    import torch

    with torch.no_grad():
        model.policy.action_net.bias.zero_()


def evaluate(
    model: PPO,
    env: CartPoleEnv,
    *,
    scenario: str,
    disturbance: str | None,
) -> EvaluationResult:
    """Run one deterministic 30-second acceptance episode."""

    observation, _ = env.reset(seed=DEFAULT_SEED)
    for episode_step in range(1, DEFAULT_MAX_EPISODE_STEPS + 1):
        if disturbance is not None and episode_step == DISTURBANCE_AT_STEP:
            env.apply_disturbance(
                disturbance,
                force_n=DEFAULT_DISTURBANCE_FORCE_N,
                control_steps=DEFAULT_DISTURBANCE_CONTROL_STEPS,
            )

        action, _ = model.predict(observation, deterministic=True)
        observation, _, terminated, truncated, info = env.step(int(action))
        if terminated or truncated:
            return EvaluationResult(
                scenario=scenario,
                passed=truncated and not terminated,
                episode_steps=episode_step,
                reason=info["termination_reason"],
            )

    raise RuntimeError(f"Evaluation scenario did not finish: {scenario}")


def train(args: argparse.Namespace) -> Path:
    """Train, save, reload, and evaluate a checkpoint."""

    checkpoint_path = normalized_checkpoint_path(args.output)
    if checkpoint_path.exists() and not args.overwrite:
        raise FileExistsError(
            f"Checkpoint already exists: {checkpoint_path}. Use --overwrite to replace it."
        )
    checkpoint_path.parent.mkdir(parents=True, exist_ok=True)

    # On macOS, Gazebo's TestFixture must exist before PyTorch loads. Importing
    # Stable-Baselines3 first can abort in native code with a mutex error.
    env = CartPoleEnv()
    training_env = SymmetryAugmentedEnv(env, seed=args.seed)
    from stable_baselines3 import PPO

    model = PPO(
        "MlpPolicy",
        training_env,
        verbose=1,
        seed=args.seed,
        n_steps=PPO_ROLLOUT_STEPS,
        batch_size=PPO_BATCH_SIZE,
        gamma=PPO_GAMMA,
        policy_kwargs={"net_arch": []},
    )
    initialize_balancing_prior(model)
    model.learn(total_timesteps=args.steps)
    remove_action_bias(model)
    model.save(str(checkpoint_path.with_suffix("")))

    loaded_model = PPO.load(str(checkpoint_path))
    validate_policy_contract(loaded_model, env)

    results = [
        evaluate(loaded_model, env, scenario="baseline", disturbance=None),
        evaluate(loaded_model, env, scenario="push_left", disturbance="left"),
        evaluate(loaded_model, env, scenario="push_right", disturbance="right"),
    ]
    for result in results:
        outcome = "PASS" if result.passed else "FAIL"
        print(
            f"Evaluation {result.scenario}: {outcome} "
            f"({result.episode_steps} steps, reason={result.reason})"
        )

    failures = [result.scenario for result in results if not result.passed]
    if failures:
        raise RuntimeError(
            "Checkpoint failed evaluation scenarios: " + ", ".join(failures)
        )

    print(f"Checkpoint ready: {checkpoint_path}")
    return checkpoint_path


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=positive_int, default=DEFAULT_TRAINING_STEPS)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace an existing checkpoint at --output.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        train(args)
    except (FileExistsError, RuntimeError) as error:
        raise SystemExit(str(error)) from error


if __name__ == "__main__":
    main()
