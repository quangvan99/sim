"""Frozen action and observation contract for CartPole RL."""

from __future__ import annotations

import numpy as np

MODEL_NAME = "vehicle_green"
POLE_LINK_NAME = "pole"
CHASSIS_LINK_NAME = "chassis"

ACTION_FORCE_N = 2000.0
PHYSICS_STEPS_PER_ACTION = 5
PHYSICS_STEP_SECONDS = 0.001
CONTROL_STEP_SECONDS = PHYSICS_STEPS_PER_ACTION * PHYSICS_STEP_SECONDS

POLE_LIMIT_RAD = 0.48
CART_LIMIT_M = 4.8
DEFAULT_MAX_EPISODE_STEPS = 6000

DEFAULT_DISTURBANCE_FORCE_N = 3000.0
DEFAULT_DISTURBANCE_CONTROL_STEPS = 10
MAX_DISTURBANCE_FORCE_N = 3000.0

FLOAT_MAX = np.finfo(np.float32).max
OBSERVATION_LOW = np.array(
    [-10.0, -np.inf, -0.96, -FLOAT_MAX], dtype=np.float32
)
OBSERVATION_HIGH = np.array(
    [10.0, np.inf, 0.96, FLOAT_MAX], dtype=np.float32
)


def physical_termination_reason(
    cart_position: float, pole_pitch: float
) -> str | None:
    """Return the physical failure reason before float32 conversion."""

    if abs(pole_pitch) > POLE_LIMIT_RAD:
        return "pole_limit"
    if abs(cart_position) > CART_LIMIT_M:
        return "cart_limit"
    return None


def action_force(action: int) -> float:
    """Map a discrete policy action to the frozen X-axis force."""

    if action == 0:
        return -ACTION_FORCE_N
    if action == 1:
        return ACTION_FORCE_N
    raise ValueError(f"Invalid CartPole action: {action!r}")


def validate_policy_contract(model, env) -> None:
    """Fail fast when a loaded policy cannot consume the shared RL contract."""

    if model.observation_space.shape != (4,):
        raise RuntimeError(
            "Checkpoint observation shape must be (4,), got "
            f"{model.observation_space.shape}"
        )
    if getattr(model.action_space, "n", None) != 2:
        raise RuntimeError("Checkpoint action space must be Discrete(2)")
    if model.observation_space.dtype != np.dtype(np.float32):
        raise RuntimeError("Checkpoint observation dtype must be float32")
    if not np.array_equal(model.observation_space.low, OBSERVATION_LOW):
        raise RuntimeError("Checkpoint observation lower bounds do not match")
    if not np.array_equal(model.observation_space.high, OBSERVATION_HIGH):
        raise RuntimeError("Checkpoint observation upper bounds do not match")
    if model.observation_space != env.observation_space:
        raise RuntimeError("Checkpoint and runtime observation spaces differ")
    if model.action_space != env.action_space:
        raise RuntimeError("Checkpoint and runtime action spaces differ")
