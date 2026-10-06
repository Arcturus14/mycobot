"""Small controller boundary that can later be replaced by MPPI.

Nothing in this module imports Isaac Sim.  A controller receives measured
joint state and returns either a position or velocity command.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray


NUM_ARM_JOINTS = 6


def _joint_vector(value: ArrayLike, name: str) -> NDArray[np.float64]:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (NUM_ARM_JOINTS,):
        raise ValueError(f"{name} must have shape ({NUM_ARM_JOINTS},), got {vector.shape}")
    return vector.copy()


@dataclass(frozen=True)
class JointState:
    q: NDArray[np.float64]
    dq: NDArray[np.float64]

    def __post_init__(self) -> None:
        object.__setattr__(self, "q", _joint_vector(self.q, "q"))
        object.__setattr__(self, "dq", _joint_vector(self.dq, "dq"))


class PositionTargetController:
    """Return a fixed joint-position target for the smoke test."""

    command_type = "position"

    def __init__(self, q_target: ArrayLike) -> None:
        self.q_target = _joint_vector(q_target, "q_target")

    def compute(self, state: JointState) -> NDArray[np.float64]:
        del state
        return self.q_target.copy()


class VelocityTargetController:
    """Return a fixed joint-velocity target for the smoke test."""

    command_type = "velocity"

    def __init__(self, dq_target: ArrayLike) -> None:
        self.dq_target = _joint_vector(dq_target, "dq_target")

    def compute(self, state: JointState) -> NDArray[np.float64]:
        del state
        return self.dq_target.copy()
