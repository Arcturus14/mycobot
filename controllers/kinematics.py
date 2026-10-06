"""Vectorized MyCobot 280 forward kinematics using the vendor DH table."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray


# Standard DH: Rz(theta + offset) Tz(d) Tx(a) Rx(alpha), metres/radians.
# Source: Elephant Robotics MyCobotBasic/ParameterList.h (myCobot 280).
DH_A = np.array([0.0, -0.1104, -0.0960, 0.0, 0.0, 0.0])
DH_D = np.array([0.13156, 0.0, 0.0, 0.06462, 0.07318, 0.04860])
DH_ALPHA = np.array([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0])
DH_THETA_OFFSET = np.array([0.0, -np.pi / 2, 0.0, -np.pi / 2, np.pi / 2, 0.0])


def forward_kinematics(q: ArrayLike, *, return_origins: bool = False):
    """Return base-frame flange pose(s); accepts ``(..., 6)`` joint arrays."""
    joints = np.asarray(q, dtype=float)
    if joints.shape[-1:] != (6,):
        raise ValueError(f"q must end in dimension 6, got {joints.shape}")
    batch = joints.shape[:-1]
    transform = np.broadcast_to(np.eye(4), batch + (4, 4)).copy()
    origins = [transform[..., :3, 3].copy()]
    for index in range(6):
        theta = joints[..., index] + DH_THETA_OFFSET[index]
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(DH_ALPHA[index]), np.sin(DH_ALPHA[index])
        link = np.zeros(batch + (4, 4), dtype=float)
        link[..., 0, 0] = ct
        link[..., 0, 1] = -st * ca
        link[..., 0, 2] = st * sa
        link[..., 0, 3] = DH_A[index] * ct
        link[..., 1, 0] = st
        link[..., 1, 1] = ct * ca
        link[..., 1, 2] = -ct * sa
        link[..., 1, 3] = DH_A[index] * st
        link[..., 2, 1] = sa
        link[..., 2, 2] = ca
        link[..., 2, 3] = DH_D[index]
        link[..., 3, 3] = 1.0
        transform = transform @ link
        origins.append(transform[..., :3, 3].copy())
    if return_origins:
        return transform, np.stack(origins, axis=-2)
    return transform


def end_effector_position(q: ArrayLike) -> NDArray[np.float64]:
    return forward_kinematics(q)[..., :3, 3]


def position_jacobian(q: ArrayLike, epsilon: float = 1e-6) -> NDArray[np.float64]:
    """Numerical 3x6 position Jacobian used only to seed MPPI proposals."""
    joints = np.asarray(q, dtype=float)
    if joints.shape != (6,):
        raise ValueError(f"q must have shape (6,), got {joints.shape}")
    jacobian = np.empty((3, 6), dtype=float)
    for index in range(6):
        delta = np.zeros(6)
        delta[index] = epsilon
        jacobian[:, index] = (
            end_effector_position(joints + delta) - end_effector_position(joints - delta)
        ) / (2.0 * epsilon)
    return jacobian
