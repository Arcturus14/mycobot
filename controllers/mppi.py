"""Simulator-independent kinematic MPPI for a Cartesian point goal."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from numpy.typing import ArrayLike, NDArray
from .kinematics import end_effector_position, forward_kinematics, position_jacobian

CONTROL_DT = 0.02  # 50 Hz
MAX_JOINT_VELOCITY = np.full(6, 2.0944)  # 120 deg/s
JOINT_MIN = np.array([-2.9321, -2.4434, -2.6179, -2.6179, -2.7052, -3.14159])
JOINT_MAX = np.array([2.9321, 2.4434, 2.6179, 2.6179, 2.7925, 3.14159])


@dataclass(frozen=True)
class MPPIConfig:
    dt: float = CONTROL_DT
    horizon: int = 10
    num_samples: int = 136
    temperature: float = 1.0
    noise_sigma: float = 0.65
    w_position: float = 180.0
    w_terminal: float = 900.0
    w_control: float = 0.025
    w_smooth: float = 0.06
    w_joint_limit: float = 1000.0
    w_collision: float = 2500.0
    w_stop: float = 1.5
    joint_margin: float = float(np.deg2rad(5.0))
    collision_clearance: float = 0.012
    slowdown_radius: float = 0.060
    goal_tolerance: float = 0.005


@dataclass(frozen=True)
class MPPIDiagnostics:
    minimum_cost: float
    effective_sample_size: float
    predicted_terminal_error_m: float


def _self_collision_penalty(origins: NDArray[np.float64], clearance: float) -> NDArray[np.float64]:
    """Conservative DH-frame sphere approximation for non-neighbouring links."""
    radii = np.array([0.045, 0.040, 0.035, 0.033, 0.030, 0.027, 0.025])
    pairs = ((0, 3), (0, 4), (0, 5), (0, 6), (1, 4), (1, 5), (1, 6),
             (2, 5), (2, 6), (3, 6))
    total = np.zeros(origins.shape[:-2], dtype=float)
    for first, second in pairs:
        distance = np.linalg.norm(origins[..., first, :] - origins[..., second, :], axis=-1)
        penetration = np.maximum(radii[first] + radii[second] + clearance - distance, 0.0)
        total += penetration * penetration
    return total


class KinematicMPPI:
    """MPPI with state q, input dq, and rollout q[t+1]=q[t]+dt*dq[t]."""
    def __init__(self, config: MPPIConfig | None = None, seed: int | None = 7) -> None:
        self.config = config or MPPIConfig()
        self.rng = np.random.default_rng(seed)
        self.nominal = np.zeros((self.config.horizon, 6), dtype=float)
        self.previous = np.zeros(6, dtype=float)
        self.last_diagnostics: MPPIDiagnostics | None = None

    def _guided_proposal(self, q: NDArray[np.float64], goal: NDArray[np.float64]):
        """Create a DLS Cartesian-servo proposal; MPPI still scores and mixes it."""
        proposal = np.zeros_like(self.nominal)
        predicted = q.copy()
        damping_sq = 0.02**2
        for step in range(self.config.horizon):
            error = goal - end_effector_position(predicted)
            jacobian = position_jacobian(predicted)
            dq = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping_sq * np.eye(3), 2.0 * error
            )
            proposal[step] = np.clip(dq, -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY)
            predicted = np.clip(predicted + self.config.dt * proposal[step], JOINT_MIN, JOINT_MAX)
        return proposal

    def _cost(self, states, controls, goal, previous):
        cfg = self.config
        ee = end_effector_position(states)
        error_sq = np.sum((ee - goal[None, None, :]) ** 2, axis=-1)
        cost = cfg.w_position * np.sum(error_sq, axis=1)
        cost += cfg.w_terminal * error_sq[:, -1]
        cost += cfg.w_control * np.sum(controls * controls, axis=(1, 2))
        prefix = np.broadcast_to(previous, (controls.shape[0], 1, 6))
        delta = np.diff(np.concatenate((prefix, controls), axis=1), axis=1)
        cost += cfg.w_smooth * np.sum(delta * delta, axis=(1, 2))
        safe_min, safe_max = JOINT_MIN + cfg.joint_margin, JOINT_MAX - cfg.joint_margin
        violation = np.maximum(safe_min - states, 0.0) ** 2 + np.maximum(states - safe_max, 0.0) ** 2
        cost += cfg.w_joint_limit * np.sum(violation, axis=(1, 2))
        _, origins = forward_kinematics(states, return_origins=True)
        collision = _self_collision_penalty(origins, cfg.collision_clearance)
        cost += cfg.w_collision * np.sum(collision, axis=1)
        distance = np.sqrt(error_sq)
        stop_gate = np.exp(-((distance / cfg.slowdown_radius) ** 2))
        cost += cfg.w_stop * np.sum(stop_gate[..., None] * controls * controls, axis=(1, 2))
        return cost, np.sqrt(error_sq[:, -1])

    def command(self, q_current: ArrayLike, p_goal_m: ArrayLike) -> NDArray[np.float64]:
        q, goal = np.asarray(q_current, dtype=float), np.asarray(p_goal_m, dtype=float)
        if q.shape != (6,) or goal.shape != (3,):
            raise ValueError("q_current and p_goal_m must have shapes (6,) and (3,)")
        noise = self.rng.normal(0.0, self.config.noise_sigma,
                                (self.config.num_samples, self.config.horizon, 6))
        candidates = np.clip(self.nominal[None, :, :] + noise,
                             -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY)
        guided = self._guided_proposal(q, goal)
        guided_count = max(1, self.config.num_samples // 8)
        candidates[:guided_count] = np.clip(
            guided[None, :, :] + 0.25 * noise[:guided_count],
            -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY,
        )
        candidates[0] = guided
        states = q[None, None, :] + self.config.dt * np.cumsum(candidates, axis=1)
        costs, terminal_errors = self._cost(states, candidates, goal, self.previous)
        weights = np.exp(-(costs - np.min(costs)) / self.config.temperature)
        weights /= max(float(np.sum(weights)), np.finfo(float).tiny)
        self.nominal += np.einsum("k,khn->hn", weights,
                                  candidates - self.nominal[None, :, :], optimize=True)
        self.nominal = np.clip(self.nominal, -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY)
        raw = self.nominal[0].copy()
        distance = float(np.linalg.norm(end_effector_position(q) - goal))
        command = np.zeros(6) if distance <= self.config.goal_tolerance else raw * min(1.0, distance / self.config.slowdown_radius)
        command = np.clip(command, -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY)
        self.previous = command.copy()
        self.last_diagnostics = MPPIDiagnostics(float(np.min(costs)),
                                                float(1.0 / np.sum(weights * weights)),
                                                float(terminal_errors[np.argmin(costs)]))
        self.nominal[:-1] = self.nominal[1:]
        self.nominal[-1] = 0.0
        return command

    def reset(self) -> None:
        self.nominal.fill(0.0)
        self.previous.fill(0.0)
        self.last_diagnostics = None
