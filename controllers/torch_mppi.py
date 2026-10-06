"""GPU-capable MyCobot point-goal controller backed by pytorch_mppi.MPPI."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import os
import sys
import time

import numpy as np
from numpy.typing import ArrayLike, NDArray


# Isaac's source build keeps its PyTorch/CUDA wheels outside python.sh's default
# sys.path. Add those bundled packages before importing torch; do not pip-install
# another torch build into the Kit interpreter.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
ISAAC_TARGET_DEPS = Path("/home/arclab/workspace/IsaacSim/_build/target-deps")
os.environ.setdefault("TORCHINDUCTOR_CACHE_DIR", str(PROJECT_ROOT / ".torch_cache" / "inductor"))
os.environ.setdefault("TRITON_CACHE_DIR", str(PROJECT_ROOT / ".torch_cache" / "triton"))
for path in (
    PROJECT_ROOT / "third_party",
    ISAAC_TARGET_DEPS / "isaac_ml_prebundle",
    ISAAC_TARGET_DEPS / "isaac_nv_prebundle",
):
    if path.is_dir() and str(path) not in sys.path:
        sys.path.insert(0, str(path))

import torch
from pytorch_mppi import MPPI
from pytorch_mppi.mppi import SpecificActionSampler

from .mppi import (CONTROL_DT, JOINT_MAX, JOINT_MIN, MAX_JOINT_VELOCITY,
                   MPPIConfig, MPPIDiagnostics)
from .kinematics import end_effector_position, position_jacobian

torch.set_float32_matmul_precision("high")


def _constants(device: torch.device, dtype: torch.dtype):
    return (
        torch.tensor([0.0, -0.1104, -0.0960, 0.0, 0.0, 0.0], device=device, dtype=dtype),
        torch.tensor([0.13156, 0.0, 0.0, 0.06462, 0.07318, 0.04860], device=device, dtype=dtype),
        torch.tensor([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0], device=device, dtype=dtype),
        torch.tensor([0.0, -np.pi / 2, 0.0, -np.pi / 2, np.pi / 2, 0.0], device=device, dtype=dtype),
    )


def torch_forward_kinematics(q: torch.Tensor, *, return_origins: bool = False):
    """Batched standard-DH FK; q has shape ``(..., 6)``."""
    if q.shape[-1] != 6:
        raise ValueError(f"q must end in dimension 6, got {tuple(q.shape)}")
    a, d, alpha, offset = _constants(q.device, q.dtype)
    batch = q.shape[:-1]
    transform = torch.eye(4, device=q.device, dtype=q.dtype).expand(*batch, 4, 4).clone()
    origins = [transform[..., :3, 3].clone()]
    for index in range(6):
        theta = q[..., index] + offset[index]
        ct, st = torch.cos(theta), torch.sin(theta)
        ca, sa = torch.cos(alpha[index]), torch.sin(alpha[index])
        link = torch.zeros(*batch, 4, 4, device=q.device, dtype=q.dtype)
        link[..., 0, 0], link[..., 0, 1] = ct, -st * ca
        link[..., 0, 2], link[..., 0, 3] = st * sa, a[index] * ct
        link[..., 1, 0], link[..., 1, 1] = st, ct * ca
        link[..., 1, 2], link[..., 1, 3] = -ct * sa, a[index] * st
        link[..., 2, 1], link[..., 2, 2], link[..., 2, 3] = sa, ca, d[index]
        link[..., 3, 3] = 1.0
        transform = transform @ link
        origins.append(transform[..., :3, 3].clone())
    if return_origins:
        return transform, torch.stack(origins, dim=-2)
    return transform


def torch_ee_position(q: torch.Tensor) -> torch.Tensor:
    return torch_forward_kinematics(q)[..., :3, 3]


def _position_jacobian(q: torch.Tensor, epsilon: float = 1e-5) -> torch.Tensor:
    columns = []
    for index in range(6):
        delta = torch.zeros_like(q)
        delta[index] = epsilon
        columns.append((torch_ee_position(q + delta) - torch_ee_position(q - delta)) / (2 * epsilon))
    return torch.stack(columns, dim=-1)


class _DLSProposal(SpecificActionSampler):
    """Supply one goal-directed trajectory to the library's normal sample set."""
    def __init__(self, owner: "TorchKinematicMPPI") -> None:
        super().__init__()
        self.owner = owner

    def sample_trajectories(self, state, info):
        # The problem is tiny; doing the 30x numerical Jacobian seed on CPU
        # avoids hundreds of small CUDA kernel launches. Rollouts/costs remain CUDA.
        # command() already received these values on CPU. Reusing them avoids
        # two GPU-to-CPU synchronizations inside every optimizer solve.
        q = self.owner._current_q_cpu.copy()
        goal = self.owner._goal_cpu
        controls = []
        for _ in range(self.owner.config.horizon):
            error = goal - end_effector_position(q)
            jacobian = position_jacobian(q)
            dq = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + 0.02**2 * np.eye(3), 2.0 * error
            )
            dq = np.clip(dq, -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY)
            controls.append(dq)
            q = np.clip(q + self.owner.config.dt * dq, JOINT_MIN, JOINT_MAX)
        return torch.tensor(np.asarray(controls), device=state.device, dtype=state.dtype).unsqueeze(0)


@dataclass(frozen=True)
class TorchBackendInfo:
    device: str
    gpu_name: str | None
    torch_version: str
    library_version: str


class TorchKinematicMPPI:
    """Compatibility wrapper whose optimizer is UM-ARM-Lab pytorch_mppi.MPPI."""
    def __init__(self, config: MPPIConfig | None = None, seed: int = 7,
                 device: str = "auto") -> None:
        self.config = config or MPPIConfig()
        selected = "cuda" if device == "auto" and torch.cuda.is_available() else device
        if selected == "auto":
            selected = "cpu"
        self.device = torch.device(selected)
        self.dtype = torch.float32
        torch.manual_seed(int(seed))
        if self.device.type == "cuda":
            torch.cuda.manual_seed_all(int(seed))
        self.goal = torch.zeros(3, device=self.device, dtype=self.dtype)
        self.q_min = torch.tensor(JOINT_MIN, device=self.device, dtype=self.dtype)
        self.q_max = torch.tensor(JOINT_MAX, device=self.device, dtype=self.dtype)
        self.u_max = torch.tensor(MAX_JOINT_VELOCITY, device=self.device, dtype=self.dtype)
        self.u_min = -self.u_max
        self.previous_action = torch.zeros(6, device=self.device, dtype=self.dtype)
        self.last_diagnostics: MPPIDiagnostics | None = None
        self.last_solve_time_sec = 0.0
        self.last_command_time_sec = 0.0
        self.last_preprocess_time_sec = 0.0
        self.last_postprocess_time_sec = 0.0
        self._current_q_cpu = np.zeros(6, dtype=float)
        self._goal_cpu = np.full(3, np.nan, dtype=float)

        sigma = torch.eye(6, device=self.device, dtype=self.dtype) * self.config.noise_sigma**2
        self._optimizer = MPPI(
            self._dynamics,
            self._running_cost,
            nx=6,
            noise_sigma=sigma,
            num_samples=self.config.num_samples,
            horizon=self.config.horizon,
            device=str(self.device),
            terminal_state_cost=self._terminal_cost,
            lambda_=self.config.temperature,
            u_min=self.u_min,
            u_max=self.u_max,
            u_init=torch.zeros(6, device=self.device, dtype=self.dtype),
            U_init=torch.zeros(self.config.horizon, 6, device=self.device, dtype=self.dtype),
            step_dependent_dynamics=True,
            specific_action_sampler=_DLSProposal(self),
        )
        if self.device.type == "cuda":
            self._optimizer.compile(options={
                "triton.cudagraphs": False,
                "triton.cudagraph_trees": False,
            })

    @property
    def backend_info(self) -> TorchBackendInfo:
        import importlib.metadata
        gpu_name = torch.cuda.get_device_name(self.device) if self.device.type == "cuda" else None
        return TorchBackendInfo(str(self.device), gpu_name, torch.__version__,
                                importlib.metadata.version("pytorch-mppi"))

    @property
    def cuda_memory_allocated_mb(self) -> float:
        if self.device.type != "cuda":
            return 0.0
        return torch.cuda.memory_allocated(self.device) / (1024.0 * 1024.0)

    def _dynamics(self, state: torch.Tensor, action: torch.Tensor, step: int) -> torch.Tensor:
        return state + self.config.dt * action

    def _collision_penalty(self, q: torch.Tensor) -> torch.Tensor:
        _, origins = torch_forward_kinematics(q, return_origins=True)
        radii = torch.tensor([0.045, 0.040, 0.035, 0.033, 0.030, 0.027, 0.025],
                             device=q.device, dtype=q.dtype)
        pairs = ((0, 3), (0, 4), (0, 5), (0, 6), (1, 4), (1, 5), (1, 6),
                 (2, 5), (2, 6), (3, 6))
        total = torch.zeros(q.shape[:-1], device=q.device, dtype=q.dtype)
        for first, second in pairs:
            distance = torch.linalg.vector_norm(origins[..., first, :] - origins[..., second, :], dim=-1)
            penetration = torch.relu(radii[first] + radii[second] + self.config.collision_clearance - distance)
            total = total + penetration.square()
        return total

    def _running_cost(self, state: torch.Tensor, action: torch.Tensor, step: int) -> torch.Tensor:
        error_sq = torch.sum((torch_ee_position(state) - self.goal) ** 2, dim=-1)
        control = self.config.w_control * torch.sum(action.square(), dim=-1)
        safe_min = self.q_min + self.config.joint_margin
        safe_max = self.q_max - self.config.joint_margin
        limits = torch.relu(safe_min - state).square() + torch.relu(state - safe_max).square()
        distance = torch.sqrt(error_sq)
        stop_gate = torch.exp(-((distance / self.config.slowdown_radius) ** 2))
        stop = self.config.w_stop * stop_gate * torch.sum(action.square(), dim=-1)
        smooth = self.config.w_smooth * torch.sum((action - self.previous_action).square(), dim=-1) if step == 0 else 0.0
        return (self.config.w_position * error_sq + control + smooth +
                self.config.w_joint_limit * torch.sum(limits, dim=-1) +
                self.config.w_collision * self._collision_penalty(state) + stop)

    def _terminal_cost(self, states: torch.Tensor, actions: torch.Tensor) -> torch.Tensor:
        final_state = states[..., -1, :]
        error = torch_ee_position(final_state) - self.goal
        return self.config.w_terminal * torch.sum(error.square(), dim=-1)

    def command(self, q_current: ArrayLike, p_goal_m: ArrayLike) -> NDArray[np.float64]:
        q = np.asarray(q_current, dtype=float)
        goal = np.asarray(p_goal_m, dtype=float)
        if q.shape != (6,) or goal.shape != (3,):
            raise ValueError("q_current and p_goal_m must have shapes (6,) and (3,)")
        command_started = time.perf_counter()
        self._current_q_cpu[:] = q
        goal_changed = not np.array_equal(goal, self._goal_cpu)
        if goal_changed:
            self._goal_cpu[:] = goal

        with torch.inference_mode():
            q_tensor = torch.as_tensor(q, device=self.device, dtype=self.dtype)
            if goal_changed:
                self.goal.copy_(torch.as_tensor(goal, device=self.device, dtype=self.dtype))
            if self.device.type == "cuda":
                torch.compiler.cudagraph_mark_step_begin()
                torch.cuda.synchronize(self.device)
            solve_started = time.perf_counter()
            self.last_preprocess_time_sec = solve_started - command_started

            raw = self._optimizer.command(q_tensor)
            if self.device.type == "cuda":
                torch.cuda.synchronize(self.device)
            solve_finished = time.perf_counter()
            self.last_solve_time_sec = solve_finished - solve_started

            # Keep control scaling and diagnostics on the GPU. Copy the six-axis
            # command and all three diagnostic scalars to CPU in one transfer,
            # which replaces five separate synchronization points.
            distance = torch.linalg.vector_norm(torch_ee_position(q_tensor) - self.goal)
            scaled = raw * torch.clamp(distance / self.config.slowdown_radius, max=1.0)
            output = torch.where(distance <= self.config.goal_tolerance, torch.zeros_like(raw), scaled)
            output = torch.clamp(output, self.u_min, self.u_max)

            costs = self._optimizer.cost_total
            weights = self._optimizer.omega
            terminal_states = self._optimizer.states[0, :, -1, :]
            terminal_errors = torch.linalg.vector_norm(torch_ee_position(terminal_states) - self.goal, dim=-1)
            best = torch.argmin(costs)
            diagnostics = torch.stack((
                torch.min(costs),
                1.0 / torch.sum(weights.square()),
                terminal_errors[best],
            ))
            self.previous_action.copy_(output)
            host_values = torch.cat((output, diagnostics)).cpu().numpy().astype(float, copy=False)
            result = host_values[:6].copy()
            self.last_diagnostics = MPPIDiagnostics(
                minimum_cost=float(host_values[6]),
                effective_sample_size=float(host_values[7]),
                predicted_terminal_error_m=float(host_values[8]),
            )

        self.last_postprocess_time_sec = time.perf_counter() - solve_finished
        self.last_command_time_sec = time.perf_counter() - command_started
        return result

    def warm_up(self, q_current: ArrayLike, p_goal_m: ArrayLike) -> float:
        """Compile/fill CUDA caches without consuming the nominal trajectory or RNG."""
        cpu_rng = torch.get_rng_state()
        cuda_rng = torch.cuda.get_rng_state(self.device) if self.device.type == "cuda" else None
        original_u = self._optimizer.U.clone()
        original_previous = self.previous_action.clone()
        started = time.perf_counter()
        self.command(q_current, p_goal_m)
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)
        elapsed = time.perf_counter() - started
        with torch.inference_mode():
            self._optimizer.U.copy_(original_u)
            self.previous_action.copy_(original_previous)
        torch.set_rng_state(cpu_rng)
        if cuda_rng is not None:
            torch.cuda.set_rng_state(cuda_rng, self.device)
        return elapsed

    def reset(self) -> None:
        self._optimizer.U.zero_()
        self.previous_action.zero_()
        self.last_diagnostics = None
        self.last_solve_time_sec = 0.0
        self.last_command_time_sec = 0.0
        self.last_preprocess_time_sec = 0.0
        self.last_postprocess_time_sec = 0.0
