"""MyCobot interface for code executed inside a running Isaac Sim application."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray


ROBOT_CONTAINER_PATH = "/World/MyCobot"
ARM_JOINT_NAMES = (
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
)
JOINT_MIN = np.array([-2.9321, -2.4434, -2.6179, -2.6179, -2.7052, -3.14159])
JOINT_MAX = np.array([2.9321, 2.4434, 2.6179, 2.6179, 2.7925, 3.14159])
MAX_VELOCITY = np.full(6, 2.0944)
PHYSICS_DT = 0.02
MPPI_POSITION_STIFFNESS = np.array([120.0, 120.0, 100.0, 80.0, 50.0, 30.0])
MPPI_POSITION_DAMPING = np.array([4.0, 4.0, 3.5, 3.0, 2.0, 1.5])


def _vector(value: ArrayLike, name: str) -> NDArray[np.float64]:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (6,):
        raise ValueError(f"{name} must have shape (6,), got {vector.shape}")
    return vector


def _as_numpy(value) -> NDArray[np.float64]:
    return value.numpy()[0].astype(float)


@dataclass(frozen=True)
class JointState:
    q: NDArray[np.float64]
    dq: NDArray[np.float64]


@dataclass(frozen=True)
class DriveState:
    mode: str
    stiffness: NDArray[np.float64]
    damping: NDArray[np.float64]


class IsaacMyCobot:
    """Target-based interface which never creates or closes SimulationApp."""

    @classmethod
    async def create(cls) -> "IsaacMyCobot":
        import omni.timeline
        import isaacsim.core.experimental.utils.stage as stage_utils
        import isaacsim.core.experimental.utils.app as app_utils
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.simulation_manager import SimulationManager

        stage = stage_utils.get_current_stage(backend="usd")
        if stage is None or not stage.GetPrimAtPath(ROBOT_CONTAINER_PATH).IsValid():
            raise RuntimeError("open scenes/mycobot_default.usd before controlling MyCobot")

        roots = [
            path
            for path in Articulation.fetch_articulation_root_api_prim_paths(ROBOT_CONTAINER_PATH)
            if path is not None
        ]
        if len(roots) != 1:
            raise RuntimeError(f"expected one articulation under {ROBOT_CONTAINER_PATH}, found {roots}")

        articulation = Articulation(roots[0])
        all_names = list(articulation.dof_names)
        try:
            arm_indices = np.array([all_names.index(name) for name in ARM_JOINT_NAMES], dtype=int)
        except ValueError as exc:
            raise RuntimeError(f"MyCobot arm DOF mismatch; stage contains {all_names}") from exc

        timeline = omni.timeline.get_timeline_interface()
        current_dt = SimulationManager.get_physics_dt()
        if current_dt is None or not np.isclose(current_dt, PHYSICS_DT):
            if timeline.is_playing():
                timeline.stop()
                await app_utils.update_app_async(steps=1)
            SimulationManager.set_physics_dt(PHYSICS_DT)

        if not timeline.is_playing():
            timeline.play()
        await app_utils.update_app_async(steps=2)
        if not articulation.is_physics_tensor_entity_valid():
            raise RuntimeError("articulation physics tensor did not initialize after timeline play")
        return cls(articulation, arm_indices)

    def __init__(self, articulation, arm_dof_indices: NDArray[np.int64]) -> None:
        self.articulation = articulation
        self.arm_dof_indices = arm_dof_indices

    @property
    def joint_names(self) -> tuple[str, ...]:
        return ARM_JOINT_NAMES

    def read_state(self) -> JointState:
        q = _as_numpy(self.articulation.get_dof_positions(dof_indices=self.arm_dof_indices))
        dq = _as_numpy(self.articulation.get_dof_velocities(dof_indices=self.arm_dof_indices))
        return JointState(q=q, dq=dq)

    def configure_position_drive(
        self,
        stiffness: ArrayLike | None = None,
        damping: ArrayLike | None = None,
    ) -> DriveState:
        if stiffness is not None or damping is not None:
            stiffness_value = None if stiffness is None else _vector(stiffness, "stiffness")[None, :]
            damping_value = None if damping is None else _vector(damping, "damping")[None, :]
            self.articulation.set_dof_gains(
                stiffnesses=stiffness_value,
                dampings=damping_value,
                dof_indices=self.arm_dof_indices,
            )
        self.articulation.switch_dof_control_mode("position", dof_indices=self.arm_dof_indices)
        return self.verify_drive("position")

    def configure_velocity_drive(self) -> DriveState:
        self.articulation.switch_dof_control_mode("velocity", dof_indices=self.arm_dof_indices)
        return self.verify_drive("velocity")

    def verify_drive(self, mode: str) -> DriveState:
        stiffness, damping = self.articulation.get_dof_gains(dof_indices=self.arm_dof_indices)
        stiffness_np = _as_numpy(stiffness)
        damping_np = _as_numpy(damping)
        if np.any(damping_np <= 0.0):
            raise RuntimeError(f"all arm drives require positive damping, got {damping_np}")
        if mode == "position" and np.any(stiffness_np <= 0.0):
            raise RuntimeError(f"position control requires positive stiffness, got {stiffness_np}")
        if mode == "velocity" and not np.allclose(stiffness_np, 0.0, atol=1e-6):
            raise RuntimeError(f"velocity control requires zero stiffness, got {stiffness_np}")
        return DriveState(mode=mode, stiffness=stiffness_np, damping=damping_np)

    def set_position_target(self, q_target: ArrayLike) -> NDArray[np.float64]:
        command = np.clip(_vector(q_target, "q_target"), JOINT_MIN, JOINT_MAX)
        self.articulation.set_dof_position_targets(command[None, :], dof_indices=self.arm_dof_indices)
        self.articulation.set_dof_velocity_targets(np.zeros((1, 6)), dof_indices=self.arm_dof_indices)
        return command.copy()

    def set_velocity_target(self, dq_target: ArrayLike) -> NDArray[np.float64]:
        command = np.clip(_vector(dq_target, "dq_target"), -MAX_VELOCITY, MAX_VELOCITY)
        self.articulation.set_dof_velocity_targets(command[None, :], dof_indices=self.arm_dof_indices)
        return command.copy()

    def zero_velocity_target(self) -> None:
        self.articulation.set_dof_velocity_targets(np.zeros((1, 6)), dof_indices=self.arm_dof_indices)
