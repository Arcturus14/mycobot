"""Isaac Sim boundary for the standalone MyCobot examples."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import ArrayLike, NDArray


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE_PATH = PROJECT_ROOT / "scenes" / "mycobot_default.usd"
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


def _vector(value: ArrayLike, name: str) -> NDArray[np.float64]:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (6,):
        raise ValueError(f"{name} must have shape (6,), got {vector.shape}")
    return vector


def _as_numpy(value) -> NDArray[np.float64]:
    return value.numpy()[0].astype(float)


def open_default_scene(simulation_app, scene_path: Path = DEFAULT_SCENE_PATH) -> None:
    """Open the saved scene; no environment prim is created in Python."""
    import isaacsim.core.experimental.utils.stage as stage_utils

    resolved = scene_path.resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"scene does not exist: {resolved}")
    opened, _ = stage_utils.open_stage(str(resolved))
    if not opened:
        raise RuntimeError(f"failed to open stage: {resolved}")
    while stage_utils.is_stage_loading():
        simulation_app.update()
    simulation_app.update()

    stage = stage_utils.get_current_stage(backend="usd")
    required = ("/World/PhysicsScene", "/World/GroundPlane", "/World/Light", ROBOT_CONTAINER_PATH)
    missing = [path for path in required if not stage.GetPrimAtPath(path).IsValid()]
    if missing:
        raise RuntimeError(f"default scene is missing required prims: {missing}")


@dataclass(frozen=True)
class DriveState:
    mode: str
    stiffness: NDArray[np.float64]
    damping: NDArray[np.float64]


class IsaacMyCobot:
    """Six-arm-joint target interface around Isaac Sim Articulation."""

    def __init__(self, simulation_app) -> None:
        import omni.timeline
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.simulation_manager import SimulationManager

        roots = [
            path
            for path in Articulation.fetch_articulation_root_api_prim_paths(ROBOT_CONTAINER_PATH)
            if path is not None
        ]
        if len(roots) != 1:
            raise RuntimeError(f"expected one articulation under {ROBOT_CONTAINER_PATH}, found {roots}")

        self.simulation_app = simulation_app
        self.timeline = omni.timeline.get_timeline_interface()
        self.articulation_root_path = roots[0]
        self.articulation = Articulation(self.articulation_root_path)
        all_names = list(self.articulation.dof_names)
        try:
            self.arm_dof_indices = np.array([all_names.index(name) for name in ARM_JOINT_NAMES], dtype=int)
        except ValueError as exc:
            raise RuntimeError(f"MyCobot arm DOF mismatch; stage contains {all_names}") from exc

        SimulationManager.set_physics_dt(PHYSICS_DT)
        self.timeline.play()
        self.simulation_app.update()
        self.simulation_app.update()
        if not self.articulation.is_physics_tensor_entity_valid():
            raise RuntimeError("articulation physics tensor was not initialized after starting the timeline")

    @property
    def joint_names(self) -> tuple[str, ...]:
        return ARM_JOINT_NAMES

    def read_state(self):
        from controllers import JointState

        q = _as_numpy(self.articulation.get_dof_positions(dof_indices=self.arm_dof_indices))
        dq = _as_numpy(self.articulation.get_dof_velocities(dof_indices=self.arm_dof_indices))
        return JointState(q=q, dq=dq)

    def configure_position_drive(self) -> DriveState:
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

    def step(self, steps: int = 1) -> None:
        for _ in range(steps):
            self.simulation_app.update()

    def stop(self) -> None:
        if self.timeline.is_playing():
            self.timeline.stop()
