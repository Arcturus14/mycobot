"""Visual-only Cartesian goal marker for the running Isaac Sim stage."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike


GOAL_MARKER_PATH = "/World/Markers/MppiGoal"
ROBOT_BASE_PATH = "/World/MyCobot"


def set_goal_marker(p_goal_base_m: ArrayLike) -> str:
    """Move the red sphere to a goal expressed in the MyCobot base frame."""
    import isaacsim.core.experimental.utils.stage as stage_utils
    from pxr import Gf, Usd, UsdGeom

    goal = np.asarray(p_goal_base_m, dtype=float)
    if goal.shape != (3,):
        raise ValueError(f"p_goal_base_m must have shape (3,), got {goal.shape}")

    stage = stage_utils.get_current_stage(backend="usd")
    if stage is None:
        raise RuntimeError("no USD stage is currently open")

    base_prim = stage.GetPrimAtPath(ROBOT_BASE_PATH)
    marker_prim = stage.GetPrimAtPath(GOAL_MARKER_PATH)
    if not base_prim.IsValid():
        raise RuntimeError(f"robot base prim is missing: {ROBOT_BASE_PATH}")
    if not marker_prim.IsValid():
        raise RuntimeError(
            f"goal marker is missing: {GOAL_MARKER_PATH}; reopen scenes/mycobot_default.usd"
        )

    base_to_world = UsdGeom.Xformable(base_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    goal_world = base_to_world.Transform(Gf.Vec3d(*goal.tolist()))
    translate = marker_prim.GetAttribute("xformOp:translate")
    if not translate.IsValid():
        raise RuntimeError(f"goal marker has no xformOp:translate: {GOAL_MARKER_PATH}")
    translate.Set(goal_world)
    UsdGeom.Imageable(marker_prim).MakeVisible()
    return GOAL_MARKER_PATH


def hide_goal_marker() -> bool:
    """Hide the marker when MPPI is idle; tolerate a stage reload during cleanup."""
    import isaacsim.core.experimental.utils.stage as stage_utils
    from pxr import UsdGeom

    stage = stage_utils.get_current_stage(backend="usd")
    if stage is None:
        return False
    marker_prim = stage.GetPrimAtPath(GOAL_MARKER_PATH)
    if not marker_prim.IsValid():
        return False
    UsdGeom.Imageable(marker_prim).MakeInvisible()
    return True
