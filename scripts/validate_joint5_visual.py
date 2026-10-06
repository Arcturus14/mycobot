"""Validate the composed joint 5 visual in the default MyCobot scene."""

from pathlib import Path

from isaacsim import SimulationApp


APP = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdShade


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = PROJECT_ROOT / "scenes" / "mycobot_default.usd"
JOINT5_VISUAL_PATH = (
    "/World/MyCobot/Geometry/joint1/joint2/joint3/joint4/joint5/joint5"
)


try:
    stage = Usd.Stage.Open(str(SCENE_PATH))
    if stage is None:
        raise RuntimeError(f"failed to open stage: {SCENE_PATH}")

    visual = stage.GetPrimAtPath(JOINT5_VISUAL_PATH)
    if not visual.IsValid():
        raise RuntimeError(f"missing joint 5 visual prim: {JOINT5_VISUAL_PATH}")

    meshes = [prim for prim in Usd.PrimRange(visual) if prim.IsA(UsdGeom.Mesh)]
    if not meshes:
        raise RuntimeError(f"joint 5 visual contains no mesh: {JOINT5_VISUAL_PATH}")

    translation = visual.GetAttribute("xformOp:translate").Get()
    expected_translation = (0.0, 0.0, -0.03356)
    if translation is None or any(
        abs(float(actual) - expected) > 1e-6
        for actual, expected in zip(translation, expected_translation)
    ):
        raise RuntimeError(
            f"joint 5 visual origin changed: expected {expected_translation}, got {translation}"
        )

    material, _ = UsdShade.MaterialBindingAPI(meshes[0]).ComputeBoundMaterial()
    expected_material = "/World/MyCobot/VisualMaterials/Joint5Material"
    if not material or str(material.GetPath()) != expected_material:
        actual = None if not material else str(material.GetPath())
        raise RuntimeError(
            f"joint 5 material mismatch: expected {expected_material}, got {actual}"
        )

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
    )
    world_range = bbox_cache.ComputeWorldBound(visual).ComputeAlignedRange()
    if world_range.IsEmpty():
        raise RuntimeError("joint 5 visual has an empty world-space bound")

    size = world_range.GetSize()
    print(f"scene: {SCENE_PATH}")
    print(f"joint5 visual: {visual.GetPath()}")
    print(f"mesh count: {len(meshes)}")
    print(f"world bbox size (m): ({size[0]:.6f}, {size[1]:.6f}, {size[2]:.6f})")
finally:
    APP.close()
