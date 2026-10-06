from pathlib import Path
import sys

PROJECT_ROOT = Path("/home/arclab/workspace/urop2_manipulator/mycobot")
sys.path.insert(0, str(PROJECT_ROOT))

import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.experimental.utils.app as app_utils
import omni.timeline

from runtime.control_session import invalidate_robot, stop_controller
from runtime.isaac_robot import ROBOT_CONTAINER_PATH

scene_path = PROJECT_ROOT / "scenes" / "mycobot_default.usd"
if not scene_path.is_file():
    raise FileNotFoundError(scene_path)

await stop_controller()
invalidate_robot()
timeline = omni.timeline.get_timeline_interface()
if timeline.is_playing():
    timeline.stop()
    await app_utils.update_app_async(steps=2)
opened, stage = await stage_utils.open_stage_async(str(scene_path))
if not opened:
    raise RuntimeError(f"failed to open stage: {scene_path}")
await app_utils.update_app_async(steps=2)

required = ("/World/PhysicsScene", "/World/GroundPlane", "/World/Light", ROBOT_CONTAINER_PATH)
missing = [path for path in required if not stage.GetPrimAtPath(path).IsValid()]
if missing:
    raise RuntimeError(f"default scene is missing required prims: {missing}")
print(f"opened: {scene_path}")
print("required prims:", ", ".join(required))
