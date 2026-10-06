from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path("/home/arclab/workspace/urop2_manipulator/mycobot")
sys.path.insert(0, str(PROJECT_ROOT))

from runtime.control_session import get_robot

robot = await get_robot()
state = robot.read_state()
print("joint names:", robot.joint_names)
print("q  [deg]:", np.rad2deg(state.q).round(4).tolist())
print("dq [deg/s]:", np.rad2deg(state.dq).round(4).tolist())
