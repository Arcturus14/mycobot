from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path("/home/arclab/workspace/urop2_manipulator/mycobot")
sys.path.insert(0, str(PROJECT_ROOT))

from runtime.control_session import resolve_control_steps, start_position_control
from runtime.isaac_robot import PHYSICS_DT

try:
    q_target_deg
except NameError:
    q_target_deg = [20, 15, 10, -15, -10, 20]
try:
    steps
except NameError:
    steps = None
try:
    duration_sec
except NameError:
    duration_sec = None
try:
    print_every
except NameError:
    print_every = 10
steps = resolve_control_steps(steps=steps, duration_sec=duration_sec, default_steps=100)
print_every = int(print_every)
print(f"resolved duration: {steps} steps x {PHYSICS_DT} s = {steps * PHYSICS_DT:.3f} s")
result = await start_position_control(np.deg2rad(q_target_deg), steps, print_every)
print(result)
