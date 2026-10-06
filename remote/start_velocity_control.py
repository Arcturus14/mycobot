from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path("/home/arclab/workspace/urop2_manipulator/mycobot")
sys.path.insert(0, str(PROJECT_ROOT))

from runtime.control_session import resolve_control_steps, start_velocity_control
from runtime.isaac_robot import PHYSICS_DT

try:
    dq_target_deg_s
except NameError:
    dq_target_deg_s = [10, 0, 0, 0, 0, 0]
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
    print_every = 5
steps = resolve_control_steps(steps=steps, duration_sec=duration_sec, default_steps=50)
print_every = int(print_every)
print(f"resolved duration: {steps} steps x {PHYSICS_DT} s = {steps * PHYSICS_DT:.3f} s")
result = await start_velocity_control(np.deg2rad(dq_target_deg_s), steps, print_every)
print(result)
