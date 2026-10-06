from pathlib import Path
import json
import sys

PROJECT_ROOT = Path("/home/arclab/workspace/urop2_manipulator/mycobot")
sys.path.insert(0, str(PROJECT_ROOT))

from runtime.control_session import stop_controller

print(json.dumps(await stop_controller(), indent=2, ensure_ascii=False))
