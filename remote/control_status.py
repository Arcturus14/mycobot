from pathlib import Path
import json
import sys

PROJECT_ROOT = Path("/home/arclab/workspace/urop2_manipulator/mycobot")
sys.path.insert(0, str(PROJECT_ROOT))

from runtime.control_session import status

print(json.dumps(status(), indent=2, ensure_ascii=False))
