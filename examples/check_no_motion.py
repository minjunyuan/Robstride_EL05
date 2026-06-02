import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from el05 import EL05Bus


# 低风险检查：清故障、读版本、停止。
# 不使能，不发目标位置。
MOTOR_ID = 1

with EL05Bus(port="COM7") as bus:
    bus.stop(MOTOR_ID, clear_fault=True)
    time.sleep(0.1)
    print("clear fault / stop sent")

    bus.read_version(MOTOR_ID)
    replies = bus.receive_frames(0.5)
    print("read version replies:")
    for can_id, data8 in replies:
        print(f"can_id=0x{can_id:08x}, data={data8.hex(' ')}")

    bus.stop(MOTOR_ID)
    time.sleep(0.1)
    print("stop sent")
