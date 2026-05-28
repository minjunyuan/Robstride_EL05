import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from el05 import EL05


# 低风险检查：清故障、读版本、停止。
# 不使能，不发目标位置。
with EL05(port="COM6", motor_id=1) as motor:
    motor.stop(clear_fault=True)
    time.sleep(0.1)
    print("clear fault / stop sent")

    motor.read_version()
    replies = motor.receive_frames(0.5)
    print("read version replies:")
    for can_id, data8 in replies:
        print(f"can_id=0x{can_id:08x}, data={data8.hex(' ')}")

    motor.stop()
    time.sleep(0.1)
    print("stop sent")
