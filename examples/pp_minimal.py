import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from el05 import EL05


# 最小 PP 示例：切 PP -> 使能 -> 发目标位置。
with EL05(port="COM6", motor_id=1) as motor:
    motor.set_pp_mode()
    time.sleep(0.05)
    motor.enable()
    time.sleep(0.05)
    motor.set_target_deg(10)
    time.sleep(3.0)
