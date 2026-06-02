import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from el05 import EL05Bus


# PP 位置模式：设置速度/加速度限制，然后发送目标角度。
MOTOR_ID = 1
TARGET_DEG = 10.0
SPEED_RAD_S = 1.0
ACC_RAD_S2 = 1.0


with EL05Bus(port="COM7") as bus:
    bus.stop(MOTOR_ID)
    time.sleep(0.05)

    bus.configure_pp(MOTOR_ID, speed=SPEED_RAD_S, acc=ACC_RAD_S2)
    bus.set_target_deg(MOTOR_ID, TARGET_DEG)

    time.sleep(2.0)
    bus.stop(MOTOR_ID)
