import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from el05 import EL05Bus


# CSP 位置模式示例。
# 上位机周期性发送位置点；这里发送小幅正弦轨迹。
MOTOR_ID = 1

with EL05Bus(port="COM6") as bus:
    bus.stop(MOTOR_ID)
    time.sleep(0.08)
    bus.set_csp_mode(MOTOR_ID)
    time.sleep(0.05)
    bus.enable(MOTOR_ID)
    time.sleep(0.05)
    bus.set_limit_speed(MOTOR_ID, 1.0)
    time.sleep(0.05)

    for i in range(200):
        t = i * 0.01
        target_deg = 10.0 * math.sin(2 * math.pi * 0.2 * t)
        bus.set_target_deg(MOTOR_ID, target_deg)
        time.sleep(0.01)

    bus.stop(MOTOR_ID)
