import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from el05 import EL05


# CSP 位置模式示例。
# 上位机周期性发送位置点；这里发送小幅正弦轨迹。
with EL05(port="COM6", motor_id=1) as motor:
    motor.stop()
    time.sleep(0.08)

    motor.configure_csp(limit_speed=1.0)

    for i in range(200):
        t = i * 0.01
        target_deg = 10.0 * math.sin(2 * math.pi * 0.2 * t)
        motor.set_csp_target_deg(target_deg)
        time.sleep(0.01)

    motor.stop()
