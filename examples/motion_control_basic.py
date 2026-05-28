import math
import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from el05 import EL05


# 运控模式基础示例。先确认 PP 模式稳定后再测试。
with EL05(port="COM6", motor_id=1) as motor:
    motor.stop()
    time.sleep(0.08)
    motor.configure_motion()
    motor.require_no_fault(timeout=0.5)

    target = 0 * math.pi / 180
    for _ in range(200):
        motor.receive_feedback(0.001)
        motor.motion_control_safe(
            pos_rad=target,
            vel_rad_s=0.0,
            kp=0.8,
            kd=0.5,
            torque_nm=0.0,
        )
        time.sleep(0.01)

    motor.stop()
