import math
import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from el05 import EL05Bus


# 运控模式基础示例。先确认 PP 模式稳定后再测试。
MOTOR_ID = 1
TARGET_DEG = 180.0
CONTROL_HZ = 100
CONTROL_TIME = 2.0

with EL05Bus(port="COM7") as bus:
    bus.stop(MOTOR_ID)
    time.sleep(0.08)
    bus.set_motion_mode(MOTOR_ID)
    bus.enable(MOTOR_ID)
    time.sleep(0.05)
    bus.set_feedback_active(MOTOR_ID, True)
    fb = bus.wait_feedback(MOTOR_ID, timeout=0.5)
    if fb.fault_bits:
        raise RuntimeError(f"motor {MOTOR_ID} fault bits: 0x{fb.fault_bits:02x}")

    target_rad = math.radians(TARGET_DEG)
    steps = int(CONTROL_TIME * CONTROL_HZ)

    for _ in range(steps):
        bus.receive_feedback(0.001)
        bus.motion_control(
            MOTOR_ID,
            pos_rad=target_rad,
            vel_rad_s=0.0,
            kp=2.0,
            kd=1.8,
            torque_nm=0.0,
        )
        time.sleep(1 / CONTROL_HZ)

    bus.stop(MOTOR_ID)
