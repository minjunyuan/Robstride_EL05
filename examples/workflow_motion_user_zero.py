import math
import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from el05 import EL05Bus


MOTOR_ID = 2
PORT = "COM7"

KP = 2.0
KD = 1.8
TARGET_USER_DEG = 0.0


with EL05Bus(port=PORT, timeout=0.005) as bus:
    try:
        bus.stop(MOTOR_ID)
        bus.configure_motion(MOTOR_ID, active_report=True)
        bus.wait_feedback(MOTOR_ID, timeout=3.0)
        bus.set_nearest_zero_as_user_zero(MOTOR_ID)

        target = bus.user_to_raw_rad(MOTOR_ID, math.radians(TARGET_USER_DEG))

        for _ in range(200):
            bus.receive_feedback(0.001)
            bus.motion_control(MOTOR_ID, pos_rad=target, kp=KP, kd=KD)

            user_deg = bus.get_user_position_deg(MOTOR_ID, max_age=0.2)
            print(f"user={user_deg:.2f} deg")

            time.sleep(0.01)

    finally:
        bus.stop(MOTOR_ID)
