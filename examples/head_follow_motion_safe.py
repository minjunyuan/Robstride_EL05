import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from el05 import EL05


# 运控模式实时跟随结构示例。
# 把 face_target_deg() 替换成视觉/交互算法输出，即可接入真实目标。

PORT = "COM6"
MOTOR_ID = 1

RATE_HZ = 100
DT = 1.0 / RATE_HZ

KP = 3.0
KD = 0.5
MAX_VEL = 0.5
MAX_ACCEL = 1.0
MIN_DEG = -45
MAX_DEG = 45


def face_target_deg(t):
    return 20.0 * math.sin(0.5 * t)


def clamp(x, lo, hi):
    return max(min(x, hi), lo)


with EL05(port=PORT, motor_id=MOTOR_ID) as motor:
    motor.stop()
    time.sleep(0.08)
    motor.configure_motion()
    motor.require_no_fault(timeout=0.5)

    pos_cmd = motor.last_feedback.position_rad
    vel_cmd = 0.0
    start = time.time()

    try:
        while time.time() - start < 10.0:
            loop_t = time.time()
            motor.receive_feedback(0.001)

            target_deg = clamp(face_target_deg(loop_t - start), MIN_DEG, MAX_DEG)
            target_rad = target_deg * math.pi / 180

            err = target_rad - pos_cmd
            vel_des = clamp(err * 3.0, -MAX_VEL, MAX_VEL)
            dv = clamp(vel_des - vel_cmd, -MAX_ACCEL * DT, MAX_ACCEL * DT)
            vel_cmd += dv
            pos_cmd += vel_cmd * DT

            motor.motion_control_safe(
                pos_rad=pos_cmd,
                vel_rad_s=vel_cmd,
                kp=KP,
                kd=KD,
                torque_nm=0.0,
                min_pos_rad=MIN_DEG * math.pi / 180,
                max_pos_rad=MAX_DEG * math.pi / 180,
                max_abs_vel=MAX_VEL,
                max_kp=10.0,
                max_kd=2.0,
                max_abs_torque=0.5,
                feedback_timeout=0.15,
            )

            elapsed = time.time() - loop_t
            time.sleep(max(0.0, DT - elapsed))
    finally:
        motor.stop()
