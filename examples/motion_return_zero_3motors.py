import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from el05 import EL05Bus


MOTOR_IDS = [1,2,3]
PORT = "COM7"

KP = 5.0
KD = 4.0
CONTROL_HZ = 100
DONE_DEG = 1.0


with EL05Bus(port=PORT, timeout=0.005) as bus:
    try:
        bus.flush_rx()

        for motor_id in MOTOR_IDS:
            bus.configure_motion(motor_id, active_report=True)

        bus.wait_feedback(MOTOR_IDS, timeout=3.0)

        for motor_id in MOTOR_IDS:
            bus.set_nearest_zero_as_user_zero(motor_id)

        while True:
            bus.receive_feedback(0.001)

            done = True
            for motor_id in MOTOR_IDS:
                bus.motion_control(
                    motor_id,
                    pos_rad=bus.user_to_raw_rad(motor_id, 0.0),
                    vel_rad_s=0.0,
                    kp=KP,
                    kd=KD,
                    torque_nm=0.0,
                )

                pos_deg = bus.get_position_deg(motor_id)
                user_deg = bus.get_user_position_deg(motor_id)
                info = bus.feedback_summary(motor_id)
                if pos_deg is None or user_deg is None or info is None:
                    done = False
                    continue
                fixed_deg = info["position_deg_fixed"]
                print(f"{motor_id}: raw={pos_deg:.2f} deg, user={user_deg:.2f} deg, fixed={fixed_deg:.2f} deg")

                if abs(user_deg) > DONE_DEG:
                    done = False

            if done:
                break

            time.sleep(1 / CONTROL_HZ)

    finally:
        for motor_id in MOTOR_IDS:
            bus.stop(motor_id)
