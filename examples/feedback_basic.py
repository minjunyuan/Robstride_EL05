import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from el05 import EL05Bus


# 只读取反馈，不使能，不发目标位置。
MOTOR_ID = 1

with EL05Bus(port="COM7") as bus:
    bus.flush_rx()
    bus.stop(MOTOR_ID)
    time.sleep(0.05)
    bus.set_feedback_active(MOTOR_ID, True)

    for _ in range(20):
        bus.receive_feedback(0.05)
        fb = bus.get_feedback(MOTOR_ID, max_age=0.5)
        if fb is None:
            print("no recent feedback")
        else:
            print(
                f"pos={fb.position_deg:.2f} deg, "
                f"vel={fb.velocity_rad_s:.3f} rad/s, "
                f"torque={fb.torque_nm:.3f} Nm, "
                f"temp={fb.temperature_c:.1f} C, "
                f"fault=0x{fb.fault_bits:02x}, "
                f"state={fb.mode_state}"
            )
        time.sleep(0.1)
