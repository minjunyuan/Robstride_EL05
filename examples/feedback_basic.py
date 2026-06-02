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

    for _ in range(20):
        bus.update_feedback(0.05)
        info = bus.feedback_summary(MOTOR_ID, max_age=0.5)
        if info is None:
            print("no recent feedback")
        else:
            print(
                f"pos={info['position_deg']:.2f} deg, "
                f"vel={info['velocity_rad_s']:.3f} rad/s, "
                f"torque={info['torque_nm']:.3f} Nm, "
                f"temp={info['temperature_c']:.1f} C, "
                f"fault=0x{info['fault_bits']:02x}, "
                f"state={info['mode_state']}"
            )
        time.sleep(0.1)
