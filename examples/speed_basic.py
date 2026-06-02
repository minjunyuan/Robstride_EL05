import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from el05 import EL05Bus


# 速度模式示例。
# 适合连续旋转机构；有限角度关节上谨慎使用，必须确认不会撞限位。
MOTOR_ID = 1

with EL05Bus(port="COM7") as bus:
    bus.stop(MOTOR_ID)
    time.sleep(0.08)

    bus.configure_speed(MOTOR_ID, limit_current=2.0, acc=1.0)
    bus.run_speed(MOTOR_ID, rad_s=0.5, configure=False)

    time.sleep(2.0)

    bus.set_speed_ref(MOTOR_ID, 0.0)
    time.sleep(0.2)
    bus.stop(MOTOR_ID)
