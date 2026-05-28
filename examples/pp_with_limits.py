import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from el05 import EL05


# PP 示例：显式设置速度和加速度。
with EL05(port="COM6", motor_id=1) as motor:
    motor.stop()
    time.sleep(0.08)
    motor.configure_pp(speed=2.0, acc=2.0)
    motor.set_target_deg(10)
    time.sleep(3.0)
    motor.stop()
