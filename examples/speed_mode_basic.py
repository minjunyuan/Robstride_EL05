import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from el05 import EL05


# 速度模式示例。
# 适合连续旋转机构；有限角度关节上谨慎使用，必须确认不会撞限位。
with EL05(port="COM6", motor_id=1) as motor:
    motor.stop()
    time.sleep(0.08)

    motor.configure_speed(limit_current=2.0, acc=1.0)
    motor.run_speed(rad_s=0.5, configure=False)

    time.sleep(2.0)

    motor.set_speed_ref(0.0)
    time.sleep(0.2)
    motor.stop()
