import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from el05 import EL05Bus


# 电流模式示例。
# 风险高：没有位置目标保护。只用很小电流短时间测试。
MOTOR_ID = 1

with EL05Bus(port="COM7") as bus:
    bus.stop(MOTOR_ID)
    time.sleep(0.08)
    bus.set_current_mode(MOTOR_ID)
    time.sleep(0.05)
    bus.enable(MOTOR_ID)
    time.sleep(0.05)
    bus.set_current_ref(MOTOR_ID, 0.2)

    time.sleep(0.2)

    bus.set_current_ref(MOTOR_ID, 0.0)
    time.sleep(0.1)
    bus.stop(MOTOR_ID)
