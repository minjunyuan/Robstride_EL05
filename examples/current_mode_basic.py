import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from el05 import EL05


# 电流模式示例。
# 风险高：没有位置目标保护。只用很小电流短时间测试。
with EL05(port="COM6", motor_id=1) as motor:
    motor.stop()
    time.sleep(0.08)

    motor.configure_current()
    motor.run_current(amp=0.2, configure=False)

    time.sleep(0.2)

    motor.set_current_ref(0.0)
    time.sleep(0.1)
    motor.stop()
