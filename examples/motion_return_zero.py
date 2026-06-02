import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from angle_control import return_motors_to_zero
from el05 import EL05Bus


# 按实际电机数量增删 ID；1 个、3 个或更多电机都用同一个入口。
MOTOR_IDS = [1, 2, 3]
PORT = "COM7"

# 每个电机允许的启动单圈角度窗口。先按机械结构填安全范围。
ALLOWED_ANGLE_DEG = {
    1: (-90.0, 90.0),
    2: (-180.0, 180.0),
    3: (-180.0, 180.0),
}


with EL05Bus(port=PORT, timeout=0.005) as bus:
    try:
        return_motors_to_zero(
            bus,
            MOTOR_IDS,
            ALLOWED_ANGLE_DEG,
            kp=2.0,
            kd=1.8,
            done_deg=1.0,
            max_target_move_deg=200.0,
            max_feedback_jump_deg=30.0,
        )

        # 后续动作写在这里；此时电机仍保持在 motion mode 原位。

    finally:
        for motor_id in MOTOR_IDS:
            bus.stop(motor_id)
