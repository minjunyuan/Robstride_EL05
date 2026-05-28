import math
import struct
import time
from dataclasses import dataclass

import serial


# EL05 私有协议里的运控模式量程，来自说明书 4.4 样例。
P_MIN, P_MAX = -12.57, 12.57
V_MIN, V_MAX = -50.0, 50.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -6.0, 6.0

# 0x7005 run_mode 的取值。
RUN_MODE_MOTION = 0
RUN_MODE_PP = 1
RUN_MODE_SPEED = 2
RUN_MODE_CURRENT = 3
RUN_MODE_CSP = 5

# 常用参数 index。写参数用通信类型 18，读参数用通信类型 17。
IDX_RUN_MODE = 0x7005
IDX_IQ_REF = 0x7006
IDX_SPD_REF = 0x700A
IDX_LIMIT_TORQUE = 0x700B
IDX_LOC_REF = 0x7016
IDX_LIMIT_SPD = 0x7017
IDX_LIMIT_CUR = 0x7018
IDX_VEL_MAX = 0x7024
IDX_ACC_SET = 0x7025
IDX_ACC_RAD = 0x7022


@dataclass
class Feedback:
    timestamp: float
    motor_id: int
    mode_state: int
    fault_bits: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    temperature_c: float


def float_to_uint(x, x_min, x_max, bits):
    x = max(min(x, x_max), x_min)
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(x, x_min, x_max, bits):
    span = x_max - x_min
    return x * span / ((1 << bits) - 1) + x_min


def make_can_id(comm_type, data2, motor_id):
    # EL05 真实 29 位 CAN ID：通信类型 | 数据区2 | 目标电机 ID。
    return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (motor_id & 0xFF)


def can_id_to_usb_id(can_id):
    # 当前 USB-CAN 串口封装规则，不是 EL05 说明书内容。
    return (can_id << 3) | 0x04


def build_serial_frame(can_id, data8):
    # 串口实际发送：AT + 包装后的 ID + 长度 08 + 8 字节 CAN 数据 + CRLF。
    if len(data8) != 8:
        raise ValueError("data8 must be exactly 8 bytes")
    usb_id = can_id_to_usb_id(can_id)
    return b"AT" + usb_id.to_bytes(4, "big") + b"\x08" + bytes(data8) + b"\r\n"


def parse_serial_frame(frame):
    # 把串口返回帧拆回真实 CAN ID 和 8 字节数据区。
    if len(frame) != 17:
        raise ValueError("serial frame must be 17 bytes")
    if frame[:2] != b"AT" or frame[6] != 0x08 or frame[-2:] != b"\r\n":
        raise ValueError("invalid serial frame")
    usb_id = int.from_bytes(frame[2:6], "big")
    can_id = usb_id >> 3
    data8 = frame[7:15]
    return can_id, data8


class EL05:
    def __init__(self, port="COM6", baud=921600, motor_id=1, host_id=0xFD, timeout=0.02):
        # baud 是电脑到 USB-CAN 板的串口波特率，不是 CAN 总线波特率。
        self.port = port
        self.baud = baud
        self.motor_id = motor_id
        self.host_id = host_id
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self._rx_buffer = bytearray()
        self.last_feedback = None
        time.sleep(0.2)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def flush_rx(self):
        self.ser.reset_input_buffer()
        self._rx_buffer.clear()

    def send_can(self, can_id, data8):
        frame = build_serial_frame(can_id, data8)
        self.ser.write(frame)
        return frame

    def send_private(self, comm_type, data8=None, data2=None):
        # 大多数私有协议命令的 data2 使用 host_id；运控模式会例外地放力矩映射值。
        if data8 is None:
            data8 = bytes(8)
        if data2 is None:
            data2 = self.host_id
        can_id = make_can_id(comm_type, data2, self.motor_id)
        return self.send_can(can_id, data8)

    def stop(self, clear_fault=False):
        # 通信类型 4：停止；Byte0=1 时尝试清故障。
        data = bytearray(8)
        if clear_fault:
            data[0] = 1
        return self.send_private(0x04, data)

    def enable(self):
        # 通信类型 3：使能运行。
        return self.send_private(0x03, bytes(8))

    def set_zero(self):
        # 通信类型 6：设置机械零位。谨慎使用。
        data = bytearray(8)
        data[0] = 1
        return self.send_private(0x06, data)

    def read_version(self):
        # 通信类型 4 的版本读取格式。
        return self.send_private(0x04, bytes.fromhex("00c4000000000000"))

    def write_param_u8(self, index, value):
        # 通信类型 18：Byte0~1=index，Byte4 起放参数值。
        data = bytearray(8)
        data[0:2] = struct.pack("<H", index)
        data[4] = value & 0xFF
        return self.send_private(0x12, data)

    def write_param_u16(self, index, value):
        data = bytearray(8)
        data[0:2] = struct.pack("<H", index)
        data[4:6] = struct.pack("<H", value)
        return self.send_private(0x12, data)

    def write_param_u32(self, index, value):
        data = bytearray(8)
        data[0:2] = struct.pack("<H", index)
        data[4:8] = struct.pack("<I", value)
        return self.send_private(0x12, data)

    def write_param_float(self, index, value):
        data = bytearray(8)
        data[0:2] = struct.pack("<H", index)
        data[4:8] = struct.pack("<f", float(value))
        return self.send_private(0x12, data)

    def read_param(self, index):
        # 通信类型 17：读取单个参数。发送后需要 receive_frames() 读取返回。
        data = bytearray(8)
        data[0:2] = struct.pack("<H", index)
        return self.send_private(0x11, data)

    def set_run_mode(self, mode):
        return self.write_param_u8(IDX_RUN_MODE, mode)

    def set_pp_mode(self):
        return self.set_run_mode(RUN_MODE_PP)

    def set_speed_mode(self):
        return self.set_run_mode(RUN_MODE_SPEED)

    def set_current_mode(self):
        return self.set_run_mode(RUN_MODE_CURRENT)

    def set_csp_mode(self):
        return self.set_run_mode(RUN_MODE_CSP)

    def set_motion_mode(self):
        return self.set_run_mode(RUN_MODE_MOTION)

    def set_pp_speed(self, rad_s):
        return self.write_param_float(IDX_VEL_MAX, rad_s)

    def set_pp_acc(self, rad_s2):
        return self.write_param_float(IDX_ACC_SET, rad_s2)

    def set_target_rad(self, rad):
        return self.write_param_float(IDX_LOC_REF, rad)

    def set_target_deg(self, deg):
        # 说明书中 loc_ref 单位是 rad；这里帮你做 deg -> rad。
        return self.set_target_rad(deg * math.pi / 180)

    def set_target_deg_limited(self, deg, min_deg=-45.0, max_deg=45.0):
        deg = max(min(deg, max_deg), min_deg)
        return self.set_target_deg(deg)

    def set_speed_ref(self, rad_s):
        return self.write_param_float(IDX_SPD_REF, rad_s)

    def set_speed_acc(self, rad_s2):
        return self.write_param_float(IDX_ACC_RAD, rad_s2)

    def set_limit_current(self, amp):
        return self.write_param_float(IDX_LIMIT_CUR, amp)

    def set_limit_torque(self, nm):
        return self.write_param_float(IDX_LIMIT_TORQUE, nm)

    def set_limit_speed(self, rad_s):
        return self.write_param_float(IDX_LIMIT_SPD, rad_s)

    def set_current_ref(self, amp):
        return self.write_param_float(IDX_IQ_REF, amp)

    def configure_pp(self, speed=None, acc=None, stop_first=True, enable=True, delay=0.05):
        # PP 模式配置。speed/acc 可省略，省略时使用电机当前值或默认值。
        if stop_first:
            self.stop()
            time.sleep(delay)
        self.set_pp_mode()
        time.sleep(delay)
        if enable:
            self.enable()
            time.sleep(delay)
        if speed is not None:
            self.set_pp_speed(speed)
            time.sleep(delay)
        if acc is not None:
            self.set_pp_acc(acc)
            time.sleep(delay)

    def move_pp_deg(self, deg, speed=None, acc=None, configure=True, delay=0.05):
        # PP：适合“转到某个角度”。configure=True 时会先切 PP 并使能。
        if configure:
            self.configure_pp(speed=speed, acc=acc, delay=delay)
        return self.set_target_deg(deg)

    def move_pp_rad(self, rad, speed=None, acc=None, configure=True, delay=0.05):
        if configure:
            self.configure_pp(speed=speed, acc=acc, delay=delay)
        return self.set_target_rad(rad)

    def configure_speed(self, limit_current=None, acc=None, stop_first=True, enable=True, delay=0.05):
        # 速度模式配置。适合轮子/转台/连续旋转，关节结构上谨慎使用。
        if stop_first:
            self.stop()
            time.sleep(delay)
        self.set_speed_mode()
        time.sleep(delay)
        if enable:
            self.enable()
            time.sleep(delay)
        if limit_current is not None:
            self.set_limit_current(limit_current)
            time.sleep(delay)
        if acc is not None:
            self.set_speed_acc(acc)
            time.sleep(delay)

    def run_speed(self, rad_s, limit_current=None, acc=None, configure=True, delay=0.05):
        if configure:
            self.configure_speed(limit_current=limit_current, acc=acc, delay=delay)
        return self.set_speed_ref(rad_s)

    def configure_current(self, stop_first=True, enable=True, delay=0.05):
        # 电流模式风险较高：没有位置目标保护，务必先确认限位和急停。
        if stop_first:
            self.stop()
            time.sleep(delay)
        self.set_current_mode()
        time.sleep(delay)
        if enable:
            self.enable()
            time.sleep(delay)

    def run_current(self, amp, configure=True, delay=0.05):
        if configure:
            self.configure_current(delay=delay)
        return self.set_current_ref(amp)

    def configure_csp(self, limit_speed=None, stop_first=True, enable=True, delay=0.05):
        # CSP：上位机周期性发送 loc_ref，适合自己规划轨迹点。
        if stop_first:
            self.stop()
            time.sleep(delay)
        self.set_csp_mode()
        time.sleep(delay)
        if enable:
            self.enable()
            time.sleep(delay)
        if limit_speed is not None:
            self.set_limit_speed(limit_speed)
            time.sleep(delay)

    def set_csp_target_deg(self, deg):
        return self.set_target_deg(deg)

    def set_csp_target_rad(self, rad):
        return self.set_target_rad(rad)

    def configure_motion(self, stop_first=True, enable=True, delay=0.05):
        # 运控模式：适合机器人关节控制，但必须保守调 Kp/Kd/目标变化。
        if stop_first:
            self.stop()
            time.sleep(delay)
        self.set_motion_mode()
        time.sleep(delay)
        if enable:
            self.enable()
            time.sleep(delay)

    def wait_feedback(self, timeout=0.5):
        # 初始化/调试用：等待一帧本电机反馈。实时循环里不要长时间阻塞等待。
        end_time = time.time() + timeout
        while time.time() < end_time:
            feedback = self.receive_feedback(0.02)
            for item in feedback:
                if item.motor_id == self.motor_id:
                    return item
        return None

    def require_feedback(self, timeout=0.5):
        feedback = self.wait_feedback(timeout)
        if feedback is None:
            raise TimeoutError(f"no feedback from motor {self.motor_id} within {timeout}s")
        return feedback

    def require_no_fault(self, timeout=0.5):
        feedback = self.require_feedback(timeout)
        if feedback.fault_bits:
            raise RuntimeError(f"motor {self.motor_id} fault bits: 0x{feedback.fault_bits:02x}")
        return feedback

    def feedback_age(self):
        if self.last_feedback is None:
            return None
        return time.time() - self.last_feedback.timestamp

    def update_feedback(self, seconds=0.01):
        # 读取一小段时间内的反馈，并更新 last_feedback。
        feedback = self.receive_feedback(seconds)
        return feedback[-1] if feedback else self.last_feedback

    def get_feedback(self, max_age=None):
        # 获取最近一次反馈；max_age 用于要求反馈不能太旧。
        if self.last_feedback is None:
            return None
        if max_age is not None and self.feedback_age() > max_age:
            return None
        return self.last_feedback

    def get_position_rad(self, max_age=None):
        feedback = self.get_feedback(max_age)
        return None if feedback is None else feedback.position_rad

    def get_position_deg(self, max_age=None):
        pos = self.get_position_rad(max_age)
        return None if pos is None else pos * 180 / math.pi

    def get_velocity_rad_s(self, max_age=None):
        feedback = self.get_feedback(max_age)
        return None if feedback is None else feedback.velocity_rad_s

    def get_torque_nm(self, max_age=None):
        feedback = self.get_feedback(max_age)
        return None if feedback is None else feedback.torque_nm

    def get_temperature_c(self, max_age=None):
        feedback = self.get_feedback(max_age)
        return None if feedback is None else feedback.temperature_c

    def get_fault_bits(self, max_age=None):
        feedback = self.get_feedback(max_age)
        return None if feedback is None else feedback.fault_bits

    def get_mode_state(self, max_age=None):
        # 这是状态机模式：0 Reset，1 Cali，2 Motor，不是 run_mode 的 PP/速度/电流。
        feedback = self.get_feedback(max_age)
        return None if feedback is None else feedback.mode_state

    def feedback_summary(self, max_age=None):
        feedback = self.get_feedback(max_age)
        if feedback is None:
            return None
        return {
            "motor_id": feedback.motor_id,
            "mode_state": feedback.mode_state,
            "fault_bits": feedback.fault_bits,
            "position_rad": feedback.position_rad,
            "position_deg": feedback.position_rad * 180 / math.pi,
            "velocity_rad_s": feedback.velocity_rad_s,
            "torque_nm": feedback.torque_nm,
            "temperature_c": feedback.temperature_c,
            "age_s": self.feedback_age(),
        }

    def motion_control(self, pos_rad, vel_rad_s=0.0, kp=0.0, kd=0.0, torque_nm=0.0):
        # 通信类型 1：运控模式控制帧。实机先用低 Kp/Kd 和小角度测试。
        torque_u = float_to_uint(torque_nm, T_MIN, T_MAX, 16)
        pos_u = float_to_uint(pos_rad, P_MIN, P_MAX, 16)
        vel_u = float_to_uint(vel_rad_s, V_MIN, V_MAX, 16)
        kp_u = float_to_uint(kp, KP_MIN, KP_MAX, 16)
        kd_u = float_to_uint(kd, KD_MIN, KD_MAX, 16)

        can_id = make_can_id(0x01, torque_u, self.motor_id)
        data = bytearray(8)
        data[0:2] = pos_u.to_bytes(2, "big")
        data[2:4] = vel_u.to_bytes(2, "big")
        data[4:6] = kp_u.to_bytes(2, "big")
        data[6:8] = kd_u.to_bytes(2, "big")
        return self.send_can(can_id, data)

    def motion_control_safe(
        self,
        pos_rad,
        vel_rad_s=0.0,
        kp=0.0,
        kd=0.0,
        torque_nm=0.0,
        min_pos_rad=-math.pi / 4,
        max_pos_rad=math.pi / 4,
        max_abs_vel=1.0,
        max_kp=30.0,
        max_kd=3.0,
        max_abs_torque=1.0,
        feedback_timeout=0.15,
    ):
        # 实时控制用保护层：限幅 + 故障/反馈超时检查。
        if self.last_feedback is None:
            raise RuntimeError("no feedback received yet; call require_no_fault() before realtime control")

        age = self.feedback_age()
        if age is None or age > feedback_timeout:
            self.stop()
            raise TimeoutError(f"feedback timeout: {age}")

        if self.last_feedback.fault_bits:
            self.stop()
            raise RuntimeError(f"motor fault bits: 0x{self.last_feedback.fault_bits:02x}")

        pos_rad = max(min(pos_rad, max_pos_rad), min_pos_rad)
        vel_rad_s = max(min(vel_rad_s, max_abs_vel), -max_abs_vel)
        kp = max(min(kp, max_kp), 0.0)
        kd = max(min(kd, max_kd), 0.0)
        torque_nm = max(min(torque_nm, max_abs_torque), -max_abs_torque)
        return self.motion_control(pos_rad, vel_rad_s, kp, kd, torque_nm)

    def receive_frames(self, seconds=0.2):
        # 读取串口返回。这里只按当前 AT 帧格式拆包。
        end_time = time.time() + seconds
        frames = []
        while time.time() < end_time:
            chunk = self.ser.read(128)
            if chunk:
                self._rx_buffer.extend(chunk)
            while len(self._rx_buffer) >= 17:
                if self._rx_buffer[:2] == b"AT" and self._rx_buffer[6] == 0x08:
                    raw = bytes(self._rx_buffer[:17])
                    try:
                        frames.append(parse_serial_frame(raw))
                    except ValueError:
                        pass
                    del self._rx_buffer[:17]
                else:
                    del self._rx_buffer[0]
        return frames

    def receive_feedback(self, seconds=0.2):
        # 只保留通信类型 2 的电机反馈帧。
        feedback = []
        for can_id, data8 in self.receive_frames(seconds):
            comm_type = (can_id >> 24) & 0x1F
            if comm_type == 0x02:
                item = parse_feedback(can_id, data8)
                feedback.append(item)
                if item.motor_id == self.motor_id:
                    self.last_feedback = item
        return feedback


def parse_feedback(can_id, data8):
    # 通信类型 2：CAN ID 中包含电机 ID、故障位、状态机模式；data8 中包含位置/速度/力矩/温度。
    data2 = (can_id >> 8) & 0xFFFF
    motor_id = data2 & 0xFF
    fault_bits = (can_id >> 16) & 0x3F
    mode_state = (can_id >> 22) & 0x03

    pos_u = int.from_bytes(data8[0:2], "big")
    vel_u = int.from_bytes(data8[2:4], "big")
    torque_u = int.from_bytes(data8[4:6], "big")
    temp_u = int.from_bytes(data8[6:8], "big")

    return Feedback(
        timestamp=time.time(),
        motor_id=motor_id,
        mode_state=mode_state,
        fault_bits=fault_bits,
        position_rad=uint_to_float(pos_u, P_MIN, P_MAX, 16),
        velocity_rad_s=uint_to_float(vel_u, V_MIN, V_MAX, 16),
        torque_nm=uint_to_float(torque_u, T_MIN, T_MAX, 16),
        temperature_c=temp_u / 10,
    )
