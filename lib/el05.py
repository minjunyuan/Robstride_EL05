# -*- coding: utf-8 -*-
"""
el05.py - EL05 / RobStride EDULITE 05 私有 CAN 协议库

推荐用法：
    from el05 import EL05Bus

    with EL05Bus(port="COM7") as bus:
        bus.set_motion_mode(1)
        bus.enable(1)
        bus.motion_control(1, pos_rad=0.0, kp=2.0, kd=0.4)

说明：
- EL05Bus 是主类：一个串口/CAN 总线，可控制一个或多个电机。
- 所有控制方法都显式传入 motor_id，避免多电机场景下混淆。
"""

import math
import struct
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple, Union

import serial


# =========================
# 1. 协议常量
# =========================

# 运控模式量程
P_MIN, P_MAX = -12.57, 12.57
V_MIN, V_MAX = -50.0, 50.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -6.0, 6.0

# run_mode 参数 0x7005 的取值
RUN_MODE_MOTION = 0
RUN_MODE_PP = 1
RUN_MODE_SPEED = 2
RUN_MODE_CURRENT = 3
RUN_MODE_CSP = 5

# 常用参数 index
IDX_RUN_MODE = 0x7005
IDX_IQ_REF = 0x7006
IDX_SPD_REF = 0x700A
IDX_LIMIT_TORQUE = 0x700B
IDX_LOC_REF = 0x7016
IDX_LIMIT_SPD = 0x7017
IDX_LIMIT_CUR = 0x7018
IDX_ACC_RAD = 0x7022
IDX_VEL_MAX = 0x7024
IDX_ACC_SET = 0x7025
IDX_ZERO_STA = 0x7029  # zero_sta: 0=0~2π；1=-π~π


# =========================
# 2. 数据结构和基础工具
# =========================

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

    @property
    def position_deg(self) -> float:
        return self.position_rad * 180.0 / math.pi


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    x = float(x)
    if x < x_min or x > x_max:
        raise ValueError(f"value out of range: {x} not in [{x_min}, {x_max}]")
    return int((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))


def require_float_range(name: str, value: float, lo: float, hi: float) -> float:
    value = float(value)
    if value < lo or value > hi:
        raise ValueError(f"{name} out of range: {value} not in [{lo}, {hi}]")
    return value


def require_int_range(name: str, value: int, lo: int, hi: int) -> int:
    value = int(value)
    if value < lo or value > hi:
        raise ValueError(f"{name} out of range: {value} not in [{lo}, {hi}]")
    return value


def uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    return int(x) * (x_max - x_min) / ((1 << bits) - 1) + x_min


def make_can_id(comm_type: int, data2: int, motor_id: int) -> int:
    """EL05 29 位 CAN ID：通信类型 | 数据区2 | 目标电机 ID。"""
    comm_type = require_int_range("comm_type", comm_type, 0, 0x1F)
    data2 = require_int_range("data2", data2, 0, 0xFFFF)
    motor_id = require_int_range("motor_id", motor_id, 0, 0xFF)
    return (comm_type << 24) | (data2 << 8) | motor_id


def can_id_to_usb_id(can_id: int) -> int:
    """官方 USB-CAN 串口封装用的 ID。"""
    can_id = require_int_range("can_id", can_id, 0, 0x1FFFFFFF)
    return (can_id << 3) | 0x04


def build_serial_frame(can_id: int, data8: bytes) -> bytes:
    """串口发送帧：AT + USB_ID + 长度08 + 8字节CAN数据 + CRLF。"""
    if len(data8) != 8:
        raise ValueError("data8 must be exactly 8 bytes")
    usb_id = can_id_to_usb_id(can_id)
    return b"AT" + usb_id.to_bytes(4, "big") + b"\x08" + bytes(data8) + b"\r\n"


def parse_serial_frame(frame: bytes) -> Tuple[int, bytes]:
    """把 17 字节串口返回帧解析为：真实 CAN ID + 8字节数据。"""
    if len(frame) != 17:
        raise ValueError("serial frame must be 17 bytes")
    if frame[:2] != b"AT" or frame[6] != 0x08 or frame[-2:] != b"\r\n":
        raise ValueError("invalid serial frame")
    usb_id = int.from_bytes(frame[2:6], "big")
    can_id = usb_id >> 3
    return can_id, frame[7:15]


def parse_feedback(can_id: int, data8: bytes) -> Feedback:
    """通信类型 2：电机反馈帧。"""
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
        temperature_c=temp_u / 10.0,
    )


# =========================
# 3. 主类：多电机总线
# =========================

class EL05Bus:
    """一个串口/CAN 总线，可控制一个或多个 EL05 电机。"""

    def __init__(
        self,
        port: str = "COM6",
        baud: int = 921600,
        host_id: int = 0xFD,
        timeout: float = 0.02,
        adapter_handshake: bool = False,
    ):
        self.port = port
        self.baud = baud
        self.host_id = host_id
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self._rx_buffer = bytearray()
        self.feedback: Dict[int, Feedback] = {}
        time.sleep(0.2)

        # 某些 USB-CAN 转接板可能需要 AT 握手；当前示例验证不需要，默认关闭。
        if adapter_handshake:
            self.ser.write(b"AT+AT\r\n")
            time.sleep(0.1)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def flush_rx(self) -> None:
        self.ser.reset_input_buffer()
        self._rx_buffer.clear()
        self.feedback.clear()

    # ---------- 底层发送 ----------

    def send_can(self, can_id: int, data8: bytes) -> bytes:
        frame = build_serial_frame(can_id, data8)
        self.ser.write(frame)
        return frame

    def send_private(
        self,
        motor_id: int,
        comm_type: int,
        data8: Optional[bytes] = None,
        data2: Optional[int] = None,
    ) -> bytes:
        if data8 is None:
            data8 = bytes(8)
        if data2 is None:
            data2 = self.host_id
        return self.send_can(make_can_id(comm_type, data2, motor_id), data8)

    # ---------- 基本命令 ----------

    def stop(self, motor_id: int, clear_fault: bool = False) -> bytes:
        data = bytearray(8)
        if clear_fault:
            data[0] = 1
        return self.send_private(motor_id, 0x04, data)

    def enable(self, motor_id: int) -> bytes:
        return self.send_private(motor_id, 0x03, bytes(8))

    def set_zero(self, motor_id: int) -> bytes:
        """设置当前位置为机械零位。谨慎使用。"""
        data = bytearray(8)
        data[0] = 1
        return self.send_private(motor_id, 0x06, data)

    def read_version(self, motor_id: int) -> bytes:
        return self.send_private(motor_id, 0x04, bytes.fromhex("00c4000000000000"))

    def save_params(self, motor_id: int) -> bytes:
        """通信类型 22：保存参数。"""
        return self.send_private(motor_id, 0x16, bytes(8))

    def set_feedback_active(self, motor_id: int, enabled: bool = True) -> bytes:
        """通信类型 24：主动反馈开关。"""
        data = bytearray(8)
        data[6] = 1 if enabled else 0
        return self.send_private(motor_id, 0x18, data)

    # ---------- 参数读写 ----------

    def write_param_u8(self, motor_id: int, index: int, value: int) -> bytes:
        data = bytearray(8)
        index = require_int_range("index", index, 0, 0xFFFF)
        value = require_int_range("value", value, 0, 0xFF)
        data[0:2] = struct.pack("<H", index)
        data[4] = value
        return self.send_private(motor_id, 0x12, data)

    def write_param_u16(self, motor_id: int, index: int, value: int) -> bytes:
        data = bytearray(8)
        index = require_int_range("index", index, 0, 0xFFFF)
        value = require_int_range("value", value, 0, 0xFFFF)
        data[0:2] = struct.pack("<H", index)
        data[4:6] = struct.pack("<H", value)
        return self.send_private(motor_id, 0x12, data)

    def write_param_u32(self, motor_id: int, index: int, value: int) -> bytes:
        data = bytearray(8)
        index = require_int_range("index", index, 0, 0xFFFF)
        value = require_int_range("value", value, 0, 0xFFFFFFFF)
        data[0:2] = struct.pack("<H", index)
        data[4:8] = struct.pack("<I", value)
        return self.send_private(motor_id, 0x12, data)

    def write_param_float(self, motor_id: int, index: int, value: float) -> bytes:
        data = bytearray(8)
        index = require_int_range("index", index, 0, 0xFFFF)
        data[0:2] = struct.pack("<H", index)
        data[4:8] = struct.pack("<f", float(value))
        return self.send_private(motor_id, 0x12, data)

    def read_param(self, motor_id: int, index: int) -> bytes:
        data = bytearray(8)
        index = require_int_range("index", index, 0, 0xFFFF)
        data[0:2] = struct.pack("<H", index)
        return self.send_private(motor_id, 0x11, data)

    def set_zero_sta(self, motor_id: int, sta: int) -> bytes:
        """设置 zero_sta：0=上电位置 0~2π；1=上电位置 -π~π。"""
        if sta not in (0, 1):
            raise ValueError("zero_sta 只能是 0 或 1")
        return self.write_param_u8(motor_id, IDX_ZERO_STA, sta)

    # ---------- 模式切换 ----------

    def set_run_mode(self, motor_id: int, mode: int) -> bytes:
        if mode not in (RUN_MODE_MOTION, RUN_MODE_PP, RUN_MODE_SPEED, RUN_MODE_CURRENT, RUN_MODE_CSP):
            raise ValueError(f"unsupported run mode: {mode}")
        return self.write_param_u8(motor_id, IDX_RUN_MODE, mode)

    def set_motion_mode(self, motor_id: int) -> bytes:
        return self.set_run_mode(motor_id, RUN_MODE_MOTION)

    def set_pp_mode(self, motor_id: int) -> bytes:
        return self.set_run_mode(motor_id, RUN_MODE_PP)

    def set_speed_mode(self, motor_id: int) -> bytes:
        return self.set_run_mode(motor_id, RUN_MODE_SPEED)

    def set_current_mode(self, motor_id: int) -> bytes:
        return self.set_run_mode(motor_id, RUN_MODE_CURRENT)

    def set_csp_mode(self, motor_id: int) -> bytes:
        return self.set_run_mode(motor_id, RUN_MODE_CSP)

    # ---------- 常用参数封装 ----------

    def set_pp_speed(self, motor_id: int, rad_s: float) -> bytes:
        return self.write_param_float(motor_id, IDX_VEL_MAX, rad_s)

    def set_pp_acc(self, motor_id: int, rad_s2: float) -> bytes:
        return self.write_param_float(motor_id, IDX_ACC_SET, rad_s2)

    def set_target_rad(self, motor_id: int, rad: float) -> bytes:
        rad = require_float_range("target position rad", rad, P_MIN, P_MAX)
        return self.write_param_float(motor_id, IDX_LOC_REF, rad)

    def set_target_deg(self, motor_id: int, deg: float) -> bytes:
        return self.set_target_rad(motor_id, deg * math.pi / 180.0)

    def set_speed_ref(self, motor_id: int, rad_s: float) -> bytes:
        return self.write_param_float(motor_id, IDX_SPD_REF, rad_s)

    def set_speed_acc(self, motor_id: int, rad_s2: float) -> bytes:
        return self.write_param_float(motor_id, IDX_ACC_RAD, rad_s2)

    def set_limit_current(self, motor_id: int, amp: float) -> bytes:
        return self.write_param_float(motor_id, IDX_LIMIT_CUR, amp)

    def set_limit_torque(self, motor_id: int, nm: float) -> bytes:
        return self.write_param_float(motor_id, IDX_LIMIT_TORQUE, nm)

    def set_limit_speed(self, motor_id: int, rad_s: float) -> bytes:
        return self.write_param_float(motor_id, IDX_LIMIT_SPD, rad_s)

    def set_current_ref(self, motor_id: int, amp: float) -> bytes:
        return self.write_param_float(motor_id, IDX_IQ_REF, amp)

    # ---------- 运控模式控制帧 ----------

    def motion_control(
        self,
        motor_id: int,
        pos_rad: float,
        vel_rad_s: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.0,
        torque_nm: float = 0.0,
    ) -> bytes:
        """通信类型 1：运控模式控制帧。"""
        pos_rad = require_float_range("pos_rad", pos_rad, P_MIN, P_MAX)
        vel_rad_s = require_float_range("vel_rad_s", vel_rad_s, V_MIN, V_MAX)
        kp = require_float_range("kp", kp, KP_MIN, KP_MAX)
        kd = require_float_range("kd", kd, KD_MIN, KD_MAX)
        torque_nm = require_float_range("torque_nm", torque_nm, T_MIN, T_MAX)

        torque_u = float_to_uint(torque_nm, T_MIN, T_MAX, 16)
        pos_u = float_to_uint(pos_rad, P_MIN, P_MAX, 16)
        vel_u = float_to_uint(vel_rad_s, V_MIN, V_MAX, 16)
        kp_u = float_to_uint(kp, KP_MIN, KP_MAX, 16)
        kd_u = float_to_uint(kd, KD_MIN, KD_MAX, 16)

        data = bytearray(8)
        data[0:2] = pos_u.to_bytes(2, "big")
        data[2:4] = vel_u.to_bytes(2, "big")
        data[4:6] = kp_u.to_bytes(2, "big")
        data[6:8] = kd_u.to_bytes(2, "big")
        return self.send_can(make_can_id(0x01, torque_u, motor_id), data)

    # ---------- 接收反馈 ----------

    def receive_frames(self, seconds: float = 0.2) -> List[Tuple[int, bytes]]:
        """读取串口返回帧。"""
        end_time = time.time() + seconds
        frames: List[Tuple[int, bytes]] = []
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

    def receive_feedback(self, seconds: float = 0.2) -> List[Feedback]:
        """读取通信类型 2 的反馈，并更新 self.feedback。"""
        items: List[Feedback] = []
        for can_id, data8 in self.receive_frames(seconds):
            comm_type = (can_id >> 24) & 0x1F
            if comm_type == 0x02:
                fb = parse_feedback(can_id, data8)
                self.feedback[fb.motor_id] = fb
                items.append(fb)
        return items

    def wait_feedback(
        self, motor_ids: Union[int, Iterable[int]], timeout: float = 3.0
    ) -> Union[Feedback, Dict[int, Feedback]]:
        """等待指定电机反馈。motor_ids 可传单个 int，也可传 [1,2,3]。"""
        single = isinstance(motor_ids, int)
        ids = (motor_ids,) if single else tuple(motor_ids)
        end_time = time.time() + timeout
        while time.time() < end_time:
            self.receive_feedback(0.05)
            if all(mid in self.feedback for mid in ids):
                if single:
                    return self.feedback[ids[0]]
                return {mid: self.feedback[mid] for mid in ids}
        missing = [mid for mid in ids if mid not in self.feedback]
        raise TimeoutError(f"no feedback from motor ids: {missing}")

    def get_feedback(self, motor_id: int, max_age: Optional[float] = None) -> Optional[Feedback]:
        fb = self.feedback.get(motor_id)
        if fb is None:
            return None
        if max_age is not None and time.time() - fb.timestamp > max_age:
            return None
        return fb

    def feedback_age(self, motor_id: int) -> Optional[float]:
        fb = self.feedback.get(motor_id)
        return None if fb is None else time.time() - fb.timestamp

    def get_position_rad(self, motor_id: int, max_age: Optional[float] = None) -> Optional[float]:
        fb = self.get_feedback(motor_id, max_age)
        return None if fb is None else fb.position_rad

    def get_position_deg(self, motor_id: int, max_age: Optional[float] = None) -> Optional[float]:
        fb = self.get_feedback(motor_id, max_age)
        return None if fb is None else fb.position_deg

    def get_velocity_rad_s(self, motor_id: int, max_age: Optional[float] = None) -> Optional[float]:
        fb = self.get_feedback(motor_id, max_age)
        return None if fb is None else fb.velocity_rad_s

    def get_torque_nm(self, motor_id: int, max_age: Optional[float] = None) -> Optional[float]:
        fb = self.get_feedback(motor_id, max_age)
        return None if fb is None else fb.torque_nm

    def get_temperature_c(self, motor_id: int, max_age: Optional[float] = None) -> Optional[float]:
        fb = self.get_feedback(motor_id, max_age)
        return None if fb is None else fb.temperature_c

    def get_fault_bits(self, motor_id: int, max_age: Optional[float] = None) -> Optional[int]:
        fb = self.get_feedback(motor_id, max_age)
        return None if fb is None else fb.fault_bits

    def get_mode_state(self, motor_id: int, max_age: Optional[float] = None) -> Optional[int]:
        fb = self.get_feedback(motor_id, max_age)
        return None if fb is None else fb.mode_state
