#20260527，这一版是EL05电机PP模式运行MVP，所有注释的东西都不必要。
import serial
import time
import struct

# ===== 参数（你只改这里）=====
PORT = "COM6"
BAUD = 921600
MOTOR_ID = 1
ANGLE_DEG = 0

# ===== 固定参数 =====
HOST_ID = 0xFD

# ===== 构造29位CANID =====
def make_canid(comm_type, host_id, motor_id):
    return ((comm_type & 0x1F) << 24) | ((host_id & 0xFFFF) << 8) | (motor_id & 0xFF)

# ===== CAN → 串口帧 =====
def build_frame(canid, data8):
    usb_id = (canid << 3) | 0x4
    frame = b"\x41\x54"                  # 头
    frame += usb_id.to_bytes(4, "big")   # ID
    frame += b"\x08"                     # 长度
    frame += data8                       # 数据
    frame += b"\x0d\x0a"                 # 尾
    return frame

# ===== 构造角度指令 =====
def build_angle_frame(motor_id, angle_deg):
    angle_rad = angle_deg * 3.1415926 / 180

    canid = make_canid(0x12, HOST_ID, motor_id)  # 通信类型18

    data = bytearray(8)
    data[0:2] = struct.pack("<H", 0x7016)  # 位置指令
    data[2:4] = b"\x00\x00"
    data[4:8] = struct.pack("<f", angle_rad)

    return build_frame(canid, data)

# ===== 主程序 =====
ser = serial.Serial(PORT, BAUD)
time.sleep(0.2)

# # 1. AT握手
# ser.write(b"AT+AT\r\n")
# time.sleep(0.1)

# # 2. 初始化（直接用你同事抓的帧）
# ser.write(bytes.fromhex("41542007e80c0800c40000000000000d0a")) # 检测设备
# time.sleep(0.05)

# ser.write(bytes.fromhex("41542007e80c0800000000000000000d0a")) # 停止
# time.sleep(0.08)
# ser.write(bytes.fromhex("41542007e80c0800000000000000000d0a")) # 停止（再发一次保险）
# time.sleep(0.10)

ser.write(bytes.fromhex("41549007e80c0805700000010000000d0a")) # 切到插补位置模式（run_mode=1）
time.sleep(0.05)

ser.write(bytes.fromhex("41541807e80c0800000000000000000d0a")) # 运行/使能
time.sleep(0.05)

# ser.write(bytes.fromhex("41549007e80c0824700000000000400d0a")) # 速度设置 2.0
# time.sleep(0.05)

# ser.write(bytes.fromhex("41549007e80c0825700000000000400d0a")) # 加减速度设置 2.0
# time.sleep(0.05)

# 3. 发角度（真正控制）
frame = build_angle_frame(MOTOR_ID, ANGLE_DEG)
ser.write(frame)
print(frame.hex())
print("已发送角度:", ANGLE_DEG)

ser.close()