# 从真实 CAN ID 生成串口帧里的 4 字节 ID。
# 例：0x1200fd01 -> 90 07 e8 0c


can_id = 0x1200FD01

usb_id = (can_id << 3) | 0x04

print(f"can_id:      0x{can_id:08x}")
print(f"usb_id:      0x{usb_id:08x}")
print(f"usb_id hex:  {usb_id.to_bytes(4, 'big').hex(' ')}")
