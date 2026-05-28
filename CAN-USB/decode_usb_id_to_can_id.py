# 从串口帧里的 4 字节 ID 反推出真实 CAN ID。
# 例：90 07 e8 0c -> 0x1200fd01


usb_id = 0x9007E80C

can_id = usb_id >> 3

print(f"usb_id: 0x{usb_id:08x}")
print(f"can_id: 0x{can_id:08x}")
