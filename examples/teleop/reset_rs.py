import time
import pyrealsense2 as rs

ctx = rs.context()

for dev in ctx.query_devices():
    name = dev.get_info(rs.camera_info.name)
    serial = dev.get_info(rs.camera_info.serial_number)
    print(f"Resetting {name} serial={serial}")
    dev.hardware_reset()

time.sleep(1)
print("Reset complete")