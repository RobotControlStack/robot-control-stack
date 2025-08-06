from xarm.wrapper import XArmAPI
import numpy as np
import time
xarm = XArmAPI(port="192.168.1.245")
xarm.motion_enable(enable=True)
xarm.clean_error()
xarm.set_mode(0)
xarm.set_state(state=0)
# xarm.set_servo_angle(angle=np.zeros(7).tolist(), is_radian=True, wait=True)
print(xarm.get_position())
xarm.set_position(x=10, is_radian=True, relative=True, wait=True)
print(xarm.get_position())
