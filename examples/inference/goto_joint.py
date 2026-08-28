"""Move the FR3 to each trajectory's final joint pose, holding 5 s at each (sync)."""
from time import sleep

import numpy as np
from rcs_fr3._core.hw import Franka, FR3Config
from rcs_robotiq2f85.hw import RobotiQ2F85Gripper, RobotiQ2F85GripperConfig

ROBOT_IP = "192.168.1.12"
GRIPPER_SERIAL = "DAANTG8W"

# Last valid action.joint_target (abs, rad) for fingertip_hover_success ep0-9.
QS = np.array([
    [-0.042342, 0.410310, 0.499609, -2.381218, -0.466364, 2.703196, 2.002923],  # ep0
    [0.224871, 0.337841, 0.079495, -2.427896, -0.071115, 2.762768, -0.290005],  # ep1
    [0.289396, 0.056239, -0.306664, -2.934712, 0.110900, 2.987024, -1.002543],  # ep2
    [0.021360, 0.544071, -0.166270, -2.057086, 0.164291, 2.589931, -1.004811],  # ep3
    [-0.017696, 0.543254, -0.124652, -2.053298, 0.123060, 2.590653, -0.629088],  # ep4
    [-0.090761, 0.448246, -0.023312, -2.223050, 0.022315, 2.671422, 0.191700],  # ep5
    [0.020851, 0.290847, 0.247079, -2.528109, -0.212252, 2.802645, 0.785708],  # ep6
    [0.074895, 0.464143, 0.053343, -2.194503, -0.051299, 2.657880, 0.004569],  # ep7
    [-0.258712, 0.244958, -0.195653, -2.604216, 0.159292, 2.840215, -0.033734],  # ep8
    [0.013336, 0.519015, -0.133797, -2.099165, 0.131226, 2.611363, -0.763780],  # ep9
])

robot = Franka(FR3Config(ip=ROBOT_IP, async_control=False, speed_factor=0.2,
                         ignore_realtime=True))
# gripper = RobotiQ2F85Gripper(
#     RobotiQ2F85GripperConfig(serial_number=GRIPPER_SERIAL, speed=100, force=50,
#                              async_control=False)  # sync: close() blocks
# )
# gripper.reset()  # activate the gripper (needed once after power-up)

input(f"Press Enter to visit {len(QS)} poses, 5 s each (Ctrl+C to abort)...")
for i, q in enumerate(QS):
    print(f"[{i + 1}/{len(QS)}] moving to ep{i}: {np.round(q, 3)}")
    robot.set_joint_position(q)   # sync: blocks until the motion finishes
    # gripper.grasp()               # sync: blocks until the gripper is closed
    sleep(2)
print("done")
