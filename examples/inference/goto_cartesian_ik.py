"""Move the FR3 EE to an absolute Cartesian position via IK (sync, Robotiq TCP)."""
import numpy as np
import rcs
from rcs._core.common import GripperType, Pose
from rcs_fr3._core.hw import Franka, FR3Config
from rcs_fr3.creators import FrankIK

from rcs_robotiq2f85.hw import RobotiQ2F85Gripper, RobotiQ2F85GripperConfig

ROBOT_IP = "192.168.1.12"
GRIPPER_SERIAL = "DAANTG8W"
TARGET_XYZ = np.array([0.5, 0.0, 0.0])  # 40 cm in the robot base frame

# rcs_ik solver; keep as a module global to avoid GC issues (see creators.py).
ik = FrankIK()
robot = Franka(
    FR3Config(
        ip=ROBOT_IP,
        async_control=False,   # sync: the move blocks until finished
        ignore_realtime=True,
        speed_factor=0.2,
        tcp_offset=rcs.GRIPPER_TCP_OFFSETS[GripperType("Robotiq2F85")],  # Robotiq 2F-85 TCP
    ),
    ik,
)

# Keep the current EE orientation; only change the position.
cur = robot.get_cartesian_position()
M = cur.pose_matrix()
M[:3, 3] = TARGET_XYZ
target = Pose(pose_matrix=M)

input(f"Press Enter to IK-move the EE to {TARGET_XYZ} m (Ctrl+C to abort)...")
robot.set_cartesian_position(target)  # rcs_ik -> IK -> sync joint move (prints "IK failed" if unreachable)
print("done")

gripper = RobotiQ2F85Gripper(
    RobotiQ2F85GripperConfig(serial_number=GRIPPER_SERIAL, speed=100, force=50,
                             async_control=False)  # sync: close() blocks
)
gripper.reset()  # activate the gripper (needed once after power-up)
gripper.set_normalized_width(1.0)
