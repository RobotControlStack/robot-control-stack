import time

from rcs_ur5e.hw import UR5e


ROBOT_IP = "192.168.25.201"

robot = UR5e(ROBOT_IP)
print(f"Robot joint positions: {robot.get_joint_position()}")
print(f"Robot cartesian position: {robot.get_cartesian_position()}")
print(f"Robot Parameters: {robot.get_parameters()}")
print(f"Robot State: {robot.get_state()}")

print("Moving to home position...")
robot.move_home()
input("Press Enter to continue...")
target_q = robot.get_joint_position()
target_q[0] += 0.05  # Move slightly
print(f"Setting joint position to {target_q}")
for _ in range(100):
    target_q[0] += 0.005  # Move slightly
    robot.set_joint_position(target_q)
    time.sleep(0.05)
