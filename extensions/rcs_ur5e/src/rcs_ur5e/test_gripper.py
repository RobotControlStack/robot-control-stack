from rcs_ur5e.hw import RobotiQGripper

ROBOT_IP = "192.168.25.201"

gripper = RobotiQGripper(ROBOT_IP)
gripper.open()
print(f"Gripper width: {gripper.get_normalized_width()}")

print("Grasping...")
gripper.grasp()
input("Press Enter to continue...")
print("Opening...")
gripper.open()
input("Press Enter to continue...")
print("Shutting...")
gripper.shut()
input("Press Enter to continue...")
print(f"Gripper width: {gripper.get_normalized_width()}")
