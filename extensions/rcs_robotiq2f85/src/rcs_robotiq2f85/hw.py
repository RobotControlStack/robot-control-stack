from rcs._core.common import Gripper
from Robotiq2F85Driver.Robotiq2F85Driver import Robotiq2F85Driver


class RobotiQGripper(Gripper):
    def __init__(self, serial_number):
        super().__init__()
        self.gripper = Robotiq2F85Driver(serial_number=serial_number)
        # self.gripper.reset()
        # self.gripper.deactivate()
        # self.gripper.activate()


    def get_normalized_width(self) -> float:
        # value between 0 and 1 (0 is closed)
        return self.gripper.opening / 85.0

    def grasp(self) -> None:
        """
        Close the gripper to grasp an object.
        """
        self.gripper.go_to(opening=0, speed=150.0, force=30.0)

    def open(self) -> None:
        """
        Open the gripper to its maximum width.
        """
        self.gripper.go_to(opening=85.0, speed=150.0, force=30.0)

    def reset(self) -> None:
        self.gripper.reset()
        self.gripper.deactivate()
        self.gripper.activate()

    def set_normalized_width(self, width: float, _: float = 0) -> None:
        """
        Set the gripper width to a normalized value between 0 and 1.
        """
        if not (0 <= width <= 1):
            msg = f"Width must be between 0 and 1, got {width}."
            raise ValueError(msg)
        abs_width = width * 85
        self.gripper.go_to(opening=float(abs_width), speed=150.0, force=30.0)

    def shut(self) -> None:
        """
        Close the gripper.
        """
        self.gripper.go_to(opening=0, speed=150.0, force=30.0)

    def close(self) -> None:
        self.gripper.go_to(opening=0, speed=150.0, force=30.0)
        

if __name__ == "__main__":
    print("[DEBUG] Creating Robotiq2F85Driver instance...")
    robotiq_2f85_driver = Robotiq2F85Driver(serial_number="DAAQM5FM", debug=False)
    print("[DEBUG] Resetting gripper...")
    robotiq_2f85_driver.reset()
    print("[DEBUG] Activating gripper...")
    robotiq_2f85_driver.deactivate()
    robotiq_2f85_driver.activate()
    print("[DEBUG] Sending go_to(opening=0, speed=150.0, force=30.0)...")
    robotiq_2f85_driver.go_to(opening=85, speed=150.0, force=30.0)
    print("[DEBUG] go_to command sent.")
