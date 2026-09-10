"""Close and open a Robotiq 2F-85 gripper with rcs and print the normalized finger distance.

Prerequisites:
  * rcs_robotiq2f85 extension installed (`pip install -ve extensions/rcs_robotiq2f85`)
  * read/write access to the gripper's serial port (add yourself to the `dialout` group)
  * find the serial number with `python -m rcs_robotiq2f85 serials`

Usage:
    python examples/fr3/robotiq_open_close.py
"""

import time

from rcs_robotiq2f85.hw import RobotiQ2F85Gripper, RobotiQ2F85GripperConfig

GRIPPER_SERIAL = "DAANTG8W"
CYCLES = 1
PAUSE_S = 1.0


def print_width(gripper: RobotiQ2F85Gripper, label: str) -> None:
    # 1.0 = fully open (85 mm), 0.0 = fully closed
    print(f"{label:>6}: normalized width = {gripper.get_normalized_width():.3f}")


def main() -> None:
    cfg = RobotiQ2F85GripperConfig(
        serial_number=GRIPPER_SERIAL,
        speed=100,  # mm/s, 20..150
        force=50,  # N, 20..235
        async_control=False,  # commands block until the gripper stops moving
    )
    gripper = RobotiQ2F85Gripper(cfg)  # resets (deactivate + activate) on construction
    try:
        gripper.open()
        print_width(gripper, "start")

        for i in range(CYCLES):
            print(f"--- cycle {i + 1}/{CYCLES}")
            gripper.set_normalized_width(1-0.72)
            print_width(gripper, "closed")
            time.sleep(PAUSE_S)

            # gripper.open()
            print_width(gripper, "open")
            time.sleep(PAUSE_S)
    except KeyboardInterrupt:
        pass
    finally:
        gripper.close()


if __name__ == "__main__":
    main()
