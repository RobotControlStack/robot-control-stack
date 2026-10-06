"""Factory for the Robotiq 2F-85 gripper, registered as an `rcs.grippers` entry point."""

from rcs._core.common import Gripper, GripperConfig
from rcs_robotiq2f85.hw import RobotiQ2F85Gripper, RobotiQ2F85GripperConfig


def create_gripper(cfg: GripperConfig) -> Gripper:
    if not isinstance(cfg, RobotiQ2F85GripperConfig):
        msg = f"Expected RobotiQ2F85GripperConfig for robotiq gripper, got {type(cfg).__name__}"
        raise TypeError(msg)
    return RobotiQ2F85Gripper(cfg)
