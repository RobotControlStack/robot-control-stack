from dataclasses import dataclass
import typing

import numpy as np

from rcs import common

@dataclass(kw_only=True)
class UR5eConfig(common.RobotConfig):
    some_custom_config: str = "default_value"
    def __post_init__(self):
        super().__init__()


@dataclass(kw_only=True)
class UR5eState(common.RobotState):
    some_custom_state: float = 0.0
    def __post_init__(self):
        super().__init__()


class UR5e: #(common.Robot): # should inherit and implement common.Robot, but currently there is a bug that needs to be fixed
    def __init__(self, ip: str, urdf_path: str):
        self.ik = common.RL(urdf_path=urdf_path)
        self._config = UR5eConfig() # with default values
        self._config.robot_platform = common.RobotPlatform.HARDWARE
        self._config.robot_type = common.RobotType.SO101

        # TODO initialize the robot with the ip

    def get_cartesian_position(self) -> common.Pose:
        # TODO: remove code below and return the cartesian position from the robot
        return self.ik.forward(self.get_joint_position()) 

    def get_ik(self) -> common.IK | None:
        return self.ik

    def get_joint_position(self) -> np.ndarray[tuple[typing.Literal[6]], np.dtype[np.float64]]:  # type: ignore
        # TODO: remove code below and return the joint position from the robot
        return np.zeros(6)
      

    def get_parameters(self) -> UR5eConfig:
        return self._config

    def set_parameters(self, robot_cfg: UR5eConfig) -> None:
        self._config = robot_cfg

    def get_state(self) -> UR5eState:
        return UR5eState

    def move_home(self) -> None:
        # TODO: check that the q_home in include/rcs/Robot.h is correct
        home = typing.cast(
            np.ndarray[tuple[typing.Literal[6]], np.dtype[np.float64]],
            common.robots_meta_config(common.RobotType.SO101).q_home,
        )
        self.set_joint_position(home)

    def reset(self) -> None:
        # TODO: any reset stuff
        pass

    def set_cartesian_position(self, pose: common.Pose) -> None:
        # TODO: remove code below and use robots way to set cartesian position
        # or (configurable with the config) use impediance to move to the pose
        joints = self.ik.ik(pose, q0=self.get_joint_position())
        if joints is not None:
            self.set_joint_position(joints)

    def set_joint_position(self, q: np.ndarray[tuple[typing.Literal[6]], np.dtype[np.float64]]) -> None:  # type: ignore
        # TODO
        pass
     


class RobtiQGripper: # (common.Gripper):
    def __init__(self):
        # TODO: initialize the gripper
        pass

    def get_normalized_width(self) -> float:
        # value between 0 and 1 (0 is closed)
        return 0

    def grasp(self) -> None:
        """
        Close the gripper to grasp an object.
        """
        self.shut()

    # def is_grasped(self) -> bool: ...

    def open(self) -> None:
        """
        Open the gripper to its maximum width.
        """
        self.set_normalized_width(1.0)

    def reset(self) -> None:
        # TODO: any reset stuff
        pass

    def set_normalized_width(self, width: float, _: float = 0) -> None:
        """
        Set the gripper width to a normalized value between 0 and 1.
        """
        if not (0 <= width <= 1):
            msg = f"Width must be between 0 and 1, got {width}."
            raise ValueError(msg)
        # TODO: set gripper width

    def shut(self) -> None:
        """
        Close the gripper.
        """
        self.set_normalized_width(0.0)
