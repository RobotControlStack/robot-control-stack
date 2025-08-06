import typing

import numpy as np
from xarm.wrapper import XArmAPI

from rcs import common


class XArm7(common.Robot):
    def __init__(self, ip: str, urdf_path: str):
        self.ik = common.RL(urdf_path=urdf_path)
        self._xarm = XArmAPI(ip=ip)

    # def get_base_pose_in_world_coordinates(self) -> Pose: ...
    def get_cartesian_position(self) -> common.Pose:
        return self.ik.forward(self.get_joint_position())

    def get_ik(self) -> common.IK | None:
        return self.ik

    def get_joint_position(self) -> np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]]:  # type: ignore
        obs = self._hf_robot.get_servo_angle(is_radian=True)[1]
        return np.array(obs, dtype=np.float64)

    def get_parameters(self):
        a = common.RobotConfig()
        a.robot_platform = common.RobotPlatform.HARDWARE
        a.robot_type = common.RobotType.XArm7
        return a

    def get_state(self) -> common.RobotState:
        return common.RobotState()

    def move_home(self) -> None:
        home = typing.cast(
            np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]],
            common.robots_meta_config(common.RobotType.XArm7).q_home,
        )
        self.set_joint_position(home)

    def reset(self) -> None:
        pass

    def set_cartesian_position(self, pose: common.Pose) -> None:
        joints = self.ik.ik(pose, q0=self.get_joint_position())
        if joints is not None:
            self.set_joint_position(joints)

    def set_joint_position(self, q: np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]]) -> None:  # type: ignore
        self._hf_robot.set_servo_angle(angle=q, is_radian=True, wait=True)

    # def to_pose_in_robot_coordinates(self, pose_in_world_coordinates: Pose) -> Pose: ...
    # def to_pose_in_world_coordinates(self, pose_in_robot_coordinates: Pose) -> Pose: ...

