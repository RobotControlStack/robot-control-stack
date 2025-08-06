from dataclasses import dataclass, field
import typing
from typing import List

import numpy as np
from xarm.wrapper import XArmAPI

from rcs import common

from scipy.spatial.transform import Rotation as R


@dataclass(kw_only=True)
class XArm7Config(common.RobotConfig):
    # some_custom_config: str = "default_value"
    payload_weight: float = 0.624
    payload_tcp: List[float] = field(default_factory=lambda: [-4.15, 5.24, 76.38])
    def __post_init__(self):
        super().__init__()


class XArm7():
    def __init__(self, ip: str):
        self.ik = None #common.RL(urdf_path=urdf_path)
        self._config = XArm7Config()
        self._config.robot_platform = common.RobotPlatform.HARDWARE
        self._config.robot_type = common.RobotType.XArm7

        self._xarm = XArmAPI(ip)
        self._xarm.set_mode(0)
        self._xarm.clean_error()
        self._xarm.clean_warn()
        self._xarm.motion_enable(enable=True)
        self._xarm.set_state(state=0)
        self._xarm.set_tcp_load(
            weight=self._config.payload_weight,
            center_of_gravity=self._config.payload_tcp,
            wait=True,
        )

    # def get_base_pose_in_world_coordinates(self) -> Pose: ...
    
    def get_cartesian_position(self) -> common.Pose:
        code, xyzrpy = self._xarm.get_position(is_radian=True)
        if code != 0:
            raise RuntimeError("couldn't get cartesian position from xarm")

        translation = np.array(xyzrpy[:3], dtype=np.float64).reshape((3, 1)) * 0.001
        rpy = xyzrpy[3:]
        quat = R.from_euler('xyz', rpy).as_quat()  # [x, y, z, w]
        quat = np.array(quat, dtype=np.float64).reshape((4, 1))

        # return common.Pose(quaternion=quat, translation=translation)
        return common.Pose(rpy_vector=rpy, translation=translation)

    def get_ik(self) -> common.IK | None:
        return self.ik

    def get_joint_position(self) -> np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]]:  # type: ignore
        obs = self._xarm.get_servo_angle(is_radian=True)[1]
        return np.array(obs, dtype=np.float64)

    def get_parameters(self) -> XArm7Config:
        return self._config
    
    def set_parameters(self, robot_cfg: XArm7Config) -> None:
        self._config = robot_cfg

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
        x, y, z, roll, pitch, yaw = pose.xyzrpy()
        x_mm, y_mm, z_mm = 1000 * x, 1000 * y, 1000 * z
        self._xarm.set_position(x_mm, y_mm, z_mm, roll, pitch, yaw, is_radian=True, wait=True)

    def set_joint_position(self, q: np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]]) -> None:  # type: ignore
        self._xarm.set_servo_angle(angle=q, is_radian=True, wait=True)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self._xarm.disconnect()


    # def to_pose_in_robot_coordinates(self, pose_in_world_coordinates: Pose) -> Pose: ...
    # def to_pose_in_world_coordinates(self, pose_in_robot_coordinates: Pose) -> Pose: ...

