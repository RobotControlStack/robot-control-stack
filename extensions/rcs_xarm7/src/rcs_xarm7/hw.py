import typing
from typing import List

import numpy as np
from rcs.common_typing import RobotConfigKwargs
from xarm.wrapper import XArmAPI

from rcs import common


class XArm7Config(common.RobotConfig):

    def __init__(
        self,
        ip: str,
        payload_weight: float = 0.624,
        payload_tcp: List[float] | None = None,
        async_control: bool = False,
        use_internal_ik: bool = True,
        **kwargs: typing.Unpack[RobotConfigKwargs],
    ):
        super().__init__(**kwargs)
        self.ip = ip
        self.payload_weight = payload_weight
        self.payload_tcp = payload_tcp if payload_tcp is not None else [-4.15, 5.24, 76.38]
        self.async_control = async_control
        self.use_internal_ik = use_internal_ik


class XArm7(common.Robot):
    def __init__(self, cfg: XArm7Config, ik: common.Kinematics):
        super().__init__()

        self.ik = ik
        self._config = cfg
        self._config.robot_platform = common.RobotPlatform.HARDWARE
        self._config.robot_type = common.RobotType("XArm7")

        self._xarm = XArmAPI(cfg.ip)
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

    def get_cartesian_position(self) -> common.Pose:
        code, xyzrpy = self._xarm.get_position(is_radian=True)
        if code != 0:
            msg = "couldn't get cartesian position from xarm"
            raise RuntimeError(msg)

        x_mm, y_mm, z_mm = xyzrpy[:3]
        translation_meter = [x_mm * 0.001, y_mm * 0.001, z_mm * 0.001]
        rpy = xyzrpy[3:]

        return common.Pose(rpy_vector=rpy, translation=translation_meter)  # type: ignore

    def get_ik(self) -> common.Kinematics | None:
        return self.ik

    def get_joint_position(self) -> np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]]:  # type: ignore
        return typing.cast(
            np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]],
            np.array(self._xarm.get_servo_angle(is_radian=True)[1]),
        )

    def get_config(self) -> XArm7Config:
        return self._config

    def set_config(self, robot_cfg: XArm7Config) -> None:
        self._config = robot_cfg

    def get_state(self) -> common.RobotState:
        return common.RobotState()

    def move_home(self) -> None:
        home = typing.cast(
            np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]],
            self._config.q_home,
        )
        # self.set_joint_position(home)
        self._xarm.set_mode(0)
        self._xarm.set_state(0)
        self._xarm.set_servo_angle(angle=home, is_radian=True, wait=True)

    def reset(self) -> None:
        pass

    def set_cartesian_position(self, pose: common.Pose) -> None:
        if not self._config.use_internal_ik:
            target_joints = self.ik.inverse(pose=pose, q0=self.get_joint_position(), tcp_offset=self._config.tcp_offset)
            if target_joints is not None:
                self.set_joint_position(target_joints)
            return
        if self._config.async_control:
            self._xarm.set_mode(7)
            self._xarm.set_state(0)
        x, y, z, roll, pitch, yaw = pose.xyzrpy()
        x_mm, y_mm, z_mm = 1000 * x, 1000 * y, 1000 * z
        self._xarm.set_position(x_mm, y_mm, z_mm, roll, pitch, yaw, is_radian=True, wait=not self._config.async_control)

    def set_joint_position(self, q: np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]]) -> None:  # type: ignore
        if self._config.async_control:
            self._xarm.set_mode(6)
            self._xarm.set_state(0)
        self._xarm.set_servo_angle(angle=q, is_radian=True, wait=not self._config.async_control)

    def close(self):
        self._xarm.disconnect()
