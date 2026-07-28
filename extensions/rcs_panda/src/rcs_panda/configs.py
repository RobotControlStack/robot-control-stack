import copy

import numpy as np
from rcs._core.common import RobotType
from rcs.envs.base import ControlMode, RelativeTo
from rcs_panda._core import hw
from rcs_panda.creators import (
    PandaHardwareEnvCreatorConfig,
    PandaMultiHardwareEnvCreatorConfig,
    RCSPandaConfigEnvCreator,
    RCSPandaMultiConfigEnvCreator,
)

import rcs
from rcs import common


class DefaultSinglePandaHardwareEnv(RCSPandaConfigEnvCreator):
    ip = "192.168.4.100"

    def config(self) -> PandaHardwareEnvCreatorConfig:
        robot_cfg = hw.PandaConfig(ip=self.ip)
        robot_cfg.robot_type = RobotType.Panda
        robot_cfg.kinematic_model_path = rcs.ROBOTS[RobotType.Panda].mjcf_model_path
        robot_cfg.tcp_offset = common.Pose(common.FrankaHandTCPOffset())
        robot_cfg.attachment_site = rcs.ROBOTS[RobotType.Panda].attachment_site
        robot_cfg.speed_factor = 0.1
        robot_cfg.ik_solver = hw.IKSolver.rcs_ik
        robot_cfg.async_control = False

        gripper_cfg = hw.FHConfig(ip=self.ip)
        gripper_cfg.epsilon_inner = gripper_cfg.epsilon_outer = 0.1
        gripper_cfg.speed = 0.1
        gripper_cfg.force = 30
        gripper_cfg.async_control = False

        return PandaHardwareEnvCreatorConfig(
            control_mode=ControlMode.CARTESIAN_TQuat,
            robot_cfg=robot_cfg,
            gripper_cfg=gripper_cfg,
            camera_cfgs=None,
            max_relative_movement=0.2,
            relative_to=RelativeTo.LAST_STEP,
        )


class DefaultPandaHardwareEnv(RCSPandaMultiConfigEnvCreator):
    ip = "192.168.4.100"
    robot_name = "right"

    def config(self) -> PandaMultiHardwareEnvCreatorConfig:
        base = DefaultSinglePandaHardwareEnv()
        base.ip = self.ip
        cfg = base.config()

        return PandaMultiHardwareEnvCreatorConfig(
            control_mode=cfg.control_mode,
            robot_cfgs={
                self.robot_name: cfg.robot_cfg,
            },
            gripper_cfgs={
                self.robot_name: cfg.gripper_cfg,
            },
            camera_cfgs=None,
            max_relative_movement=cfg.max_relative_movement,
            relative_to=cfg.relative_to,
            robot_to_shared_base_frame={
                self.robot_name: common.Pose(
                    translation=np.array([0, 0, 0]),
                    rpy_vector=np.array([0, 0, 0]),
                ),
            },
            wrapper_cfg=cfg.wrapper_cfg,
        )


class DROIDEnv(RCSPandaMultiConfigEnvCreator):
    robot_ip = "192.168.4.100"
    gripper_serial_number = "DAAQMPDC"

    def config(self) -> PandaMultiHardwareEnvCreatorConfig:
        try:
            from rcs_robotiq2f85.hw import RobotiQ2F85GripperConfig
        except ImportError as e:
            msg = "Robotiq gripper support requires the `rcs_robotiq2f85` extension to be installed."
            raise ImportError(msg) from e

        base = DefaultSinglePandaHardwareEnv()

        cfg = base.config()
        cfg.robot_cfg.async_control = True
        cfg.robot_cfg.ip = self.robot_ip
        cfg.robot_cfg.tcp_offset = rcs.GRIPPER_TCP_OFFSETS[common.GripperType("Robotiq2F85")]
        cfg.robot_cfg.q_home = rcs.ROBOTS[RobotType.Panda].q_home

        return PandaMultiHardwareEnvCreatorConfig(
            control_mode=ControlMode.CARTESIAN_TQuat,
            robot_cfgs={
                "right": cfg.robot_cfg,
            },
            camera_cfgs=None,
            max_relative_movement=0.5,
            relative_to=RelativeTo.LAST_STEP,
            gripper_cfgs={
                "right": RobotiQ2F85GripperConfig(
                    serial_number=self.gripper_serial_number,
                    speed=100,
                    force=50,
                    async_control=True,
                ),
            },
            robot_to_shared_base_frame={
                "right": common.Pose(
                    translation=np.array([0, 0, 0]),
                    rpy_vector=np.array([0, 0, 0]),
                ),
            },
        )


class DefaultPandaMultiHardwareEnv(RCSPandaMultiConfigEnvCreator):
    left_ip = "192.168.4.100"
    right_ip = "192.168.4.101"

    def config(self) -> PandaMultiHardwareEnvCreatorConfig:
        base = DefaultSinglePandaHardwareEnv()

        base.ip = self.left_ip
        left_env_cfg = base.config()
        left_env_cfg.robot_cfg.async_control = True
        if isinstance(left_env_cfg.gripper_cfg, hw.FHConfig):
            left_env_cfg.gripper_cfg.async_control = True

        base.ip = self.right_ip
        right_env_cfg = base.config()
        right_env_cfg.robot_cfg.async_control = True
        if isinstance(right_env_cfg.gripper_cfg, hw.FHConfig):
            right_env_cfg.gripper_cfg.async_control = True

        return PandaMultiHardwareEnvCreatorConfig(
            control_mode=ControlMode.CARTESIAN_TQuat,
            robot_cfgs={
                "left": copy.deepcopy(left_env_cfg.robot_cfg),
                "right": copy.deepcopy(right_env_cfg.robot_cfg),
            },
            gripper_cfgs={
                "left": copy.deepcopy(left_env_cfg.gripper_cfg),
                "right": copy.deepcopy(right_env_cfg.gripper_cfg),
            },
            camera_cfgs=None,
            max_relative_movement=0.2,
            relative_to=RelativeTo.LAST_STEP,
            # this is an example how the robots are oriented to each other
            # in this example the have an aloha like mounting, facing each other
            # with a distance of 1.5m
            robot_to_shared_base_frame={
                "right": common.Pose(
                    translation=np.array([0, 0, 0]),
                    rpy_vector=np.array([0, 0, 0]),
                ),
                "left": common.Pose(
                    translation=np.array([1.5, 0, 0]),
                    rpy_vector=np.array([0, 0, np.pi]),
                ),
            },
        )
