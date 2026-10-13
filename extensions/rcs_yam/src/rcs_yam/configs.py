from rcs._core.common import GripperConfig, GripperType, RobotType
from rcs.envs.base import ControlMode, RelativeTo
from rcs_yam.creators import (
    RCSYamConfigEnvCreator,
    RCSYamMultiConfigEnvCreator,
    YamHardwareEnvCreatorConfig,
    YamMultiHardwareEnvCreatorConfig,
)
from rcs_yam.hw import YamConfig

import rcs


class DefaultYamHardwareEnv(RCSYamConfigEnvCreator):
    channel = "can0"
    # in N, <=0 turns off the limit
    # 50 N is the default from YAM
    gripper_force = 50.0

    def config(self) -> YamHardwareEnvCreatorConfig:
        robot_type = RobotType("Yam")
        gripper_type = GripperType("Yam")
        robot_cfg = YamConfig(
            channel=self.channel,
            gripper_type_id="linear_4310",
            gripper_force=self.gripper_force,
            async_control=False,
            robot_type=robot_type,
            kinematic_model_path=rcs.ROBOTS[robot_type].mjcf_model_path,
            attachment_site=rcs.ROBOTS[robot_type].attachment_site,
            dof=rcs.ROBOTS[robot_type].dof,
            joint_limits=rcs.ROBOTS[robot_type].joint_limits,
            q_home=rcs.ROBOTS[robot_type].q_home,
            # The gripper is part of the arm's mjcf, so this is the flange to grasp point offset.
            tcp_offset=rcs.GRIPPER_TCP_OFFSETS[gripper_type],
        )

        gripper_cfg = GripperConfig(gripper_type=gripper_type)

        return YamHardwareEnvCreatorConfig(
            control_mode=ControlMode.JOINTS,
            robot_cfg=robot_cfg,
            gripper_cfg=gripper_cfg,
            max_relative_movement=0.2,
            relative_to=RelativeTo.LAST_STEP,
        )


class DefaultYamDualMultiHardwareEnv(RCSYamMultiConfigEnvCreator):
    left_channel = "can0"
    right_channel = "can1"
    # in N, <=0 turns off the limit
    gripper_force = 50

    def config(self) -> YamMultiHardwareEnvCreatorConfig:
        base = DefaultYamHardwareEnv()
        base.gripper_force = self.gripper_force

        base.channel = self.left_channel
        left_cfg = base.config()
        left_cfg.robot_cfg.async_control = True

        base.channel = self.right_channel
        right_cfg = base.config()
        right_cfg.robot_cfg.async_control = True

        return YamMultiHardwareEnvCreatorConfig(
            control_mode=ControlMode.JOINTS,
            robot_cfgs={
                "left": left_cfg.robot_cfg,
                "right": right_cfg.robot_cfg,
            },
            gripper_cfgs={
                "left": left_cfg.gripper_cfg,
                "right": right_cfg.gripper_cfg,
            },
            max_relative_movement=0.2,
            relative_to=RelativeTo.LAST_STEP,
        )
