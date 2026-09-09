from rcs._core.common import RobotType
from rcs.envs.base import ControlMode, RelativeTo
from rcs_flexiv.creators import (
    FlexivHardwareEnvCreatorConfig,
    FlexivMultiHardwareEnvCreatorConfig,
    RCSFlexivConfigEnvCreator,
    RCSFlexivMultiConfigEnvCreator,
)
from rcs_flexiv.hw import FlexivConfig, FlexivControlMode, FlexivGripperConfig

import rcs


class DefaultRizon4SHardwareEnv(RCSFlexivConfigEnvCreator):
    """Rizon 4s with a Grav GN-01 gripper, Cartesian control through RCS's IK and joint impedance on the robot."""

    robot_sn = "Rizon4s-123456"
    gripper_device_name = "Flexiv-GN01"
    """Gripper device name from Flexiv Elements -> Settings -> Device."""
    tool_name: str | None = None
    """Tool to activate from Flexiv Elements -> Settings -> Tool, e.g. the one created for the Grav. None keeps the
    active tool. The active tool matters for gravity compensation, so it should match the mounted gripper."""

    def config(self) -> FlexivHardwareEnvCreatorConfig:
        robot_type = RobotType("Rizon4S")
        gripper_type = rcs.common.GripperType("FlexivGrav")
        robot_cfg = FlexivConfig(
            robot_sn=self.robot_sn,
            control_mode=FlexivControlMode.JOINT_IMPEDANCE,
            async_control=False,
            tool_name=self.tool_name,
            robot_type=robot_type,
            kinematic_model_path=rcs.ROBOTS[robot_type].mjcf_model_path,
            attachment_site=rcs.ROBOTS[robot_type].attachment_site,
            dof=rcs.ROBOTS[robot_type].dof,
            joint_limits=rcs.ROBOTS[robot_type].joint_limits,
            q_home=rcs.ROBOTS[robot_type].q_home,
            tcp_offset=rcs.GRIPPER_TCP_OFFSETS[gripper_type],
        )

        gripper_cfg = FlexivGripperConfig(
            device_name=self.gripper_device_name,
            async_control=False,
            gripper_type=gripper_type,
        )

        return FlexivHardwareEnvCreatorConfig(
            control_mode=ControlMode.CARTESIAN_TQuat,
            robot_cfg=robot_cfg,
            gripper_cfg=gripper_cfg,
            max_relative_movement=0.2,
            relative_to=RelativeTo.LAST_STEP,
        )


class DefaultRizon4SDualMultiHardwareEnv(RCSFlexivMultiConfigEnvCreator):
    """Two Rizon 4s in the duo arrangement of `rcs/rizon4s_duo`: bases 0.3 m apart in y, each tilted 45 degrees
    about x away from the other arm (right +45, left -45 degrees). Both arms run in async mode, as needed for
    teleoperation."""

    left_sn = "Rizon4s-123456"
    right_sn = "Rizon4s-654321"
    gripper_device_name = "Flexiv-GN01"
    tool_name: str | None = None

    def config(self) -> FlexivMultiHardwareEnvCreatorConfig:
        base = DefaultRizon4SHardwareEnv()
        base.gripper_device_name = self.gripper_device_name
        base.tool_name = self.tool_name

        robot_cfgs = {}
        gripper_cfgs: dict[str, FlexivGripperConfig | None] = {}
        for name, sn in (("left", self.left_sn), ("right", self.right_sn)):
            base.robot_sn = sn
            cfg = base.config()
            cfg.robot_cfg.async_control = True
            cfg.robot_cfg.q_home = rcs.HOME_POSITIONS[f"RIZON4S_DUO_{name.upper()}"]
            assert cfg.gripper_cfg is not None
            cfg.gripper_cfg.async_control = True
            robot_cfgs[name] = cfg.robot_cfg
            gripper_cfgs[name] = cfg.gripper_cfg

        return FlexivMultiHardwareEnvCreatorConfig(
            control_mode=ControlMode.CARTESIAN_TQuat,
            robot_cfgs=robot_cfgs,
            gripper_cfgs=gripper_cfgs,
            max_relative_movement=0.2,
            relative_to=RelativeTo.LAST_STEP,
            robot_to_shared_base_frame={
                "left": rcs.DEFAULT_TRANSFORMS["RIZON4S_DUO_LEFT_ROBOT"],
                "right": rcs.DEFAULT_TRANSFORMS["RIZON4S_DUO_RIGHT_ROBOT"],
            },
        )
