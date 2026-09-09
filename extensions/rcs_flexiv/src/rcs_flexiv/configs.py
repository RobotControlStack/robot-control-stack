import rcs
from rcs._core.common import RobotType
from rcs.envs.base import ControlMode, RelativeTo

from rcs_flexiv.creators import (
    FlexivHardwareEnvCreatorConfig,
    RCSFlexivConfigEnvCreator,
)
from rcs_flexiv.hw import FlexivConfig, FlexivControlMode, FlexivGripperConfig


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
