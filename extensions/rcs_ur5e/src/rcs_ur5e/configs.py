from rcs._core.common import GripperType, RobotType
from rcs.envs.base import ControlMode, RelativeTo
from rcs_ur5e.creators import RCSUR5eConfigEnvCreator, UR5eHardwareEnvCreatorConfig
from rcs_ur5e.hw import RobotiQGripperConfig, UR5eConfig

import rcs


class DefaultUR5eHardwareEnv(RCSUR5eConfigEnvCreator):
    ip = "192.168.1.15"

    def config(self) -> UR5eHardwareEnvCreatorConfig:
        robot_type = RobotType("UR5e")
        robot_cfg = UR5eConfig(
            ip=self.ip,
            max_velocity=1.0,
            max_acceleration=1.0,
            async_control=False,
            max_servo_joint_step=0.15,
            max_servo_cartesian_step=0.01,
            lookahead_time=0.05,
            gain=500.0,
            robot_type=robot_type,
            kinematic_model_path=rcs.ROBOTS[robot_type].mjcf_model_path,
            attachment_site=rcs.ROBOTS[robot_type].attachment_site,
            dof=rcs.ROBOTS[robot_type].dof,
            joint_limits=rcs.ROBOTS[robot_type].joint_limits,
            q_home=rcs.ROBOTS[robot_type].q_home,
            tcp_offset=rcs.common.Pose(),
        )

        gripper_cfg = RobotiQGripperConfig(
            ip=self.ip,
            gripper_type=GripperType("Robotiq2F85"),
        )

        return UR5eHardwareEnvCreatorConfig(
            control_mode=ControlMode.CARTESIAN_TRPY,
            robot_cfg=robot_cfg,
            gripper_cfg=gripper_cfg,
            camera_cfgs=None,
            max_relative_movement=(0.2, 0.2),
            relative_to=RelativeTo.LAST_STEP,
        )
