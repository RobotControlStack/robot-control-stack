from rcs._core.common import RobotType
from rcs.envs.base import ControlMode, RelativeTo
from rcs_xarm7.creators import RCSXArm7ConfigEnvCreator, XArm7HardwareEnvCreatorConfig
from rcs_xarm7.hw import XArm7Config

import rcs


class DefaultXArm7HardwareEnv(RCSXArm7ConfigEnvCreator):
    ip = "192.168.1.245"

    def config(self) -> XArm7HardwareEnvCreatorConfig:
        robot_type = RobotType("XArm7")
        robot_cfg = XArm7Config(
            ip=self.ip,
            payload_weight=0.624,
            payload_tcp=[-4.15, 5.24, 76.38],
            async_control=False,
            use_internal_ik=True,
            robot_type=robot_type,
            kinematic_model_path=rcs.ROBOTS[robot_type].mjcf_model_path,
            attachment_site=rcs.ROBOTS[robot_type].attachment_site,
            dof=rcs.ROBOTS[robot_type].dof,
            joint_limits=rcs.ROBOTS[robot_type].joint_limits,
            q_home=rcs.ROBOTS[robot_type].q_home,
            tcp_offset=rcs.common.Pose(),
        )

        return XArm7HardwareEnvCreatorConfig(
            control_mode=ControlMode.CARTESIAN_TQuat,
            robot_cfg=robot_cfg,
            calibration_dir=None,
            camera_cfgs=None,
            hand_cfg=None,
            max_relative_movement=0.5,
            relative_to=RelativeTo.LAST_STEP,
        )
