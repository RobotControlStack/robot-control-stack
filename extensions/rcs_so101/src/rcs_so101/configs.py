from rcs._core.common import RobotType
from rcs.envs.base import ControlMode, RelativeTo
from rcs_so101.creators import RCSSO101ConfigEnvCreator, SO101HardwareEnvCreatorConfig
from rcs_so101.hw import SO101Config

import rcs


class DefaultSO101HardwareEnv(RCSSO101ConfigEnvCreator):
    id = "follower"
    port = "/dev/ttyACM0"
    calibration_dir = "."

    def config(self) -> SO101HardwareEnvCreatorConfig:
        robot_cfg = SO101Config(
            id=self.id,
            port=self.port,
            calibration_dir=self.calibration_dir,
            robot_type=RobotType("SO101"),
            kinematic_model_path=rcs.ROBOTS[RobotType("SO101")].mjcf_model_path,
            attachment_site=rcs.ROBOTS[RobotType("SO101")].attachment_site,
            dof=rcs.ROBOTS[RobotType("SO101")].dof,
            joint_limits=rcs.ROBOTS[RobotType("SO101")].joint_limits,
            q_home=rcs.ROBOTS[RobotType("SO101")].q_home,
            tcp_offset=rcs.common.Pose(),
        )

        return SO101HardwareEnvCreatorConfig(
            control_mode=ControlMode.JOINTS,
            robot_cfg=robot_cfg,
            camera_cfgs=None,
            max_relative_movement=None,
            relative_to=RelativeTo.LAST_STEP,
        )
