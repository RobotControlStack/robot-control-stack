import logging
from pathlib import Path

from rcs.envs.base import ControlMode, CoverWrapper, MultiRobotWrapper
from rcs.operator.interface import TeleopLoop
from rcs.operator.so101 import SO101Operator, SO101OperatorConfig
from rcs_so101 import RCSSO101ConfigEnvCreator, SO101Config

import rcs

logger = logging.getLogger(__name__)

FOLLOWER_PORT = "/dev/ttyACM0"
LEADER_PORT = "/dev/ttyACM1"
FOLLOWER_CALIBRATION_DIR = Path(".cache/so101/follower")
LEADER_CALIBRATION_DIR = Path(".cache/so101/leader")
ROBOT_NAME = "so101"


def get_env():
    robot_type = rcs.common.RobotType("SO101")
    robot_meta = rcs.ROBOTS[robot_type]
    robot_cfg = SO101Config(
        id="follower",
        port=FOLLOWER_PORT,
        calibration_dir=str(FOLLOWER_CALIBRATION_DIR),
        robot_type=robot_type,
        attachment_site=robot_meta.attachment_site,
        kinematic_model_path=robot_meta.mjcf_model_path,
        dof=robot_meta.dof,
        joint_limits=robot_meta.joint_limits,
        q_home=robot_meta.q_home,
        tcp_offset=rcs.common.Pose(),
    )
    env = RCSSO101ConfigEnvCreator()(
        robot_cfg=robot_cfg,
        control_mode=ControlMode.JOINTS,
        max_relative_movement=None,
        relative_to=SO101Operator.control_mode[1],
    )
    return CoverWrapper(MultiRobotWrapper({ROBOT_NAME: env}))


def main():
    env = get_env()
    operator = SO101Operator(
        SO101OperatorConfig(
            controller_name=ROBOT_NAME,
            id="leader",
            port=LEADER_PORT,
            calibration_dir=str(LEADER_CALIBRATION_DIR),
            use_degrees=True,
        )
    )
    tele = TeleopLoop(env, operator)
    with env, tele:  # type: ignore
        tele.environment_step_loop()


if __name__ == "__main__":
    main()
