import logging
from os import PathLike
from pathlib import Path

import gymnasium as gym
from lerobot.common.robots import make_robot_from_config
from lerobot.common.robots.so101_follower.config_so101_follower import (
    SO101FollowerConfig,
)
from lerobot.common.teleoperators.so101_leader.config_so101_leader import (
    SO101LeaderConfig,
)
from lerobot.common.teleoperators.so101_leader.so101_leader import SO101Leader
from lerobot.common.teleoperators.utils import make_teleoperator_from_config
from rcs.camera.hw import HardwareCameraSet
from rcs.envs.base import (
    CameraSetWrapper,
    ControlMode,
    GripperWrapper,
    RelativeActionSpace,
    RelativeTo,
    RobotEnv,
)
from rcs.envs.creators import RCSHardwareEnvCreator
from rcs_so101.hw import SO101, S0101Gripper

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class RCSSO101EnvCreator(RCSHardwareEnvCreator):
    def __call__(  # type: ignore
        self,
        id: str,
        port: str,
        urdf_path: str,
        calibration_dir: PathLike | str | None = None,
        camera_set: HardwareCameraSet | None = None,
        max_relative_movement: float | tuple[float, float] | None = None,
        relative_to: RelativeTo = RelativeTo.LAST_STEP,
    ) -> gym.Env:
        if isinstance(calibration_dir, str):
            calibration_dir = Path(calibration_dir)
        cfg = SO101FollowerConfig(id=id, calibration_dir=calibration_dir, port=port)
        hf_robot = make_robot_from_config(cfg)
        robot = SO101(hf_robot)  # TODO add path
        env: gym.Env = RobotEnv(robot, ControlMode.JOINTS)
        # env = FR3HW(env)

        gripper = S0101Gripper(hf_robot)
        env = GripperWrapper(env, gripper, binary=False)

        if camera_set is not None:
            camera_set.start()
            camera_set.wait_for_frames()
            logger.info("CameraSet started")
            env = CameraSetWrapper(env, camera_set)

        if max_relative_movement is not None:
            env = RelativeActionSpace(env, max_mov=max_relative_movement, relative_to=relative_to)

        return env

    @staticmethod
    def teleoperator(
        id: str,
        port: str,
        calibration_dir: PathLike | str | None = None,
    ) -> SO101Leader:
        if isinstance(calibration_dir, str):
            calibration_dir = Path(calibration_dir)
        cfg = SO101LeaderConfig(id=id, calibration_dir=calibration_dir, port=port)
        teleop = make_teleoperator_from_config(cfg)
        teleop.connect()
        return teleop
