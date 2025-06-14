import logging
from rcs.camera.hw import HardwareCameraSet
from rcs.envs.base import CameraSetWrapper, ControlMode, GripperWrapper, RelativeActionSpace, RelativeTo, RobotEnv
from rcs.envs.creators import RCSHardwareEnvCreator
from rcs.envs.hw import FR3HW
import rcs.robots.so101

from lerobot.common.robots import (  # noqa: F401
    make_robot_from_config,
)
from lerobot.common.robots.so101_follower.config_so101_follower import SO101FollowerConfig
import gymnasium as gym

from lerobot.common.teleoperators.so101_leader.config_so101_leader import SO101LeaderConfig
from lerobot.common.teleoperators.so101_leader.so101_leader import SO101Leader
from lerobot.common.teleoperators.utils import make_teleoperator_from_config

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class RCSSO101EnvCreator(RCSHardwareEnvCreator):
    def __call__(  # type: ignore
        self,
        id: str,
        port: str,
        calibration_dir: str | None = None,
        camera_set: HardwareCameraSet | None = None,
        max_relative_movement: float | tuple[float, float] | None = None,
        relative_to: RelativeTo = RelativeTo.LAST_STEP,
    ) -> gym.Env:
        cfg = SO101FollowerConfig(id, calibration_dir=calibration_dir, port=port)
        hf_robot = make_robot_from_config(cfg)
        robot = rcs.robots.so101.SO101(hf_robot)
        env: gym.Env = RobotEnv(robot, ControlMode.JOINTS)
        env = FR3HW(env)

        gripper = rcs.robots.so101.S0101Gripper(hf_robot)
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
        calibration_dir: str | None = None,
    ) -> SO101Leader:
        cfg = SO101LeaderConfig(id, calibration_dir=calibration_dir, port=port)
        return make_teleoperator_from_config(cfg)
