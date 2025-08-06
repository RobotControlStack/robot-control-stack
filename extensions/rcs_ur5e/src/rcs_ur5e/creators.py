import logging

import gymnasium as gym
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

from rcs_ur5e.src.rcs_ur5e.hw import RobtiQGripper, UR5e

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class RCSUR5eEnvCreator(RCSHardwareEnvCreator):
    def __call__(  # type: ignore
        self,
        ip: str,
        urdf_path: str,
        camera_set: HardwareCameraSet | None = None,
        max_relative_movement: float | tuple[float, float] | None = None,
        relative_to: RelativeTo = RelativeTo.LAST_STEP,
    ) -> gym.Env:
        robot = UR5e(ip, urdf_path=urdf_path)
        env: gym.Env = RobotEnv(robot, ControlMode.JOINTS)

        gripper = RobtiQGripper()
        env = GripperWrapper(env, gripper, binary=False)

        if camera_set is not None:
            camera_set.start()
            camera_set.wait_for_frames()
            logger.info("CameraSet started")
            env = CameraSetWrapper(env, camera_set)

        if max_relative_movement is not None:
            env = RelativeActionSpace(env, max_mov=max_relative_movement, relative_to=relative_to)

        return env
