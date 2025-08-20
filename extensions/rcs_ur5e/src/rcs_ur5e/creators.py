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
from rcs_ur5e.hw import RobotiQGripper, UR5e, UR5eConfig

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class RCSUR5eEnvCreator(RCSHardwareEnvCreator):
    def __call__(  # type: ignore
        self,
        ip: str,
        robot_cfg: UR5eConfig,
        camera_set: HardwareCameraSet | None = None,
        control_mode: ControlMode = ControlMode.CARTESIAN_TRPY,
        max_relative_movement: float | tuple[float, float] | None = None,
        relative_to: RelativeTo = RelativeTo.LAST_STEP,
    ) -> gym.Env:
        robot = UR5e(ip)
        robot.set_parameters(robot_cfg)
        env: gym.Env = RobotEnv(robot, control_mode, home_on_reset=True)

        gripper = RobotiQGripper(ip)
        env = GripperWrapper(env, gripper, binary=True)

        if camera_set is not None:
            camera_set.start()
            camera_set.wait_for_frames()
            logger.info("CameraSet started")
            env = CameraSetWrapper(env, camera_set)

        if max_relative_movement is not None:
            env = RelativeActionSpace(env, max_mov=max_relative_movement, relative_to=relative_to)

        return env
