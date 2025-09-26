import logging
from os import PathLike
from pathlib import Path

import gymnasium as gym
from rcs.camera.hw import HardwareCameraSet
from rcs.envs.base import (
    CameraSetWrapper,
    ControlMode,
    HandWrapper,
    RelativeActionSpace,
    RelativeTo,
    RobotEnv,
)
from rcs.envs.creators import RCSHardwareEnvCreator
from rcs.hand.tilburg_hand import THConfig, TilburgHand
from rcs_xarm7.hw import XArm7

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class RCSXArm7EnvCreator(RCSHardwareEnvCreator):
    def __call__(  # type: ignore
        self,
        control_mode: ControlMode,
        ip: str,
        calibration_dir: PathLike | str | None = None,
        camera_set: HardwareCameraSet | None = None,
        hand_cfg: THConfig | None = None,
        max_relative_movement: float | tuple[float, float] | None = None,
        relative_to: RelativeTo = RelativeTo.LAST_STEP,
    ) -> gym.Env:
        if isinstance(calibration_dir, str):
            calibration_dir = Path(calibration_dir)
        robot = XArm7(ip=ip)
        env: gym.Env = RobotEnv(robot, control_mode, home_on_reset=True)

        if camera_set is not None:
            camera_set.start()
            camera_set.wait_for_frames()
            logger.info("CameraSet started")
            env = CameraSetWrapper(env, camera_set, include_depth=True)
        if hand_cfg is not None and isinstance(hand_cfg, THConfig):
            hand = TilburgHand(cfg=hand_cfg, verbose=True)
            env = HandWrapper(env, hand, True)

        if max_relative_movement is not None:
            env = RelativeActionSpace(env, max_mov=max_relative_movement, relative_to=relative_to)

        return env
