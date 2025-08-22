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
from rcs_so101.hw import SO101, SO101Config, SO101Gripper

import rcs

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class RCSSO101EnvCreator(RCSHardwareEnvCreator):
    def __call__(  # type: ignore
        self,
        robot_cfg: SO101Config,
        control_mode: ControlMode,
        camera_set: HardwareCameraSet | None = None,
        max_relative_movement: float | tuple[float, float] | None = None,
        relative_to: RelativeTo = RelativeTo.LAST_STEP,
    ) -> gym.Env:
        ik = rcs.common.Pin(
            robot_cfg.kinematic_model_path,
            robot_cfg.attachment_site,
            urdf=robot_cfg.kinematic_model_path.endswith(".urdf"),
        )
        robot = SO101(robot_cfg=robot_cfg, ik=ik)
        env: gym.Env = RobotEnv(robot, control_mode, home_on_reset=True)

        gripper = SO101Gripper(robot.hf_robot, robot)
        env = GripperWrapper(env, gripper, binary=False)

        if camera_set is not None:
            camera_set.start()
            camera_set.wait_for_frames()
            logger.info("CameraSet started")
            env = CameraSetWrapper(env, camera_set)

        if max_relative_movement is not None:
            env = RelativeActionSpace(env, max_mov=max_relative_movement, relative_to=relative_to)

        return env

    # @staticmethod
    # def teleoperator(
    #     id: str,
    #     port: str,
    #     calibration_dir: PathLike | str | None = None,
    # ) -> SO101Leader:
    #     if isinstance(calibration_dir, str):
    #         calibration_dir = Path(calibration_dir)
    #     cfg = SO101LeaderConfig(id=id, calibration_dir=calibration_dir, port=port)
    #     teleop = make_teleoperator_from_config(cfg)
    #     teleop.connect()
    #     return teleop
