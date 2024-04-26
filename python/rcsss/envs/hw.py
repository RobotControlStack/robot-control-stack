import logging
from typing import Any, cast

import gymnasium as gym
from rcsss import hw
from rcsss.camera.kinect import KinectCamera, KinectConfig
from rcsss.envs.base import (
    ARM_OBS_SPACE,
    FRAME_SPACE,
    ArmObs,
    CameraDictType,
    CartOrAngleControl,
    FR3Env,
    FrameDictType,
    ImuDictType,
)

_logger = logging.getLogger(__name__)


class FR3HW(gym.Wrapper):
    def __init__(self, env: FR3Env):
        self.env: FR3Env
        super().__init__(env)
        assert isinstance(self.env.robot, hw.FR3), "Robot must be a hw.FR3 instance."
        self.hw_robot = cast(hw.FR3, self.env.robot)

    def step(self, action: CartOrAngleControl) -> tuple[ArmObs, float, bool, bool, dict]:
        try:
            return self.env.step(action)
        except hw.exceptions.FrankaControlException as e:
            _logger.error("FrankaControlException: %s", e)
            self.hw_robot.automatic_error_recovery()
            return self.env._get_obs(), 0, False, True, {}

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[ArmObs, dict[str, Any]]:
        self.hw_robot.move_home()
        return self.env.reset(seed, options)


KINECT_OBS_SPACE = gym.spaces.Dict(
    {
        "kinect_frame": FRAME_SPACE,
    }
)
KINECT_OBS_SPACE.spaces.update(ARM_OBS_SPACE)


class KinectObsType(ArmObs):
    kinect_frame: FrameDictType


class KinectWrapper(gym.Wrapper[KinectObsType, CartOrAngleControl, ArmObs, CartOrAngleControl]):
    def __init__(self, env: FR3HW):
        self.env: FR3HW
        self.observation_space: gym.spaces.Space
        super().__init__(env)
        config = KinectConfig()
        self.camera = KinectCamera(config)
        self.observation_space = KINECT_OBS_SPACE

    def step(self, action: CartOrAngleControl) -> tuple[KinectObsType, float, bool, bool, dict]:
        obs, reward, terminated, truncated, info = self.env.step(action)
        frame = self.camera.get_current_frame()
        kinect_frame = FrameDictType(
            camera=CameraDictType(
                color=frame.camera.color,
                ir=frame.camera.ir,
                depth=frame.camera.depth,
            ),
            imu=(
                ImuDictType(
                    acc_sample=frame.imu.acc_sample,
                    gyro_sample=frame.imu.gyro_sample,
                )
                if frame.imu is not None
                else None
            ),
        )
        kinect_obs = KinectObsType(kinect_frame=kinect_frame, angles=obs["angles"], pose=obs["pose"])
        info["camera_temperature"] = frame.camera.temperature
        if frame.imu is not None:
            info["imu_acc_sample_usec"] = frame.imu.acc_sample_usec
            info["imu_gyro_sample_usec"] = frame.imu.gyro_sample_usec
            info["imu_temperature"] = frame.imu.temperature
        return kinect_obs, reward, terminated, truncated, info
