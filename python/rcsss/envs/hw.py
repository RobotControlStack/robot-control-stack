import logging
from typing import Any, SupportsFloat, cast

import gymnasium as gym
import rcsss
from rcsss import hw
from rcsss.config import read_config_yaml
from rcsss.desk import Desk
from rcsss.envs.base import ControlMode, FR3Env, GripperWrapper, RelativeActionSpace
from rcsss.envs.sim import CollisionGuard

from python.rcsss.desk import FCI

_logger = logging.getLogger(__name__)


class FR3HW(gym.Wrapper):
    def __init__(self, env):
        super().__init__(env)
        self.unwrapped: FR3Env
        assert isinstance(self.unwrapped.robot, hw.FR3), "Robot must be a hw.FR3 instance."
        self.hw_robot = cast(hw.FR3, self.unwrapped.robot)

    def step(self, action: Any) -> tuple[Any, SupportsFloat, bool, bool, dict]:
        try:
            return super().step(action)
        except hw.exceptions.FrankaControlException as e:
            _logger.error("FrankaControlException: %s", e)
            self.hw_robot.automatic_error_recovery()
            # TODO: this does not work if some wrappers are in between
            # FR3HW and FR3Env
            return self.unwrapped.get_obs(), 0, False, True, {}

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        self.hw_robot.move_home()
        return super().reset(seed=seed, options=options)


if __name__ == "__main__":
    cfg = read_config_yaml("config.yaml")
    d = Desk("192.168.101.1", cfg.hw.username, cfg.hw.password)
    with FCI(d, unlock=True, lock_when_done=True):
        logger = logging.getLogger(__name__)
        robot = hw.FR3("192.168.101.1", str(rcsss.scenes["lab"].parent / "fr3.urdf"))
        env = FR3Env(robot, ControlMode.JOINTS)
        env_hw = FR3HW(env)
        gripper_cfg = hw.FHConfig()
        gripper_cfg.epsilon_inner = gripper_cfg.epsilon_outer = 0.1
        gripper_cfg.speed = 0.1
        gripper_cfg.force = 10
        gripper = hw.FrankaHand("192.168.101.1", gripper_cfg)
        env_hw = GripperWrapper(env_hw, gripper)

        env_cam = CollisionGuard.env_from_xml_paths(
            env_hw,
            rcsss.scenes["fr3_empty_world"],
            str(rcsss.scenes["lab"].parent / "fr3.urdf"),
            gripper=True,
            check_home_collision=False,
            camera=True,
        )
        env_cam = RelativeActionSpace(env_cam)
        while True:
            robot.move_home()
            obs, info = env_cam.reset()
            for _ in range(10):
                act = env_cam.action_space.sample()
                obs, reward, terminated, truncated, info = env_cam.step(act)
                if truncated or terminated:
                    logger.info("Truncated or terminated!")
                    env_cam.reset()
                logger.info(act["gripper"], obs["gripper"])
