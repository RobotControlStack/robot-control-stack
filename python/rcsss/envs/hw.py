import logging
from typing import Any, cast

import gymnasium as gym
from rcsss import hw
from rcsss.envs.base import ArmObs, CartOrAngleControl, FR3Env

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
