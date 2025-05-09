import logging
from typing import Any, SupportsFloat, cast

import gymnasium as gym
from rcsss import hw
from rcsss.envs.base import FR3Env

_logger = logging.getLogger(__name__)


class FR3HW(gym.Wrapper):
    def __init__(self, env):
        super().__init__(env)
        self.unwrapped: FR3Env
        assert isinstance(self.unwrapped.robot, hw.FR3), "Robot must be a hw.FR3 instance."
        self.hw_robot = cast(hw.FR3, self.unwrapped.robot)

    def step(self, action: Any) -> tuple[dict[str, Any], SupportsFloat, bool, bool, dict]:
        try:
            return super().step(action)
        except hw.exceptions.FrankaControlException as e:
            _logger.error("FrankaControlException: %s", e)
            self.hw_robot.automatic_error_recovery()
            # TODO: this does not work if some wrappers are in between
            # FR3HW and FR3Env
            return dict(self.unwrapped.get_obs()), 0, False, True, {}

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        return super().reset(seed=seed, options=options)

    def close(self):
        self.hw_robot.stop_control_thread()
        super().close()
