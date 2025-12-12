import logging
from typing import Any, SupportsFloat, cast

import gymnasium as gym
from rcs.envs.base import RobotEnv
from rcs_fr3._core import hw

_logger = logging.getLogger(__name__)


class FR3HW(gym.Wrapper):
    def __init__(self, env):
        super().__init__(env)
        self.unwrapped: RobotEnv
        assert isinstance(self.unwrapped.robot, hw.Franka), "Robot must be a hw.Franka instance."
        self.hw_robot = cast(hw.Franka, self.unwrapped.robot)

    def step(self, action: Any) -> tuple[dict[str, Any], SupportsFloat, bool, bool, dict]:
        try:
            obs, reward, terminated, truncated, info = super().step(action)
            obs = self.get_obs(obs)
            return obs, reward, terminated, truncated, info
        except hw.exceptions.FrankaControlException as e:
            _logger.error("FrankaControlException: %s", e)
            self.hw_robot.automatic_error_recovery()
            # TODO: this does not work if some wrappers are in between
            # FR3HW and RobotEnv
            return self.get_obs(), 0, False, True, {}

    def get_obs(self, obs: dict | None = None) -> dict[str, Any]:
        if obs is None:
            obs = dict(self.unwrapped.get_obs())
        robot_state = cast(hw.FrankaState, self.unwrapped.robot.get_state())
        obs["robot_state"] = vars(robot_state.robot_state)
        return obs

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        return super().reset(seed=seed, options=options)

    def close(self):
        self.hw_robot.stop_control_thread()
        super().close()
