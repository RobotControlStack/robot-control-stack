import logging
from typing import Any, SupportsFloat, cast

import gymnasium as gym
from rcs._core.common import RobotPlatform
from rcs_fr3._core import hw

_logger = logging.getLogger(__name__)


class FR3HW(gym.Wrapper):
    def __init__(self, env):
        super().__init__(env)
        assert self.env.get_wrapper_attr("PLATFORM") == RobotPlatform.HARDWARE, "Base environment must be hardware."
        assert isinstance(self.get_wrapper_attr("robot"), hw.Franka), "Robot must be a hw.Franka instance."
        self.hw_robot = cast(hw.Franka, self.get_wrapper_attr("robot"))
        self._robot_state_keys: list[str] | None = None

    def step(self, action: Any) -> tuple[dict[str, Any], SupportsFloat, bool, bool, dict]:
        try:
            obs, reward, terminated, truncated, info = super().step(action)
            obs = self.get_obs(obs)
            return obs, reward, terminated, truncated, info
        except hw.exceptions.FrankaControlException as e:
            _logger.error("FrankaControlException: %s", e)
            self.hw_robot.automatic_error_recovery()
            return self.get_obs(), 0, False, True, {}

    def get_obs(self, obs: dict | None = None) -> dict[str, Any]:
        if obs is None:
            obs = dict(self.get_wrapper_attr("get_robot_obs")())
        robot_state = cast(hw.FrankaState, self.hw_robot.get_state())
        obs["robot_state"] = self._rs2dict(robot_state.robot_state)
        return obs

    def _rs2dict(self, state: hw.RobotState):
        if self._robot_state_keys is None:
            self._robot_state_keys = [
                attr for attr in dir(state) if not attr.startswith("__") and not callable(getattr(state, attr))
            ]
            self._robot_state_keys.remove("robot_mode")
        return {key: getattr(state, key) for key in self._robot_state_keys}

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        return super().reset(seed=seed, options=options)

    def close(self):
        self.hw_robot.stop_control_thread()
        super().close()
