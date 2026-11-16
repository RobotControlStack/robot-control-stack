import logging
from typing import Any, SupportsFloat, cast

import gymnasium as gym
from rcs._core.common import RobotPlatform
from rcs_fr3 import hw

_logger = logging.getLogger(__name__)


class FR3HW(gym.Wrapper):
    """Hardware specific additions for the FR3.

    - adds the libfranka robot state to the observation (``obs["robot_state"]``)
    - if ``record_control_states`` is enabled in the robot config, all states seen by the 1 kHz
      control loop since the last step are added to ``info["control_states"]`` as one array per
      signal (shape N or N x D) together with ``num_samples`` and ``dropped``
    - a robot error (e.g. a reflex) ends the episode visibly: the step returns ``truncated=True``
      and the error message in ``info["robot_error"]`` (empty string if there is no error).
      The robot is not recovered silently in the step, the recovery happens on the next reset.
    """

    CONTROL_STATES_KEY = "control_states"
    ROBOT_ERROR_KEY = "robot_error"

    def __init__(self, env):
        super().__init__(env)
        assert self.env.get_wrapper_attr("PLATFORM") == RobotPlatform.HARDWARE, "Base environment must be hardware."
        assert isinstance(self.get_wrapper_attr("robot"), hw.Franka), "Robot must be a hw.Franka instance."
        self.hw_robot = cast(hw.Franka, self.get_wrapper_attr("robot"))
        self._robot_state_keys: list[str] | None = None
        self._record_control_states = bool(self.hw_robot.get_config().record_control_states)
        self._last_obs: dict[str, Any] | None = None

    def _add_control_states(self, info: dict[str, Any]) -> dict[str, Any]:
        if self._record_control_states:
            info[self.CONTROL_STATES_KEY] = self.hw_robot.drain_control_states()
        return info

    def step(self, action: Any) -> tuple[dict[str, Any], SupportsFloat, bool, bool, dict]:
        try:
            obs, reward, terminated, truncated, info = super().step(action)
            obs = self.get_obs(obs)
            info[self.ROBOT_ERROR_KEY] = ""
            return obs, reward, terminated, truncated, self._add_control_states(info)
        except hw.exceptions.FrankaControlException as e:
            _logger.error("FrankaControlException, truncating the episode: %s", e)
            try:
                obs = self.get_obs()
            except Exception:
                _logger.exception("could not read the robot state after the control exception")
                obs = self._last_obs if self._last_obs is not None else {}
            info = {
                "platform": "hardware",
                "robot_type": self.hw_robot.get_config().robot_type.id,
                self.ROBOT_ERROR_KEY: str(e),
            }
            return obs, 0, False, True, self._add_control_states(info)

    def recover(self) -> None:
        """Clears a robot error (e.g. a reflex) without resetting the env: the robot stays where it stopped and the
        next action starts the controller again from the measured state. Raises if the robot can not recover.

        The stored control-thread exception is already cleared when it was raised in the step
        (Franka::check_for_background_errors loads and clears it and stops the control thread)."""
        self.hw_robot.automatic_error_recovery()

    def get_obs(self, obs: dict | None = None) -> dict[str, Any]:
        if obs is None:
            obs = dict(self.get_wrapper_attr("get_robot_obs")())
        robot_state = cast(hw.FrankaState, self.hw_robot.get_state())
        obs["robot_state"] = self._rs2dict(robot_state.robot_state)
        self._last_obs = obs
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
        obs, info = super().reset(seed=seed, options=options)
        # the config might have changed, states recorded before and during the reset do not belong to the next episode
        self._record_control_states = bool(self.hw_robot.get_config().record_control_states)
        self.hw_robot.clear_control_states()
        info[self.ROBOT_ERROR_KEY] = ""
        return self.get_obs(obs), info

    def close(self):
        self.hw_robot.stop_control_thread()
        super().close()
