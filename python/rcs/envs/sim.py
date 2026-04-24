import logging
from typing import Any, cast

import gymnasium as gym
from rcs._core.common import RobotPlatform
from rcs.envs.space_utils import ActObsInfoWrapper

from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class RobotSimWrapper(ActObsInfoWrapper):
    def __init__(self, env):
        super().__init__(env)
        assert self.env.get_wrapper_attr("PLATFORM") == RobotPlatform.SIMULATION, "Base environment must be simulation."
        assert isinstance(self.get_wrapper_attr("robot"), sim.SimRobot), "Robot must be a sim.SimRobot instance."
        self.sim_robot = cast(sim.SimRobot, self.get_wrapper_attr("robot"))
        self.sim = cast(sim.Sim, self.get_wrapper_attr("sim"))

    def action(self, action: dict[str, Any]) -> dict[str, Any]:
        self.sim_robot.clear_collision_flag()
        return action

    def observation(self, observation: dict, info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        state = self.sim_robot.get_state()
        if "collision" not in info:
            info["collision"] = state.collision
        else:
            info["collision"] = info["collision"] or state.collision
        info["ik_success"] = state.ik_success
        info["is_sim_converged"] = self.sim.is_converged()
        return observation, info

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        self.sim_robot.clear_collision_flag()
        return super().reset(seed=seed, options=options)


class GripperWrapperSim(ActObsInfoWrapper):
    def __init__(self, env):
        super().__init__(env)
        assert self.env.get_wrapper_attr("PLATFORM") == RobotPlatform.SIMULATION, "Base environment must be simulation."
        assert isinstance(
            self.get_wrapper_attr("gripper"), sim.SimGripper
        ), "Gripper must be a sim.SimGripper instance."
        self._gripper = cast(sim.SimGripper, self.get_wrapper_attr("gripper"))

    def action(self, action: dict[str, Any]) -> dict[str, Any]:
        self._gripper.clear_collision_flag()
        return action

    def observation(self, observation: dict[str, Any], info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        state = self._gripper.get_state()
        if "collision" not in info or not info["collision"]:
            info["collision"] = state.collision
        info["gripper_width"] = self._gripper.get_normalized_width()
        info["is_grasped"] = self._gripper.get_normalized_width() > 0.01 and self._gripper.get_normalized_width() < 0.99
        return observation, info

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        self._gripper.clear_collision_flag()
        return super().reset(seed=seed, options=options)


class HandWrapperSim(ActObsInfoWrapper):
    def __init__(self, env):
        super().__init__(env)
        assert self.env.get_wrapper_attr("PLATFORM") == RobotPlatform.SIMULATION, "Base environment must be simulation."
        assert isinstance(
            self.get_wrapper_attr("hand"), sim.SimTilburgHand
        ), "Hand must be a sim.SimTilburgHand instance."
        self._hand = cast(sim.SimTilburgHand, self.get_wrapper_attr("hand"))

    def action(self, action: dict[str, Any]) -> dict[str, Any]:
        if isinstance(action["hand"], int | float):
            return action
        if len(action["hand"]) == 18:
            action["hand"] = action["hand"][:16]
        assert len(action["hand"]) == 16 or len(action["hand"]) == 1, "Hand action must be of length 16 or 1"
        return action

    def observation(self, observation: dict[str, Any], info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        state = self._hand.get_state()
        if "collision" not in info or not info["collision"]:
            info["collision"] = state.collision
        info["hand_position"] = self._hand.get_normalized_joint_poses()
        # info["is_grasped"] = self._hand.get_normalized_joint_poses() > 0.01 and self._hand.get_normalized_joint_poses() < 0.99
        return observation, info


class DigitalTwin(gym.Wrapper):
    def __init__(self, env, twin_env):
        super().__init__(env)
        self.twin_env = twin_env

    def step(self, action):
        obs, reward, terminated, truncated, info = super().step(action)

        twin_obs, _, _, _, _ = self.twin_env.step(obs)
        info["twin_obs"] = twin_obs
        return obs, reward, terminated, truncated, info
