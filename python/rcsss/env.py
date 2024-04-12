"""Gym API."""

from collections import OrderedDict
from typing import Any, Optional, cast

import gymnasium as gym
import numpy as np
from rcsss import sim

RPY = gym.spaces.Box(low=np.deg2rad(-180), high=np.deg2rad(180), shape=(3,))
xyz = gym.spaces.Box(low=np.array(
    [-855, -855, 0]), high=np.array([855, 855, 1188]), shape=(3,))
pose = gym.spaces.Dict({"rpy": RPY, "xyz": xyz})
angles = gym.spaces.Box(
    low=np.array([-2.3093, -1.5133, -2.4937, -
                 2.7478, -2.4800, 0.8521, -2.6895]),
    high=np.array([2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094, 2.6895]),
    dtype=np.float32,
    shape=(7,),
)
collision = ik_success = gym.spaces.Discrete(2)


# TODO: Typing for the action space?
# cf. https://github.com/Farama-Foundation/Gymnasium/issues/845
# and https://gymnasium.farama.org/_modules/gymnasium/core/
# Disable the missing docstring warning, base class already has a docstring
class FR3Base(gym.Env):
    """Simulated Franka Research 3."""

    def __init__(self, mjcf: str, urdf: str, render: bool = True):
        """Parameters: path to MJCF and path to urdf."""
        self.robot = sim.FR3(mjcf, urdf, render)
        self.action_space = angles
        self.observation_space = gym.spaces.Dict(
            {
                "pose": pose,
                "angles": angles,
                "collision": collision,
            }
        )

    def _get_obs(self):
        state = cast(sim.FR3State, self.robot.get_state())
        pose = self.robot.get_cartesian_position()
        rpy = pose.rotation_rpy()
        xyz = pose.translation()
        return OrderedDict(
            pose=OrderedDict(rpy=np.array(
                [rpy.roll, rpy.pitch, rpy.yaw]), xyz=np.array(xyz)),
            angles=self.robot.get_joint_position(),
            collision=state.collision,
        )

    def step(self, act: Any) -> tuple[Any, float, bool, bool, dict]:
        self.robot.set_joint_position(act)
        return self._get_obs(), 0, False, False, {}

    def reset(
        self, seed: Optional[int] = None, options: Optional[dict[str, Any]] = None
    ) -> tuple[Any, dict[str, Any]]:
        if seed is not None:
            msg = "seeding not implemented yet"
            raise NotImplementedError(msg)
        if options is not None:
            msg = "options not implemented yet"
            raise NotImplementedError(msg)
        self.robot.reset()
        return self._get_obs()


if __name__ == "__main__":
    env = FR3Base("models/mjcf/scene.xml",
                  "models/urdf/fr3_from_panda.urdf", render=False)
    obs = env.reset()
    # print(f"Initial obs: {obs}")
    act = env.action_space.sample()
    # print(f"Initial action: {act}")
    obs, reward, terminated, truncated, info = env.step(
        env.action_space.sample())
    # print(f"New obs: {obs}")
