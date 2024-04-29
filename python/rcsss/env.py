"""Gym API."""

from collections import OrderedDict
from typing import Any, Optional, cast

import gymnasium as gym
import mujoco
import numpy as np
from rcsss import common, sim

RPY = gym.spaces.Box(low=np.deg2rad(-180), high=np.deg2rad(180), shape=(3,))
XYZ = gym.spaces.Box(low=np.array([-855, -855, 0]), high=np.array([855, 855, 1188]), shape=(3,))
POSE = gym.spaces.Dict({"rpy": RPY, "xyz": XYZ})
ANGLES = gym.spaces.Box(
    low=np.array([-2.3093, -1.5133, -2.4937, -2.7478, -2.4800, 0.8521, -2.6895]),
    high=np.array([2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094, 2.6895]),
    dtype=np.float32,
    shape=(7,),
)
COLLISION = IK_SUCCESS = gym.spaces.Discrete(2)


# TODO: Typing for the action space?
# cf. https://github.com/Farama-Foundation/Gymnasium/issues/845
# and https://gymnasium.farama.org/_modules/gymnasium/core/
class FR3Base(gym.Env):
    """Simulated Franka Research 3."""

    def __init__(self, robot: common.Robot):
        """Parameters: path to MJCF and path to urdf."""
        self.robot = robot
        self.action_space = ANGLES
        self.observation_space = gym.spaces.Dict(
            {
                "pose": POSE,
                "angles": ANGLES,
                "collision": COLLISION,
            }
        )

    def _get_obs(self):
        state = cast(sim.FR3State, self.robot.get_state())
        pose = self.robot.get_cartesian_position()
        rpy = pose.rotation_rpy()
        xyz = pose.translation()
        return OrderedDict(
            pose=OrderedDict(rpy=np.array([rpy.roll, rpy.pitch, rpy.yaw]), xyz=np.array(xyz)),
            angles=self.robot.get_joint_position(),
            collision=state.collision,
        )

    def step(self, act: Any) -> tuple[Any, float, bool, bool, dict]:
        self.robot.set_joint_position(act)
        return self._get_obs(), 0, False, False, {}

    def reset(self, seed: Optional[int] = None, options: Optional[dict[str, Any]] = None) -> tuple[Any, dict[str, Any]]:
        if seed is not None:
            msg = "seeding not implemented yet"
            raise NotImplementedError(msg)
        if options is not None:
            msg = "options not implemented yet"
            raise NotImplementedError(msg)
        self.robot.move_home()
        return self._get_obs()


if __name__ == "__main__":
    robot = sim.FR3("models/mjcf/scene.xml", "models/urdf/fr3_from_panda.urdf")
    cfg = sim.FR3Config()
    cfg.ik_duration = 300
    cfg.realtime = True
    cfg.trajectory_trace = True
    robot.set_parameters(cfg)
    env = FR3Base(robot)
    obs = env.reset()
    for _ in range(100):
        act = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(act)
