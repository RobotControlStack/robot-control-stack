from typing import Any, cast

import gymnasium as gym
from rcsss import sim
from rcsss.envs.base import ArmObs, CartOrAngleControl, ControlMode, FR3Env


class FR3Sim(gym.Wrapper):
    def __init__(self, env: FR3Env):
        self.env: FR3Env
        super().__init__(env)
        assert isinstance(self.env.robot, sim.FR3), "Robot must be a sim.FR3 instance."
        self.sim_robot = cast(sim.FR3, self.env.robot)

    def step(self, action: CartOrAngleControl) -> tuple[ArmObs, float, bool, bool, dict]:
        obs, _, _, _, info = self.env.step(action)
        state = self.sim_robot.get_state()
        info["collision"] = state.collision
        info["ik_success"] = state.ik_success
        # truncate episode if collision
        return obs, 0, False, state.collision or not state.ik_success, info

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[ArmObs, dict[str, Any]]:
        # TODO: reset sim
        return self.env.reset(seed, options)


if __name__ == "__main__":
    robot = sim.FR3("models/mjcf/scene.xml", "models/urdf/fr3_from_panda.urdf", render=True)
    cfg = sim.FR3Config()
    cfg.ik_duration = 300
    cfg.realtime = True
    cfg.trajectory_trace = True
    robot.set_parameters(cfg)
    env = FR3Env(robot, ControlMode.ANGLES)
    env_sim = FR3Sim(env)
    obs, info = env_sim.reset()
    for _ in range(100):
        act = env_sim.action_space.sample()
        obs, reward, terminated, truncated, info = env_sim.step(act)
