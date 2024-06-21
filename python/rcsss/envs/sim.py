from typing import Any, cast

import gymnasium as gym
import rcsss
from rcsss import sim
from rcsss.camera.sim import SimCameraConfig, SimCameraSet
from rcsss.envs.base import ArmObs, CameraSetWrapper, CartOrAngleControl, ControlMode, FR3Env


class FR3Sim(gym.Wrapper):
    def __init__(self, env: FR3Env, simulation: sim.Sim):
        self.env: FR3Env
        super().__init__(env)
        assert isinstance(self.env.robot, sim.FR3), "Robot must be a sim.FR3 instance."
        self.sim_robot = cast(sim.FR3, self.env.robot)
        self.sim = simulation

    def step(self, action: CartOrAngleControl) -> tuple[ArmObs, float, bool, bool, dict]:
        obs, _, _, _, info = self.env.step(action)
        self.sim.step_until_convergence()
        state = self.sim_robot.get_state()
        info["collision"] = state.collision
        info["ik_success"] = state.ik_success
        # truncate episode if collision
        return obs, 0, False, state.collision or not state.ik_success, info

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[ArmObs, dict[str, Any]]:
        # TODO: reset sim
        return self.env.reset(seed, options)


if __name__ == "__main__":
    simulation = sim.Sim("models/mjcf/fr3_modular/scene.xml")
    robot = rcsss.sim.FR3(simulation, "0", "models/fr3/urdf/fr3_from_panda.urdf")
    cfg = sim.FR3Config()
    cfg.ik_duration_in_milliseconds = 300
    cfg.realtime = False
    robot.set_parameters(cfg)
    env = FR3Env(robot, ControlMode.CARTESIAN)
    env_sim = FR3Sim(env, simulation)
    cam_cfg = SimCameraConfig(camera2id={"birdeye-camera": "birdeye-camera"})
    camera_set = SimCameraSet(simulation, cam_cfg)
    env_cam = CameraSetWrapper(env, camera_set)
    # TODO: test env_cam
    obs, info = env_sim.reset()
    for _ in range(100):
        act = env_sim.action_space.sample()
        obs, reward, terminated, truncated, info = env_sim.step(act)
        print(act, obs, info) # noqa: T201
