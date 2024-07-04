from typing import Any, cast

import gymnasium as gym
import numpy as np
import rcsss
from rcsss import sim
from rcsss._core.sim import CameraType
from rcsss.camera.sim import SimCameraConfig, SimCameraSet, SimCameraSetConfig
from rcsss.envs.base import CameraSetWrapper, ControlMode, FR3Env, GripperWrapper, RelativeActionSpace


class FR3Sim(gym.Wrapper):
    def __init__(self, env: FR3Env, simulation: sim.Sim):
        self.env: FR3Env
        super().__init__(env)
        assert isinstance(self.env.robot, sim.FR3), "Robot must be a sim.FR3 instance."
        self.sim_robot = cast(sim.FR3, self.env.robot)
        self.sim = simulation

    def step(self, action: dict[str, Any]) -> tuple[dict[str, Any], float, bool, bool, dict]:
        obs, _, _, _, info = self.env.step(action)
        self.sim.step_until_convergence()
        obs = self.env.get_obs()
        state = self.sim_robot.get_state()
        info["collision"] = state.collision
        info["ik_success"] = state.ik_success
        # truncate episode if collision
        return obs, 0, False, state.collision or not state.ik_success, info

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        self.sim_robot.reset()
        self.sim.step(1)
        return self.env.reset(seed=seed, options=options)


if __name__ == "__main__":
    simulation = sim.Sim("models/mjcf/fr3_modular/scene.xml")
    robot = rcsss.sim.FR3(simulation, "0", "models/fr3/urdf/fr3_from_panda.urdf")
    cfg = sim.FR3Config()
    cfg.ik_duration_in_milliseconds = 300
    cfg.realtime = False
    robot.set_parameters(cfg)
    env = FR3Env(robot, ControlMode.CARTESIAN_TQuart)
    env = RelativeActionSpace(env)
    env_sim = FR3Sim(env, simulation)
    cameras = {
        "birdeye": SimCameraConfig(identifier="eye-in-hand_0", type=CameraType.fixed, on_screen_render=True),
        "default_free": SimCameraConfig(identifier="", type=CameraType.default_free, on_screen_render=True),
    }
    cam_cfg = SimCameraSetConfig(cameras=cameras, resolution_width=640, resolution_height=480, frame_rate=50)
    camera_set = SimCameraSet(simulation, cam_cfg)
    env_cam = CameraSetWrapper(env_sim, camera_set)
    obs, info = env_cam.reset()
    for _ in range(100):
        act = env_cam.action_space.sample()
        obs, reward, terminated, truncated, info = env_cam.step(act)
        if truncated or terminated:
            env_cam.reset()
        print(simulation.data.time)  # noqa: T201
