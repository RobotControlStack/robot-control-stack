import logging
import sys
from typing import Any, SupportsFloat, cast

import gymnasium as gym
import rcsss
from rcsss import sim
from rcsss._core.sim import CameraType
from rcsss.camera.sim import SimCameraConfig, SimCameraSet, SimCameraSetConfig
from rcsss.envs.base import (
    CameraSetWrapper,
    ControlMode,
    FR3Env,
    GripperWrapper,
    RelativeActionSpace,
)


class FR3Sim(gym.Wrapper):
    def __init__(self, env, simulation: sim.Sim):
        super().__init__(env)
        self.unwrapped: FR3Env
        assert isinstance(self.unwrapped.robot, sim.FR3), "Robot must be a sim.FR3 instance."
        self.sim_robot = cast(sim.FR3, self.unwrapped.robot)
        self.sim = simulation

    def step(self, action: dict[str, Any]) -> tuple[dict[str, Any], float, bool, bool, dict]:
        obs, _, _, _, info = super().step(action)
        self.sim.step_until_convergence()
        state = self.sim_robot.get_state()
        info["collision"] = state.collision
        info["ik_success"] = state.ik_success
        info["is_sim_converged"] = self.sim.is_converged()
        # truncate episode if collision
        return obs, 0, False, state.collision or not state.ik_success, info

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        self.sim_robot.reset()
        self.sim.step(1)
        return super().reset(seed=seed, options=options)


class CollisionGuard(gym.Wrapper[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any]]):
    """
    - Gripper Wrapper has to be added before this as it removes the gripper action
    - RelativeActionSpace has to be added after this as it changes the input space, and the input expects absolute actions
    """

    def __init__(self, env: gym.Env, simulation: sim.Sim, collision_env: FR3Sim, check_home_collision: bool = True):
        super().__init__(env)
        self.unwrapped: FR3Env
        self.collision_env = collision_env
        self.sim = simulation
        self.last_obs: tuple[dict[str, Any], dict[str, Any]] | None = None
        self._logger = logging.getLogger(__name__)
        self.check_home_collision = check_home_collision

    def step(self, action: dict[str, Any]) -> tuple[dict[str, Any], SupportsFloat, bool, bool, dict[str, Any]]:
        _, _, _, _, info = self.collision_env.step(action)
        if (
            self.unwrapped.control_mode == ControlMode.JOINTS
            and self.collision_env.unwrapped.control_mode != ControlMode.JOINTS
        ):
            action[self.unwrapped.joints_key] = self.collision_env.unwrapped.robot.get_cartesian_position()

        # modify action to be joint angles down stream
        if info["collision"] or not info["ik_success"]:
            # return old obs, with truncated and print warning
            self._logger.warning("Collision detected! Truncating episode.")
            if self.last_obs is None:
                msg = "Collisions detected and no old observation."
                raise RuntimeError(msg)
            old_obs, old_info = self.last_obs
            old_info.update(info)
            return old_obs, 0, False, True, old_info

        obs, reward, done, truncated, info = super().step(action)
        self.last_obs = obs, info
        return obs, reward, done, truncated, info

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        # check if move to home is collision free
        if self.check_home_collision:
            self.collision_env.sim_robot.move_home()
            self.collision_env.sim.step_until_convergence()
            state = self.collision_env.sim_robot.get_state()
            if state.collision or not state.ik_success:
                msg = "Collision detected while moving to home position!"
                raise RuntimeError(msg)
        else:
            self.collision_env.sim_robot.reset()
        obs, info = super().reset(seed=seed, options=options)
        self.last_obs = obs, info
        return obs, info

    @classmethod
    def env_from_xml_paths(
        cls,
        env: gym.Env,
        mjmld: str,
        rlmdl: str,
        id="0",
        gripper=False,
        check_home_collision=True,
        tcp_offset=None,
        control_mode: ControlMode | None = None,
    ) -> "CollisionGuard":
        assert isinstance(env.unwrapped, FR3Env)
        simulation = sim.Sim(mjmld)
        robot = rcsss.sim.FR3(simulation, id, rlmdl)
        cfg = sim.FR3Config()
        cfg.ik_duration_in_milliseconds = 300
        cfg.realtime = False
        if tcp_offset is not None:
            cfg.tcp_offset = tcp_offset
        robot.set_parameters(cfg)
        c_env = FR3Env(robot, env.unwrapped.control_mode) if control_mode is None else FR3Env(robot, control_mode)
        if gripper:
            gripper_cfg = sim.FHConfig()
            gripper = sim.FrankaHand(simulation, "0", gripper_cfg)
            g_env = GripperWrapper(c_env, gripper)
            return cls(env, simulation, FR3Sim(g_env, simulation), check_home_collision)
        return cls(env, simulation, FR3Sim(c_env, simulation), check_home_collision)


if __name__ == "__main__":
    logger = logging.getLogger(__name__)
    if "lab" not in rcsss.scenes:
        logger.error("This pip package was not built with the UTN lab models, aborting.")
        sys.exit()
    simulation = sim.Sim(rcsss.scenes["lab"])
    robot = rcsss.sim.FR3(simulation, "0", str(rcsss.scenes["lab"].parent / "fr3.urdf"))
    cfg = sim.FR3Config()
    cfg.tcp_offset = rcsss.common.FrankaHandTCPOffset()
    cfg.ik_duration_in_milliseconds = 300
    cfg.realtime = False
    robot.set_parameters(cfg)
    # env = FR3Env(robot, ControlMode.CARTESIAN_TQuart)
    env = FR3Env(robot, ControlMode.JOINTS)
    env_sim = FR3Sim(env, simulation)
    cameras = {
        "wrist": SimCameraConfig(identifier="eye-in-hand_0", type=int(CameraType.fixed), on_screen_render=False),
        "default_free": SimCameraConfig(identifier="", type=int(CameraType.default_free), on_screen_render=True),
    }
    cam_cfg = SimCameraSetConfig(cameras=cameras, resolution_width=640, resolution_height=480, frame_rate=50)
    camera_set = SimCameraSet(simulation, cam_cfg)
    env_cam: gym.Env = CameraSetWrapper(env_sim, camera_set)

    gripper_cfg = sim.FHConfig()
    gripper = sim.FrankaHand(simulation, "0", gripper_cfg)
    env_cam = GripperWrapper(env_cam, gripper)
    env_cam = CollisionGuard.env_from_xml_paths(
        env_cam,
        str(rcsss.scenes["lab"]),
        str(rcsss.scenes["lab"].parent / "fr3.urdf"),
        gripper=True,
        check_home_collision=False,
    )
    env_cam = RelativeActionSpace(env_cam)
    obs, info = env_cam.reset()
    for _ in range(100):
        act = env_cam.action_space.sample()
        # act = {"tquart": np.array([0, 0, 0, 0, 0, 0, 1]), "gripper": i%2}
        # act["gripper"] = i % 2
        obs, reward, terminated, truncated, info = env_cam.step(act)
        if truncated or terminated:
            logger.info("Truncated or terminated!")
            env_cam.reset()
        logger.info(act["gripper"], obs["gripper"])
