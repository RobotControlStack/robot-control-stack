import logging
from typing import Any, SupportsFloat, cast

import gymnasium as gym
import rcsss
from rcsss import sim
from rcsss.envs.base import ControlMode, FR3Env, GripperWrapper


class FR3Sim(gym.Wrapper):
    def __init__(self, env, simulation: sim.Sim):
        super().__init__(env)
        self.unwrapped: FR3Env
        assert isinstance(self.unwrapped.robot, sim.FR3), "Robot must be a sim.FR3 instance."
        self.sim_robot = cast(sim.FR3, self.unwrapped.robot)
        self.sim = simulation

    def step(self, action: dict[str, Any]) -> tuple[dict[str, Any], float, bool, bool, dict]:
        _, _, _, _, info = super().step(action)
        self.sim.step_until_convergence()
        state = self.sim_robot.get_state()
        info["collision"] = state.collision
        info["ik_success"] = state.ik_success
        info["is_sim_converged"] = self.sim.is_converged()
        # truncate episode if collision

        return dict(self.unwrapped.get_obs()), 0, False, state.collision or not state.ik_success, info

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        _, info = super().reset(seed=seed, options=options)
        self.sim.step(1)
        obs = cast(dict, self.unwrapped.get_obs())
        return obs, info


class CollisionGuard(gym.Wrapper[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any]]):
    """
    - Gripper Wrapper has to be added before this as it removes the gripper action
    - RelativeActionSpace has to be added after this as it changes the input space, and the input expects absolute actions
    """

    def __init__(
        self,
        env: gym.Env,
        simulation: sim.Sim,
        collision_env: gym.Env,
        check_home_collision: bool = True,
        to_joint_control: bool = False,
        sim_gui: bool = True,
    ):
        super().__init__(env)
        self.unwrapped: FR3Env
        self.collision_env = collision_env
        self.sim = simulation
        self.last_obs: tuple[dict[str, Any], dict[str, Any]] | None = None
        self._logger = logging.getLogger(__name__)
        self.check_home_collision = check_home_collision
        self.to_joint_control = to_joint_control
        if to_joint_control:
            assert (
                self.unwrapped.get_unwrapped_control_mode(-2) == ControlMode.JOINTS
            ), "Previous control mode must be joints"
            # change action space
            self.action_space = self.collision_env.action_space
        if sim_gui:
            self.sim.open_gui()

    def step(self, action: dict[str, Any]) -> tuple[dict[str, Any], SupportsFloat, bool, bool, dict[str, Any]]:
        # TODO: we should set the state of the sim to the state of the real robot
        _, _, _, _, info = self.collision_env.step(action)
        if self.to_joint_control:
            fr3_env = self.collision_env.unwrapped
            assert isinstance(fr3_env, FR3Env), "Collision env must be an FR3Env instance."
            action[self.unwrapped.joints_key] = fr3_env.robot.get_joint_position()

        # modify action to be joint angles down stream
        if info["collision"] or not info["ik_success"] or not info["is_sim_converged"]:
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
            self.collision_env.get_wrapper_attr("sim_robot").move_home()
            self.collision_env.get_wrapper_attr("sim").step_until_convergence()
            state = self.collision_env.get_wrapper_attr("sim_robot").get_state()
            if state.collision or not state.ik_success:
                msg = "Collision detected while moving to home position!"
                raise RuntimeError(msg)
        else:
            self.collision_env.get_wrapper_attr("sim_robot").reset()
        obs, info = super().reset(seed=seed, options=options)
        self.last_obs = obs, info
        return obs, info

    @classmethod
    def env_from_xml_paths(
        cls,
        env: gym.Env,
        mjmld: str,
        urdf: str,
        id: str = "0",
        gripper: bool = True,
        check_home_collision: bool = True,
        tcp_offset: rcsss.common.Pose | None = None,
        control_mode: ControlMode | None = None,
        sim_gui: bool = True,
    ) -> "CollisionGuard":
        assert isinstance(env.unwrapped, FR3Env)
        simulation = sim.Sim(mjmld)
        ik = rcsss.common.IK(urdf, max_duration_ms=300)
        robot = rcsss.sim.FR3(simulation, id, ik)
        cfg = sim.FR3Config()
        cfg.realtime = False
        if tcp_offset is not None:
            cfg.tcp_offset = tcp_offset
        robot.set_parameters(cfg)
        to_joint_control = False
        if control_mode is not None:
            if control_mode != env.unwrapped.get_control_mode():
                assert (
                    env.unwrapped.get_control_mode() == ControlMode.JOINTS
                ), "A different control mode between collision guard and base env can only be used if the base env uses joint control"
                env.unwrapped.override_control_mode(control_mode)
                to_joint_control = True
        else:
            control_mode = env.unwrapped.get_control_mode()
        c_env: gym.Env = FR3Env(robot, control_mode)
        c_env = FR3Sim(c_env, simulation)
        if gripper:
            gripper_cfg = sim.FHConfig()
            fh = sim.FrankaHand(simulation, id, gripper_cfg)
            c_env = GripperWrapper(c_env, fh)
        return cls(env, simulation, c_env, check_home_collision, to_joint_control, sim_gui)
