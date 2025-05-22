import logging
from typing import Any, SupportsFloat, Type, cast

import gymnasium as gym
import numpy as np
import rcs
from rcs import sim
from rcs.envs.base import ControlMode, GripperWrapper, RobotEnv
from rcs.envs.space_utils import ActObsInfoWrapper, VecType
from rcs.envs.utils import default_fr3_sim_robot_cfg

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class SimWrapper(gym.Wrapper):
    """A sub class of this wrapper can be passed to FR3Sim to assure that its code is called before
    step_until_convergence() is called.
    """

    def __init__(self, env: gym.Env, simulation: sim.Sim):
        super().__init__(env)
        self.unwrapped: RobotEnv
        assert isinstance(self.unwrapped.robot, sim.SimRobot), "Robot must be a sim.SimRobot instance."
        self.sim = simulation


class RobotSimWrapper(gym.Wrapper):
    def __init__(self, env, simulation: sim.Sim, sim_wrapper: Type[SimWrapper] | None = None):
        self.sim_wrapper = sim_wrapper
        if sim_wrapper is not None:
            env = sim_wrapper(env, simulation)
        super().__init__(env)
        self.unwrapped: RobotEnv
        assert isinstance(self.unwrapped.robot, sim.SimRobot), "Robot must be a sim.SimRobot instance."
        self.sim_robot = cast(sim.SimRobot, self.unwrapped.robot)
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
        self.sim.reset()
        _, info = super().reset(seed=seed, options=options)
        # self.unwrapped.robot.move_home()
        self.sim.step(1)
        obs = cast(dict, self.unwrapped.get_obs())
        return obs, info


class GripperWrapperSim(ActObsInfoWrapper):
    def __init__(self, env, gripper: sim.SimGripper):
        super().__init__(env)
        self._gripper = gripper

    def observation(self, observation: dict[str, Any], info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        state = self._gripper.get_state()
        if "collision" not in info or not info["collision"]:
            info["collision"] = state.collision
        return observation, info


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
        truncate_on_collision: bool = True,
    ):
        super().__init__(env)
        self.unwrapped: RobotEnv
        self.collision_env = collision_env
        self.sim = simulation
        self.last_obs: tuple[dict[str, Any], dict[str, Any]] | None = None
        self._logger = logging.getLogger(__name__)
        self.check_home_collision = check_home_collision
        self.to_joint_control = to_joint_control
        self.truncate_on_collision = truncate_on_collision
        if to_joint_control:
            assert (
                self.unwrapped.get_unwrapped_control_mode(-2) == ControlMode.JOINTS
            ), "Previous control mode must be joints"
            # change action space
            self.action_space = self.collision_env.action_space
        if sim_gui:
            self.sim.open_gui()

    def step(self, action: dict[str, Any]) -> tuple[dict[str, Any], SupportsFloat, bool, bool, dict[str, Any]]:

        self.collision_env.get_wrapper_attr("robot").set_joints_hard(self.unwrapped.robot.get_joint_position())
        _, _, _, _, info = self.collision_env.step(action)

        if self.to_joint_control:
            fr3_env = self.collision_env.unwrapped
            assert isinstance(fr3_env, RobotEnv), "Collision env must be an RobotEnv instance."
            action[self.unwrapped.joints_key] = fr3_env.robot.get_joint_position()

        if info["collision"]:
            self._logger.warning("Collision detected! %s", info)
            action[self.unwrapped.joints_key] = self.unwrapped.robot.get_joint_position()
            if self.truncate_on_collision:
                if self.last_obs is None:
                    msg = "Collision detected in the first step!"
                    raise RuntimeError(msg)
                return self.last_obs[0], 0, True, True, info

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
        tcp_offset: rcs.common.Pose | None = None,
        control_mode: ControlMode | None = None,
        sim_gui: bool = True,
        truncate_on_collision: bool = True,
    ) -> "CollisionGuard":
        # TODO: this needs to support non FR3 robots
        assert isinstance(env.unwrapped, RobotEnv)
        simulation = sim.Sim(mjmld)
        ik = rcs.common.IK(urdf, max_duration_ms=300)
        cfg = default_fr3_sim_robot_cfg(mjmld, id)
        cfg.realtime = False
        if tcp_offset is not None:
            cfg.tcp_offset = tcp_offset
        robot = rcs.sim.SimRobot(simulation, ik, cfg)
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
        c_env: gym.Env = RobotEnv(robot, control_mode)
        c_env = RobotSimWrapper(c_env, simulation)
        if gripper:
            gripper_cfg = sim.SimGripperConfig()
            gripper_cfg.add_id(id)
            fh = sim.SimGripper(simulation, gripper_cfg)
            c_env = GripperWrapper(c_env, fh)
            c_env = GripperWrapperSim(c_env, fh)
        return cls(
            env=env,
            simulation=simulation,
            collision_env=c_env,
            check_home_collision=check_home_collision,
            to_joint_control=to_joint_control,
            sim_gui=sim_gui,
            truncate_on_collision=truncate_on_collision,
        )


class RandomCubePos(SimWrapper):
    """Wrapper to randomly place cube in the lab environments."""

    def __init__(self, env: gym.Env, simulation: sim.Sim, include_rotation: bool = False):
        super().__init__(env, simulation)
        self.include_rotation = include_rotation

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        obs, info = super().reset(seed=seed, options=options)
        self.sim.step(1)

        iso_cube = np.array([0.498, 0.0, 0.226])
        iso_cube_pose = rcs.common.Pose(translation=np.array(iso_cube), rpy_vector=np.array([0, 0, 0]))
        iso_cube = self.unwrapped.robot.to_pose_in_world_coordinates(iso_cube_pose).translation()
        pos_z = 0.826
        pos_x = iso_cube[0] + np.random.random() * 0.2 - 0.1
        pos_y = iso_cube[1] + np.random.random() * 0.2 - 0.1

        if self.include_rotation:
            self.sim.data.joint("box-joint").qpos = [pos_x, pos_y, pos_z, 2 * np.random.random() - 1, 0, 0, 1]
        else:
            self.sim.data.joint("box-joint").qpos = [pos_x, pos_y, pos_z, 0, 0, 0, 1]

        return obs, info


class PickCubeSuccessWrapper(gym.Wrapper):
    """Wrapper to check if the cube is successfully picked up in the FR3SimplePickUpSim environment."""

    EE_HOME = np.array([0.34169773, 0.00047028, 0.4309004])

    def __init__(self, env):
        super().__init__(env)
        self.unwrapped: RobotEnv
        assert isinstance(self.unwrapped.robot, sim.SimRobot), "Robot must be a sim.SimRobot instance."
        self.sim = env.get_wrapper_attr("sim")

    def step(self, action: dict[str, Any]):
        obs, reward, _, truncated, info = super().step(action)

        success = (
            self.sim.data.joint("box-joint").qpos[2] > 0.15 + 0.852
            and obs["gripper"] == GripperWrapper.BINARY_GRIPPER_CLOSED
        )
        info["success"] = success
        if success:
            reward = 5
        else:
            tcp_to_obj_dist = np.linalg.norm(
                self.sim.data.joint("box-joint").qpos[:3] - self.unwrapped.robot.get_cartesian_position().translation()
            )
            obj_to_goal_dist = np.linalg.norm(self.sim.data.joint("box-joint").qpos[:3] - self.EE_HOME)

            # old reward
            # reward = -obj_to_goal_dist - tcp_to_obj_dist

            # Maniskill grasp reward
            reaching_reward = 1 - np.tanh(5 * tcp_to_obj_dist)
            reward = reaching_reward
            is_grasped = info["is_grasped"]
            reward += is_grasped
            place_reward = 1 - np.tanh(5 * obj_to_goal_dist)
            reward += place_reward * is_grasped

            # velocities are currently always zero after a step
            # qvel = self.agent.robot.get_qvel()
            # static_reward = 1 - np.tanh(5 * np.linalg.norm(qvel, axis=1))
            # reward += static_reward * info["is_obj_placed"]

        # normalize
        reward /= 5  # type: ignore
        return obs, reward, success, truncated, info


class CamRobot(gym.Wrapper):
    """Use this wrapper in lab setups where the second arm is used as a camera holder."""

    def __init__(self, env, cam_robot_joints: VecType, scene: str = "lab_simple_pick_up_digit_hand"):
        super().__init__(env)
        self.unwrapped: RobotEnv
        assert isinstance(self.unwrapped.robot, sim.SimRobot), "Robot must be a sim.SimRobot instance."
        self.sim = env.get_wrapper_attr("sim")
        cfg = default_fr3_sim_robot_cfg(scene, "1")
        self.cam_robot = rcs.sim.SimRobot(
            self.sim, env.unwrapped.robot.get_ik(), cfg, register_convergence_callback=False
        )
        self.cam_robot.set_parameters(default_fr3_sim_robot_cfg(scene))
        self.cam_robot_joints = cam_robot_joints

    def step(self, action: dict):
        self.cam_robot.set_joints_hard(self.cam_robot_joints)
        obs, reward, done, truncated, info = super().step(action)
        return obs, reward, done, truncated, info

    def reset(self, *, seed=None, options=None):
        re = super().reset(seed=seed, options=options)
        self.cam_robot.reset()
        return re
