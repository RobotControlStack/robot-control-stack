from dataclasses import dataclass, field
from typing import Any

import gymnasium as gym
import mujoco
import numpy as np
from rcs.envs.base import GripperWrapper
from rcs.envs.scenes import BaseTaskConfig, SimEnvCreatorConfig, Task
from rcs.sim.composer import ModelComposer
from rcs.sim.sim import Sim

import rcs


class PickObjSuccessWrapper(gym.Wrapper):
    """
    Wrapper to check if an object is successfully picked up.
    Obj must be lifted 10 cm above its position.
    Computes a reward between 0 and 1 based on:
    - TCP to object distance
    - cube z height
    - whether the arm is standing still once the task is solved.
    """

    def __init__(self, env, robot_name: str, shared2world: rcs.common.Pose, obj_joint_name="box_joint"):
        super().__init__(env)
        self.sim = self.env.get_wrapper_attr("sim")
        self.obj_joint_name = obj_joint_name
        self._gripper_closing = 0
        self.robot_name = robot_name
        self._gripper = self.get_wrapper_attr("gripper")[self.robot_name]
        self.shared2world = shared2world

    def step(self, action: dict[str, Any]):  # type: ignore
        obs, reward, _, truncated, info = super().step(action)

        gripper_closed = obs[self.robot_name]["gripper"][0] == GripperWrapper.BINARY_GRIPPER_CLOSED[0]

        if (
            self._gripper.get_normalized_width() > 0.01
            and self._gripper.get_normalized_width() < 0.99
            and gripper_closed
        ):
            self._gripper_closing += 1
        else:
            self._gripper_closing = 0

        obj_pose_in_world = rcs.common.Pose(translation=self.sim.data.joint(self.obj_joint_name).qpos[:3])
        obj_pose_in_shared = self.shared2world.inverse() * obj_pose_in_world
        tcp_to_obj_dist = np.linalg.norm(obj_pose_in_shared.translation() - obs[self.robot_name]["tquat"][:3])
        obj_to_goal_dist = 0.10 - min(obj_pose_in_shared.translation()[-1], 0.10)

        is_grasped = (
            self._gripper_closing >= 4
            and gripper_closed
            and tcp_to_obj_dist <= 0.01
        )
        success = obj_to_goal_dist <= 0.022 and info[self.robot_name]["is_grasped"]
        movement = np.linalg.norm(self.sim.data.qvel)

        reaching_reward = 1 - np.tanh(5 * tcp_to_obj_dist)
        place_reward = 1 - np.tanh(5 * obj_to_goal_dist)
        static_reward = 1 - np.tanh(5 * movement)
        info["is_grasped"] = is_grasped
        info["success"] = success
        reward = reaching_reward + place_reward * is_grasped + static_reward * success
        reward /= 3  # type: ignore
        return obs, reward, success, truncated, info

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        obs, info = super().reset()
        return obs, info


class StartGraspedWrapper(gym.Wrapper):
    """Start the episode with the cube already held by the gripper."""

    def __init__(
        self,
        env: gym.Env,
        robot_name: str,
        obj_joint_name: str,
        settle_steps: int = 30,
        post_gravity_settle_steps: int = 10,
        start_cube_offset: rcs.common.Pose | None = None,
    ):
        super().__init__(env)
        self.robot_name = robot_name
        self.obj_joint_name = obj_joint_name
        self.settle_steps = settle_steps
        self.post_gravity_settle_steps = post_gravity_settle_steps
        self.start_cube_offset = start_cube_offset or rcs.common.Pose()

    def _closed_gripper_hold_action(self) -> dict[str, Any]:
        action = self.env.action_space.sample()

        def visit(value: Any) -> None:
            if isinstance(value, dict):
                if "gripper" in value:
                    value["gripper"] = np.array([0.0], dtype=np.float32)
                for child in value.values():
                    visit(child)
            elif isinstance(value, np.ndarray):
                value[...] = 0

        visit(action)
        return action

    def _command_gripper_width(self, sim: Any, gripper: Any, width: float) -> None:
        cfg = gripper.get_config()
        actuator_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_ACTUATOR, cfg.actuator)
        if actuator_id < 0:
            msg = f"Could not find gripper actuator {cfg.actuator!r}."
            raise ValueError(msg)
        ctrl = width * (cfg.max_actuator_width - cfg.min_actuator_width) + cfg.min_actuator_width
        ctrl_low, ctrl_high = sim.model.actuator_ctrlrange[actuator_id]
        sim.data.ctrl[actuator_id] = np.clip(ctrl, ctrl_low, ctrl_high)

    def _set_object_pose(self, sim: Any, pose: rcs.common.Pose) -> None:
        sim.data.joint(self.obj_joint_name).qpos = np.append(pose.translation(), pose.rotation_q_wxyz())

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        sim = self.env.get_wrapper_attr("sim")
        gravity = np.array(sim.model.opt.gravity, copy=True)

        sim.model.opt.gravity[:] = 0
        obs, info = self.env.reset(seed=seed, options=options)

        robot = self.env.get_wrapper_attr("robot")
        gripper = self.env.get_wrapper_attr("gripper")
        if isinstance(robot, dict):
            robot = robot[self.robot_name]
        if isinstance(gripper, dict):
            gripper = gripper[self.robot_name]
        hold_joints = robot.get_joint_position().copy()

        try:
            pregrasp_pose = robot.get_cartesian_position() * self.start_cube_offset
            self._set_object_pose(sim, pregrasp_pose)
            for step_idx in range(self.settle_steps):
                width = 1.0 - (step_idx + 1) / max(self.settle_steps, 1)
                self._command_gripper_width(sim, gripper, width)
                robot.set_joint_position(hold_joints)
                self._set_object_pose(sim, pregrasp_pose)
                sim.step(1)
        finally:
            sim.model.opt.gravity[:] = gravity

        gripper.grasp()
        for _ in range(self.post_gravity_settle_steps):
            robot.set_joint_position(hold_joints)
            self._set_object_pose(sim, pregrasp_pose)
            gripper.grasp()
            sim.step(1)

        obs, _, terminated, truncated, info = self.env.step(self._closed_gripper_hold_action())
        if terminated or truncated:
            obs, info = self.env.reset(seed=seed, options=options)
        sim.sync_gui()
        return obs, info


class MinOpenHoldRewardWrapper(gym.Wrapper):
    """Reward holding the cube while keeping the gripper as open as possible."""

    def __init__(
        self,
        env: gym.Env,
        *,
        robot_name: str,
        obj_joint_name: str,
        drop_z_threshold: float = 0.02,
        hold_bonus: float = 1.0,
        max_episode_steps: int = 100,
    ):
        super().__init__(env)
        self.robot_name = robot_name
        self.obj_joint_name = obj_joint_name
        self.drop_z_threshold = drop_z_threshold
        self.hold_bonus = hold_bonus
        self.max_episode_steps = max_episode_steps
        self._start_obj_z: float | None = None
        self._step_count = 0

    def _object_z(self) -> float:
        sim = self.env.get_wrapper_attr("sim")
        return float(np.asarray(sim.data.joint(self.obj_joint_name).qpos).reshape(-1)[2])

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        obs, info = super().reset(seed=seed, options=options)
        self._start_obj_z = self._object_z()
        self._step_count = 0
        return obs, info

    def step(self, action: dict[str, Any]):  # type: ignore[override]
        obs, _, terminated, truncated, info = super().step(action)
        self._step_count += 1
        gripper_width = float(info.get("gripper_width", 0.0))
        is_grasped = bool(info.get("is_grasped", False))
        obj_z = self._object_z()
        dropped = self._start_obj_z is not None and obj_z < self._start_obj_z - self.drop_z_threshold

        if dropped or not is_grasped:
            reward = 0.0
            terminated = True
        else:
            reward = self.hold_bonus + gripper_width
            truncated = truncated or self._step_count >= self.max_episode_steps

        info["hold_open_reward"] = reward
        info["object_z"] = obj_z
        info["start_object_z"] = self._start_obj_z
        info["is_dropped"] = dropped or not is_grasped
        info["step_count"] = self._step_count
        return obs, reward, terminated, truncated, info


class RandomSquareObjPos(gym.Wrapper):
    """Wrapper to an object in a simulated environment in a random spot inside a defined square.

    Works only for single robot
    """

    def __init__(
        self,
        env: gym.Env,
        center2world: rcs.common.Pose,
        include_rotation: bool = True,
        obj_joint_name: str = "box_joint",
        x_width: float = 0.2,
        y_width: float = 0.2,
    ):
        super().__init__(env)
        self.include_rotation = include_rotation
        self.obj_joint_name = obj_joint_name
        self.center2world = center2world
        self.x_width = x_width
        self.y_width = y_width

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:

        pos_x = self.np_random.uniform(-self.x_width / 2, self.x_width / 2)
        pos_y = self.np_random.uniform(-self.y_width / 2, self.y_width / 2)

        if self.include_rotation:
            theta = self.np_random.uniform(0, 2 * np.pi)
            qw = np.cos(theta / 2)
            qz = np.sin(theta / 2)
        else:
            qw = 1.0
            qz = 0.0

        pose_in_center_frame = rcs.common.Pose(
            translation=np.array([pos_x, pos_y, 0]), quaternion=np.array([0, 0, qz, qw])
        )
        pose_in_world_frame = self.center2world * pose_in_center_frame

        self.get_wrapper_attr("sim").data.joint(self.obj_joint_name).qpos = np.append(
            pose_in_world_frame.translation(), pose_in_world_frame.rotation_q_wxyz()
        )

        return super().reset(seed=seed, options=options)


@dataclass(kw_only=True)
class PickTaskConfig(BaseTaskConfig):
    robot_name: str
    object_center_to_root_frame: rcs.common.Pose = field(
        default_factory=lambda: rcs.common.Pose(
            translation=np.array([0.5, 0.0, 0.05]), quaternion=np.array([0, 0, 0, 1])
        )
    )
    object_xml = rcs.OBJECT_PATHS["green_cube"]
    object_joint: str = "box_joint"
    prefix: str = "PickTask_"
    include_rotation: bool = True
    task_id: str = "pick"


class PickTask(Task[PickTaskConfig]):

    @staticmethod
    def add_task_mujoco(cfg: PickTaskConfig, composer: ModelComposer, env_cfg: SimEnvCreatorConfig):
        """Add task-specific elements to the Mujoco scene."""
        object2world = cfg.object_center_to_root_frame * env_cfg.root_frame_to_world

        composer.add_object_world_frame(
            cfg.object_xml,
            object_prefix=cfg.prefix,
            pose=object2world,
            register_root_relative_replay_free_joints=True,
        )

    @staticmethod
    def add_task_env(cfg: PickTaskConfig, env: gym.Env, _simulation: Sim, env_cfg: SimEnvCreatorConfig) -> gym.Env:
        """Add task-specific wrappers to the environment."""
        object2world = cfg.object_center_to_root_frame * env_cfg.root_frame_to_world
        shared2world = env_cfg.shared_base_frame_to_root_frame * env_cfg.root_frame_to_world
        object_joint = cfg.prefix + cfg.object_joint
        env = PickObjSuccessWrapper(env, cfg.robot_name, shared2world, object_joint)
        return RandomSquareObjPos(
            env, center2world=object2world, include_rotation=cfg.include_rotation, obj_joint_name=object_joint
        )


rcs.TASKS["pick"] = PickTask


@dataclass(kw_only=True)
class TaximPickHoldOpenTaskConfig(BaseTaskConfig):
    robot_name: str
    object_center_to_root_frame: rcs.common.Pose = field(
        default_factory=lambda: rcs.common.Pose(
            translation=np.array([0.5, 0.0, 0.05]), quaternion=np.array([0, 0, 0, 1])
        )
    )
    object_xml = rcs.OBJECT_PATHS["green_cube"]
    object_joint: str = "box_joint"
    prefix: str = "TaximHoldOpenTask_"
    grasp_settle_steps: int = 30
    post_gravity_settle_steps: int = 10
    drop_z_threshold: float = 0.02
    hold_bonus: float = 1.0
    max_episode_steps: int = 100
    start_cube_offset: rcs.common.Pose = field(default_factory=rcs.common.Pose)
    task_id: str = "taxim_pick_hold_open"


class TaximPickHoldOpenTask(Task[TaximPickHoldOpenTaskConfig]):
    @staticmethod
    def add_task_mujoco(cfg: TaximPickHoldOpenTaskConfig, composer: ModelComposer, env_cfg: SimEnvCreatorConfig):
        object2world = cfg.object_center_to_root_frame * env_cfg.root_frame_to_world
        composer.add_object_world_frame(
            cfg.object_xml,
            object_prefix=cfg.prefix,
            pose=object2world,
            register_root_relative_replay_free_joints=True,
        )

    @staticmethod
    def add_task_env(
        cfg: TaximPickHoldOpenTaskConfig, env: gym.Env, _simulation: Sim, env_cfg: SimEnvCreatorConfig
    ) -> gym.Env:
        object_joint = cfg.prefix + cfg.object_joint
        env = StartGraspedWrapper(
            env,
            robot_name=cfg.robot_name,
            obj_joint_name=object_joint,
            settle_steps=cfg.grasp_settle_steps,
            post_gravity_settle_steps=cfg.post_gravity_settle_steps,
            start_cube_offset=cfg.start_cube_offset,
        )
        return MinOpenHoldRewardWrapper(
            env,
            robot_name=cfg.robot_name,
            obj_joint_name=object_joint,
            drop_z_threshold=cfg.drop_z_threshold,
            hold_bonus=cfg.hold_bonus,
            max_episode_steps=cfg.max_episode_steps,
        )


rcs.TASKS["taxim_pick_hold_open"] = TaximPickHoldOpenTask
