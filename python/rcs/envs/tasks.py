from dataclasses import dataclass, field
from typing import Any

import gymnasium as gym
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
        # assert isinstance(self.get_wrapper_attr("robot"), sim.SimRobot), "Robot must be a sim.SimRobot instance."
        # self._robot = cast(sim.SimRobot, self.get_wrapper_attr("robot"))
        self.sim = self.env.get_wrapper_attr("sim")
        self.obj_joint_name = obj_joint_name

        # self.home_pose = self._robot.get_cartesian_position()
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

        # obj_pose = self._robot.to_pose_in_robot_coordinates(obj_pose)

        obj_pose_in_shared = self.shared2world.inverse() * obj_pose_in_world

        # tcp_to_obj_dist = np.linalg.norm(obj_pose.translation() - self._robot.get_cartesian_position().translation())
        tcp_to_obj_dist = np.linalg.norm(obj_pose_in_shared.translation() - obs[self.robot_name]["tquat"][:3])

        obj_to_goal_dist = 0.10 - min(obj_pose_in_shared.translation()[-1], 0.10)
        # obj_to_goal_dist = np.linalg.norm(cube_pose.translation() - self.home_pose.translation())

        # NOTE: 4 depends on the time passing between each step.
        is_grasped = (
            self._gripper_closing >= 4  # gripper is closing since more than 4 steps
            and gripper_closed  # command is still close
            and tcp_to_obj_dist <= 0.01  # tcp to cube center is max 1cm
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
        # self.home_pose = self._robot.get_cartesian_position()
        return obs, info


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

        # place randomly in square
        pos_x = self.np_random.uniform(-self.x_width / 2, self.x_width / 2)
        pos_y = self.np_random.uniform(-self.y_width / 2, self.y_width / 2)

        if self.include_rotation:
            # 1. Sample a random angle between 0 and 2*pi (360 degrees)
            theta = self.np_random.uniform(0, 2 * np.pi)

            # 2. Convert the angle to a unit quaternion for the Z-axis
            qw = np.cos(theta / 2)
            qz = np.sin(theta / 2)
        else:
            # No rotation (Identity quaternion)
            qw = 1.0
            qz = 0.0

        pose_in_center_frame = rcs.common.Pose(
            translation=np.array([pos_x, pos_y, 0]), quaternion=np.array([0, 0, qz, qw])
        )
        pose_in_world_frame = self.center2world * pose_in_center_frame

        # qpos array format for a free joint: [x, y, z, qw, qx, qy, qz]
        self.get_wrapper_attr("sim").data.joint(self.obj_joint_name).qpos = np.append(
            pose_in_world_frame.translation(), pose_in_world_frame.rotation_q_wxyz()
        )

        # reset of remaining stack, must happen after our reset!
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
