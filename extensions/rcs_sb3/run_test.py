"""Smoke-test the RCS SB3 policy wrapper on a simulated FR3 + Taxim stack.

This uses a tiny PPO model to verify the runtime pipeline:

    RCS + Taxim observations -> policy wrapper -> generated action dict -> robot wrappers
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium.spaces import Dict as DictSpace
from rcs._core.common import GripperType, Pose
from rcs._core.sim import SimGripperConfig
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.configs import EmptyWorldFR3
from rcs_sb3 import StableBaselines3PolicyWrapper, StableBaselines3Wrapper
from rcs_taxim.taxim_wrapper import TaximSimWrapper
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv


import rcs

_TAXIM_GRIPPER_TYPE_ID = "Robotiq2F85Digit"
_TAXIM_GRIPPER_XML = Path("/home/sbien/Documents/Development/V2T/mujoco-taxim/assets/robotiq_2f85/robotiq_2f85.xml")
_ROBOT_NAME = "robot"
_CUBE_GEOM = "_box_geom"


def _taxim_gripper_type() -> GripperType:
    return GripperType(_TAXIM_GRIPPER_TYPE_ID)


def _zero_closed_action(space: gym.Space) -> dict[str, Any]:
    action = space.sample()

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


def _taxim_gripper_cfg() -> SimGripperConfig:
    return SimGripperConfig(
        epsilon_inner=0.005,
        epsilon_outer=0.005,
        seconds_between_callbacks=0.1,
        ignored_collision_geoms=[],
        collision_geoms=[],
        collision_geoms_fingers=[],
        joints=["right_driver_joint", "left_driver_joint"],
        max_joint_width=0.005,
        min_joint_width=1.0,
        actuator="fingers_actuator",
        max_actuator_width=0,
        min_actuator_width=255,
        gripper_type=_taxim_gripper_type(),
    )


class DummySB3Model(PPO):
    """PPO subclass used as the starting point for real SB3 training logic."""

    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__(*args, **kwargs)


def make_zero_sb3_model(env: gym.Env, args: argparse.Namespace) -> DummySB3Model:
    return DummySB3Model(
        "MultiInputPolicy",
        env,
        n_steps=args.n_steps,
        batch_size=args.batch_size,
        seed=args.seed,
        verbose=1,
    )


class StartGraspedWrapper(gym.Wrapper):
    """Close the gripper after reset so the episode starts with the object held."""

    def __init__(self, env: gym.Env, settle_steps: int = 30, post_gravity_settle_steps: int = 10):
        super().__init__(env)
        self.settle_steps = settle_steps
        self.post_gravity_settle_steps = post_gravity_settle_steps

    def _closed_gripper_hold_action(self) -> dict[str, Any]:
        return _zero_closed_action(self.env.action_space)

    def _command_gripper_width(self, sim: Any, gripper: Any, width: float) -> None:
        cfg = gripper.get_config()
        actuator_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_ACTUATOR, cfg.actuator)
        if actuator_id < 0:
            msg = f"Could not find gripper actuator {cfg.actuator!r}."
            raise ValueError(msg)
        ctrl = width * (cfg.max_actuator_width - cfg.min_actuator_width) + cfg.min_actuator_width
        ctrl_low, ctrl_high = sim.model.actuator_ctrlrange[actuator_id]
        sim.data.ctrl[actuator_id] = np.clip(ctrl, ctrl_low, ctrl_high)

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        sim = self.env.get_wrapper_attr("sim")
        gravity = np.array(sim.model.opt.gravity, copy=True)

        # CoverWrapper steps the simulation during reset. Disable gravity before
        # the wrapped reset so the cube does not fall before we can close.
        sim.model.opt.gravity[:] = 0
        obs, info = self.env.reset(seed=seed, options=options)

        robot = self.env.get_wrapper_attr("robot")
        gripper = self.env.get_wrapper_attr("gripper")
        if isinstance(robot, dict):
            robot = robot["robot"]
        if isinstance(gripper, dict):
            gripper = gripper["robot"]
        hold_joints = robot.get_joint_position().copy()

        try:
            for step_idx in range(self.settle_steps):
                width = 1.0 - (step_idx + 1) / max(self.settle_steps, 1)
                self._command_gripper_width(sim, gripper, width)
                robot.set_joint_position(hold_joints)
                sim.step(1)
        finally:
            sim.model.opt.gravity[:] = gravity

        gripper.grasp()
        for _ in range(self.post_gravity_settle_steps):
            robot.set_joint_position(hold_joints)
            gripper.grasp()
            sim.step(1)

        obs, _, terminated, truncated, info = self.env.step(self._closed_gripper_hold_action())
        if terminated or truncated:
            obs, info = self.env.reset(seed=seed, options=options)
        sim.sync_gui()
        return obs, info


def make_franka_taxim_sim_env(
    *,
    render_mode: str,
    visualize_taxim: bool,
    enable_depth: bool,
    start_grasped: bool,
    grasp_settle_steps: int,
    post_gravity_settle_steps: int,
) -> gym.Env:
    rcs.GRIPPER_PATHS[_taxim_gripper_type()] = str(_TAXIM_GRIPPER_XML)

    scene = EmptyWorldFR3()
    cfg = scene.config()
    cfg.control_mode = ControlMode.JOINTS
    cfg.headless = render_mode != "human"
    cfg.sim_cfg.realtime = render_mode == "human"
    cfg.sim_cfg.async_control = render_mode == "human"
    cfg.sim_cfg.frequency = 30
    cfg.wrapper_cfg.binary_gripper = True
    cfg.wrapper_cfg.home_on_reset = True
    cfg.max_relative_movement = np.deg2rad(5)
    cfg.relative_to = RelativeTo.LAST_STEP
    cfg.gripper_cfgs = {_ROBOT_NAME: _taxim_gripper_cfg()}
    cfg.gripper_offsets = None
    cfg.root_frame_objects = {
        "": (
            rcs.OBJECT_PATHS["green_cube"],
            Pose(translation=np.array([0.31, 0.0, 0.425]), quaternion=np.array([0.0, 0.0, 0.0, 1.0])),
        )
    }
    cfg.robot_cfgs[_ROBOT_NAME].tcp_offset = rcs.GRIPPER_OFFSETS[rcs.common.GripperType("Robotiq2F85")]
    q_home = cfg.robot_cfgs[_ROBOT_NAME].q_home.copy()
    q_home[-1] = -1.571
    cfg.robot_cfgs[_ROBOT_NAME].q_home = q_home
    cfg.camera_cfgs = None
    cfg.camera_adds = None

    env = scene.create_env(cfg)
    env = TaximSimWrapper(
        env,
        taxim_sites=["gripperleft_digit_pad", "gripperright_digit_pad"],
        taxim_pad_geoms=["gripperleft_digit_pad", "gripperright_digit_pad"],
        target_geom_mesh_dict={_CUBE_GEOM: _CUBE_GEOM},
        taxim_sensor_type="digit",
        taxim_fps=60,
        enable_depth=enable_depth,
        visualize=visualize_taxim,
    )
    if start_grasped:
        env = StartGraspedWrapper(
            env,
            settle_steps=grasp_settle_steps,
            post_gravity_settle_steps=post_gravity_settle_steps,
        )
    if render_mode == "human":
        env.get_wrapper_attr("sim").open_gui()
    return env


def make_inference_env(args: argparse.Namespace) -> gym.Env:
    raw_env = make_franka_taxim_sim_env(
        render_mode="human" if args.gui else "rgb_array",
        visualize_taxim=args.visualize_taxim,
        enable_depth=args.enable_taxim_depth,
        start_grasped=not args.no_start_grasped,
        grasp_settle_steps=args.grasp_settle_steps,
        post_gravity_settle_steps=args.post_gravity_settle_steps,
    )
    env = StableBaselines3Wrapper(
        raw_env,
        flatten_actions=False,
        flatten_observations=False,
    )
    zero_model = make_zero_sb3_model(env, args)
    return StableBaselines3PolicyWrapper(
        env,
        zero_model,
        deterministic=False,
        flatten_actions=False,
        flatten_observations=False,
    )


def run_inference_smoke(args: argparse.Namespace) -> None:
    env = make_inference_env(args)
    obs, info = env.reset(seed=args.seed)
    print(f"reset: obs_keys={list(obs.keys())}, info_keys={list(info.keys())}")

    for step_idx in range(args.infer_steps):
        obs, reward, terminated, truncated, info = env.step()
        action = info.get(StableBaselines3PolicyWrapper.ACTION_INFO_KEY)
        action_keys = list(action.keys()) if isinstance(action, dict) else type(action).__name__
        frame_keys = list(obs.get("frames", {}).keys()) if isinstance(obs, dict) else []
        print(
            f"step={step_idx:03d} reward={reward:.3f} "
            f"terminated={terminated} truncated={truncated} "
            f"action_keys={action_keys} frame_keys={frame_keys}"
        )
        if terminated or truncated:
            obs, info = env.reset()
            print(f"reset: obs_keys={list(obs.keys())}, info_keys={list(info.keys())}")

    env.close()


def make_train_env(
    rank: int,
    seed: int,
    start_grasped: bool,
    grasp_settle_steps: int,
    post_gravity_settle_steps: int,
):
    def _init():
        env = make_franka_taxim_sim_env(
            render_mode="rgb_array",
            visualize_taxim=False,
            enable_depth=False,
            start_grasped=start_grasped,
            grasp_settle_steps=grasp_settle_steps,
            post_gravity_settle_steps=post_gravity_settle_steps,
        )
        env.reset(seed=seed + rank)
        return env

    return _init


def make_sb3_train_env(
    rank: int,
    seed: int,
    start_grasped: bool,
    grasp_settle_steps: int,
    post_gravity_settle_steps: int,
):
    raw_env_fn = make_train_env(
        rank,
        seed,
        start_grasped,
        grasp_settle_steps,
        post_gravity_settle_steps,
    )

    def _init():
        return StableBaselines3Wrapper(
            raw_env_fn(),
            flatten_actions=False,
            flatten_observations=False,
        )

    return _init


def run_training_smoke(args: argparse.Namespace) -> None:
    if args.num_envs < 0:
        msg = "--num-envs must be >= 0. Use 0 for single-process breakpoint debugging."
        raise ValueError(msg)

    if args.num_envs == 0:
        train_env = make_sb3_train_env(
            0,
            args.seed,
            not args.no_start_grasped,
            args.grasp_settle_steps,
            args.post_gravity_settle_steps,
        )()
    else:
        train_env = SubprocVecEnv(
            [
                make_sb3_train_env(
                    rank,
                    args.seed,
                    not args.no_start_grasped,
                    args.grasp_settle_steps,
                    args.post_gravity_settle_steps,
                )
                for rank in range(args.num_envs)
            ],
            start_method="spawn",
        )
    try:
        if isinstance(train_env.action_space, DictSpace):
            msg = (
                "Raw RCS envs expose Dict action spaces, but SB3 PPO does not support Dict actions. "
                "Expected make_sb3_train_env to adapt actions before constructing PPO."
            )
            raise TypeError(msg)
        model = make_zero_sb3_model(train_env, args)
        model.learn(total_timesteps=args.total_timesteps)
    finally:
        train_env.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Smoke-test RCS + Taxim with SB3.")
    parser.add_argument("--mode", choices=("infer", "train"), default="infer")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--enable-taxim-depth", action="store_true")
    parser.add_argument("--no-start-grasped", action="store_true", help="Do not close the gripper after reset.")
    parser.add_argument("--grasp-settle-steps", type=int, default=30)
    parser.add_argument("--post-gravity-settle-steps", type=int, default=10)

    # Inference mode args
    parser.add_argument("--infer-steps", type=int, default=20, help="Inference smoke-test steps.")
    parser.add_argument("--gui", action="store_true", help="Open the MuJoCo GUI for inference mode.")
    parser.add_argument("--visualize-taxim", action="store_true", help="Show nonblocking Taxim tactile windows.")

    # Training mode args
    parser.add_argument("--num-envs", type=int, default=2, help="Parallel env count for train mode. Use 0 to disable subprocess vectorization for debugging.",)
    parser.add_argument("--total-timesteps", type=int, default=32)
    parser.add_argument("--n-steps", type=int, default=8)
    parser.add_argument("--batch-size", type=int, default=16)
    args = parser.parse_args()

    if args.mode == "infer":
        run_inference_smoke(args)
    else:
        run_training_smoke(args)


if __name__ == "__main__":
    main()
