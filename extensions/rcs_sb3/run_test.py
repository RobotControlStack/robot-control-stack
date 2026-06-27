"""Smoke-test the RCS SB3 policy wrapper on a simulated FR3 + Taxim stack.

This does not train PPO. It uses a tiny SB3-like zero model with a ``predict``
method to verify the runtime pipeline:

    RCS + Taxim observations -> policy wrapper -> generated action dict -> robot wrappers
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import gymnasium as gym
import mujoco
import numpy as np
from rcs._core.common import GripperType, Pose
from rcs._core.sim import SimGripperConfig
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.configs import EmptyWorldFR3
from rcs_sb3 import StableBaselines3PolicyWrapper
from rcs_taxim.taxim_wrapper import TaximSimWrapper
from stable_baselines3 import PPO

import rcs

_TAXIM_GRIPPER_TYPE = GripperType("Robotiq2F85Digit")
_TAXIM_GRIPPER_XML = Path("/home/sbien/Documents/Development/V2T/mujoco-taxim/assets/robotiq_2f85/robotiq_2f85.xml")
_ROBOT_NAME = "robot"
_CUBE_GEOM = "_box_geom"


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
        gripper_type=_TAXIM_GRIPPER_TYPE,
    )


class ZeroSB3Model(PPO):
    """PPO-shaped zero-action model for pipeline testing.

    This deliberately does not call ``PPO.__init__`` yet: the smoke test only
    needs the runtime ``predict`` surface, while keeping this class as the local
    starting point for a real PPO-backed implementation.
    """

    def __init__(self, action_space: gym.Space):
        self.action_space = action_space

    def predict(
        self,
        observation: Any,
        state: Any = None,
        episode_start: np.ndarray | None = None,
        deterministic: bool = False,
    ) -> tuple[dict[str, np.ndarray], Any]:
        del observation, episode_start, deterministic
        return _zero_closed_action(self.action_space), state


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
    rcs.GRIPPER_PATHS[_TAXIM_GRIPPER_TYPE] = str(_TAXIM_GRIPPER_XML)

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


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a zero-policy SB3 wrapper smoke test in FR3 + Taxim simulation.")
    parser.add_argument("--steps", type=int, default=20)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--gui", action="store_true", help="Open the MuJoCo GUI.")
    parser.add_argument("--visualize-taxim", action="store_true", help="Show nonblocking Taxim tactile windows.")
    parser.add_argument("--disable-taxim-depth", type=bool, default=True, help="Do not include Taxim depth frames.")
    parser.add_argument("--no-start-grasped", action="store_true", help="Do not close the gripper after reset.")
    parser.add_argument("--grasp-settle-steps", type=int, default=30)
    parser.add_argument("--post-gravity-settle-steps", type=int, default=10)
    args = parser.parse_args()

    env = make_franka_taxim_sim_env(
        render_mode="human" if args.gui else "rgb_array",
        visualize_taxim=args.visualize_taxim,
        enable_depth=not args.disable_taxim_depth,
        start_grasped=not args.no_start_grasped,
        grasp_settle_steps=args.grasp_settle_steps,
        post_gravity_settle_steps=args.post_gravity_settle_steps,
    )

    zero_model = ZeroSB3Model(env.action_space)
    env = StableBaselines3PolicyWrapper(
        env,
        zero_model,
        deterministic=False,
        flatten_actions=False,
        flatten_observations=True,
    )

    obs, info = env.reset(seed=args.seed)
    print(f"reset: obs_keys={list(obs.keys())}, info_keys={list(info.keys())}")

    for step_idx in range(args.steps):
        obs, reward, terminated, truncated, info = env.step()
        # # Save some images to disk for visual inspection.
        # if step_idx == 0:
        #     frames = obs.get("frames", {})
        #     for site, tactile_obs in frames.items():
        #         rgb = tactile_obs.get("rgb", {}).get("data")
        #         if rgb is not None:
        #             from PIL import Image
        #             rgb_path = Path(f"taxim_{site}_rgb.png")
        #             print(f"Saving {rgb_path}")
        #             Image.fromarray(rgb).save(rgb_path)
        #         depth = tactile_obs.get("depth", {}).get("data")
        #         # Normalize then repeat to 3 channels for saving as PNG.
        #         depth = depth.astype(np.float32)
        #         depth_min, depth_max = np.nanmin(depth), np.nanmax(depth)
                    
        #         if depth is not None:
        #             if depth_max > depth_min:
        #                 gt_vis = np.repeat(depth[:, :, np.newaxis], 3, axis=2)
        #                 div = 1 if np.max(gt_vis) == 0 else np.max(gt_vis)
        #                 gt_vis = (gt_vis / div * 255).astype(np.uint8)
        #             depth_path = Path(f"taxim_{site}_depth.png")
        #             print(f"Saving {depth_path}")
        #             Image.fromarray(gt_vis).save(depth_path)
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


if __name__ == "__main__":
    main()
