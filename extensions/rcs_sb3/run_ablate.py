"""Grid-search box physics parameters and record grasp slippage.

The environment setup mirrors ``run_test.py`` but swaps in generated box XML
files with different MuJoCo friction/density values. Each rollout starts with
``StartGraspedWrapper`` so the box is already held, then commands a fixed
gripper width while recording the box z-position over time. Taxim rendering is
intentionally skipped because it does not affect these physics measurements.
"""

from __future__ import annotations

import argparse
import csv
import itertools
from pathlib import Path
from typing import Any

import gymnasium as gym
import mujoco
import numpy as np
import rcs
from rcs._core.common import GripperType, Pose
from rcs._core.sim import SimGripperConfig
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.configs import EmptyWorldFR3

_TAXIM_GRIPPER_TYPE_ID = "Robotiq2F85Digit"
_TAXIM_GRIPPER_XML = Path("/home/sbien/Documents/Development/V2T/mujoco-taxim/assets/robotiq_2f85/robotiq_2f85.xml")
_BOX_XML = "/home/sbien/Documents/Development/V2T/norm2tex/grasp_assets/box/box.xml"
_ROBOT_NAME = "robot"
_SOURCE_FRICTION = 'friction="1 0.3 0.1"'
_SOURCE_DENSITY = 'density="50"'
_BOX_BODY_NAMES = ("_box_body", "box_body")
_BOX_JOINT_NAMES = ("_box_joint", "box_joint")
_CONTROL_FREQUENCY_HZ = 30.0


def _taxim_gripper_type() -> GripperType:
    return GripperType(_TAXIM_GRIPPER_TYPE_ID)


def _zero_closed_action(space: gym.Space) -> dict[str, Any]:
    return _make_gripper_width_action(space, 0.0)


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


def _float_list(value: str) -> list[float]:
    return [float(item.strip()) for item in value.split(",") if item.strip()]


def _format_float(value: float) -> str:
    return f"{value:.6g}"


def _format_filename_float(value: float) -> str:
    return _format_float(value).replace("-", "m").replace(".", "p")


def _format_friction(friction: tuple[float, float, float]) -> str:
    return " ".join(_format_float(value) for value in friction)


def _make_gripper_width_action(space: gym.Space, width: float) -> dict[str, Any]:
    action = space.sample()

    def visit(value: Any) -> None:
        if isinstance(value, dict):
            for key, child in value.items():
                if key == "gripper":
                    value[key] = np.array([width], dtype=np.float32)
                    continue
                visit(child)
        elif isinstance(value, np.ndarray):
            value[...] = 0

    visit(action)
    return action


def _extract_gripper_width(action: Any) -> float | None:
    if isinstance(action, dict):
        if "gripper" in action:
            return float(np.asarray(action["gripper"]).reshape(-1)[0])
        for value in action.values():
            width = _extract_gripper_width(value)
            if width is not None:
                return width
    return None


def _write_box_xml(
    *,
    source_xml: Path,
    friction: tuple[float, float, float],
    density: float,
) -> Path:
    xml_content = source_xml.read_text()
    if _SOURCE_FRICTION not in xml_content:
        msg = f"Expected {_SOURCE_FRICTION!r} in {source_xml}."
        raise ValueError(msg)
    if _SOURCE_DENSITY not in xml_content:
        msg = f"Expected {_SOURCE_DENSITY!r} in {source_xml}."
        raise ValueError(msg)

    xml_content = xml_content.replace(_SOURCE_FRICTION, f'friction="{_format_friction(friction)}"')
    xml_content = xml_content.replace(_SOURCE_DENSITY, f'density="{_format_float(density)}"')

    stem_values = [*friction, density]
    stem = "_".join(_format_filename_float(value) for value in stem_values)
    xml_path = source_xml.parent / f"box_{stem}.xml"
    xml_path.write_text(xml_content)
    return xml_path


class BoxZRecorderWrapper(gym.Wrapper):
    """Append one CSV row per reset/step with the box z-position."""

    FIELDNAMES = (
        "run_id",
        "phase",
        "step",
        "time_s",
        "seed",
        "friction_slide",
        "friction_spin",
        "friction_roll",
        "density",
        "commanded_gripper_width",
        "box_z",
        "z_drop_from_reset",
        "slipped",
        "terminated",
        "truncated",
        "reward",
    )

    def __init__(
        self,
        env: gym.Env,
        *,
        csv_path: Path,
        metadata: dict[str, Any],
        slip_threshold: float,
    ):
        super().__init__(env)
        self.csv_path = csv_path
        self.metadata = metadata
        self.slip_threshold = slip_threshold
        self.step_idx = 0
        self.reset_z: float | None = None

        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._needs_header = not self.csv_path.exists() or self.csv_path.stat().st_size == 0

    def _box_z(self) -> float:
        sim = self.env.get_wrapper_attr("sim")
        for joint_name in _BOX_JOINT_NAMES:
            joint_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id >= 0:
                return float(np.asarray(sim.data.joint(joint_name).qpos)[2])

        for body_name in _BOX_BODY_NAMES:
            body_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            if body_id >= 0:
                return float(sim.data.xpos[body_id, 2])

        msg = f"Could not find box joint/body using names {_BOX_JOINT_NAMES} / {_BOX_BODY_NAMES}."
        raise ValueError(msg)

    def _append_row(
        self,
        *,
        phase: str,
        step: int,
        commanded_gripper_width: float | None,
        reward: float | None = None,
        terminated: bool = False,
        truncated: bool = False,
    ) -> None:
        box_z = self._box_z()
        if self.reset_z is None:
            self.reset_z = box_z
        z_drop = self.reset_z - box_z
        row = {
            **self.metadata,
            "phase": phase,
            "step": step,
            "time_s": max(step, 0) / _CONTROL_FREQUENCY_HZ,
            "commanded_gripper_width": commanded_gripper_width,
            "box_z": box_z,
            "z_drop_from_reset": z_drop,
            "slipped": z_drop >= self.slip_threshold,
            "terminated": terminated,
            "truncated": truncated,
            "reward": reward,
        }
        with self.csv_path.open("a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.FIELDNAMES)
            if self._needs_header:
                writer.writeheader()
                self._needs_header = False
            writer.writerow(row)

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        obs, info = self.env.reset(seed=seed, options=options)
        self.step_idx = 0
        self.reset_z = None
        self._append_row(phase="reset", step=-1, commanded_gripper_width=None)
        return obs, info

    def step(self, action: Any):
        obs, reward, terminated, truncated, info = self.env.step(action)
        self._append_row(
            phase="step",
            step=self.step_idx,
            commanded_gripper_width=_extract_gripper_width(action),
            reward=float(reward),
            terminated=bool(terminated),
            truncated=bool(truncated),
        )
        self.step_idx += 1
        return obs, reward, terminated, truncated, info


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

        sim.model.opt.gravity[:] = 0
        obs, info = self.env.reset(seed=seed, options=options)

        robot = self.env.get_wrapper_attr("robot")
        gripper = self.env.get_wrapper_attr("gripper")
        if isinstance(robot, dict):
            robot = robot[_ROBOT_NAME]
        if isinstance(gripper, dict):
            gripper = gripper[_ROBOT_NAME]
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


def make_franka_sim_env_without_taxim(
    *,
    box_xml_path: Path,
    render_mode: str,
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
    cfg.sim_cfg.frequency = int(_CONTROL_FREQUENCY_HZ)
    cfg.wrapper_cfg.binary_gripper = True
    cfg.wrapper_cfg.home_on_reset = True
    cfg.max_relative_movement = np.deg2rad(5)
    cfg.relative_to = RelativeTo.LAST_STEP
    cfg.gripper_cfgs = {_ROBOT_NAME: _taxim_gripper_cfg()}
    cfg.gripper_offsets = None
    cfg.root_frame_objects = {
        "": (
            str(box_xml_path),
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
    if start_grasped:
        env = StartGraspedWrapper(
            env,
            settle_steps=grasp_settle_steps,
            post_gravity_settle_steps=post_gravity_settle_steps,
        )
    if render_mode == "human":
        env.get_wrapper_attr("sim").open_gui()
    return env


def make_ablation_env(
    *,
    box_xml_path: Path,
    args: argparse.Namespace,
    metadata: dict[str, Any],
    csv_path: Path,
) -> gym.Env:
    env = make_franka_sim_env_without_taxim(
        box_xml_path=box_xml_path,
        render_mode="human" if args.gui else "rgb_array",
        start_grasped=True,
        grasp_settle_steps=args.grasp_settle_steps,
        post_gravity_settle_steps=args.post_gravity_settle_steps,
    )
    return BoxZRecorderWrapper(
        env,
        csv_path=csv_path,
        metadata=metadata,
        slip_threshold=args.slip_threshold,
    )


def run_ablation(args: argparse.Namespace) -> None:
    output_dir = args.output_dir.expanduser().resolve()
    csv_path = output_dir / "box_z_records.csv"
    source_xml = Path(_BOX_XML)

    friction_grid = itertools.product(
        args.friction_slide_values,
        args.friction_spin_values,
        args.friction_roll_values,
    )
    run_idx = 0
    for friction_values, density, gripper_width in itertools.product(
        friction_grid,
        args.density_values,
        args.gripper_widths,
    ):
        friction = tuple(float(value) for value in friction_values)
        run_id = f"run_{run_idx:04d}"
        box_xml_path = _write_box_xml(
            source_xml=source_xml,
            friction=friction,
            density=float(density),
        )
        metadata = {
            "run_id": run_id,
            "seed": args.seed + run_idx,
            "friction_slide": friction[0],
            "friction_spin": friction[1],
            "friction_roll": friction[2],
            "density": float(density),
        }

        print(
            f"{run_id}: friction={_format_friction(friction)} "
            f"density={_format_float(float(density))} width={_format_float(float(gripper_width))}"
        )
        env = make_ablation_env(
            box_xml_path=box_xml_path,
            args=args,
            metadata=metadata,
            csv_path=csv_path,
        )
        try:
            env.reset(seed=args.seed + run_idx)
            action = _make_gripper_width_action(env.action_space, float(gripper_width))
            for _ in range(args.steps):
                _, _, terminated, truncated, _ = env.step(action)
                if args.stop_on_done and (terminated or truncated):
                    break
        finally:
            env.close()
        run_idx += 1

    print(f"Wrote {run_idx} ablation rollouts to {csv_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Ablate box friction/density and record z slippage.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output-dir", type=Path, default=Path("ablation_runs"))
    parser.add_argument("--steps", type=int, default=120)
    parser.add_argument("--slip-threshold", type=float, default=0.01, help="Meters of z-drop counted as slipped.")
    parser.add_argument("--stop-on-done", action="store_true")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--grasp-settle-steps", type=int, default=30)
    parser.add_argument("--post-gravity-settle-steps", type=int, default=10)

    parser.add_argument("--friction-slide-values", type=_float_list, default=[0.25, 0.5, 1.0, 2.0])
    parser.add_argument("--friction-spin-values", type=_float_list, default=[0.3])
    parser.add_argument("--friction-roll-values", type=_float_list, default=[0.1])
    parser.add_argument("--density-values", type=_float_list, default=[50.0])
    parser.add_argument("--gripper-widths", type=_float_list, default=[0.0, 0.25, 0.5, 0.75, 1.0])

    args = parser.parse_args()
    run_ablation(args)


if __name__ == "__main__":
    main()
