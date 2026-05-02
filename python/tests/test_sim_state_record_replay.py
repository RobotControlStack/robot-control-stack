from __future__ import annotations

import importlib.util
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import gymnasium as gym
import mujoco as mj
import numpy as np
import pyarrow.dataset as ds
from rcs._core.common import RobotPlatform
from rcs._core.sim import SimConfig
from rcs.camera.interface import CameraFrame, DataFrame, Frame, FrameSet
from rcs.envs.base import ControlMode, JointsDictType
from rcs.envs.creators import SimMultiEnvCreator
from rcs.envs.storage_wrapper import StorageWrapper
from rcs.envs.utils import default_sim_gripper_cfg, default_sim_robot_cfg

import rcs

REPO_ROOT = Path(__file__).resolve().parents[2]


def _load_local_module(module_name: str, relative_path: str):
    module_path = REPO_ROOT / relative_path
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        msg = f"Could not create an import spec for {module_name} from {module_path}."
        raise ImportError(msg)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    parent_name, _, child_name = module_name.rpartition(".")
    if parent_name:
        parent_module = sys.modules[parent_name]
        setattr(parent_module, child_name, module)
    spec.loader.exec_module(module)
    return module


local_sim_module = _load_local_module("rcs.sim.sim", "python/rcs/sim/sim.py")
rcs.sim.__dict__["Sim"] = local_sim_module.Sim
_load_local_module("rcs.envs.sim", "python/rcs/envs/sim.py")
_load_local_module("rcs.sim_state_replay", "python/rcs/sim_state_replay.py")

from rcs.envs.sim import SimStateObservationWrapper  # noqa: E402
from rcs.sim.sim import Sim  # noqa: E402
from rcs.sim_state_replay import (  # noqa: E402
    load_trajectory,
    replay_trajectory,
    restore_sim_step,
)

XML = """
<mujoco>
  <worldbody>
    <camera name="main" pos="1 0 0.7" xyaxes="0 1 0 -0.5 0 1"/>
    <body name="box" pos="0 0 0.1">
      <freejoint name="box_free"/>
      <geom type="box" size="0.05 0.05 0.05" rgba="0.2 0.6 0.9 1"/>
    </body>
  </worldbody>
</mujoco>
"""


@dataclass
class DummyCameraSet:
    sim: Sim

    def get_latest_frames(self) -> FrameSet:
        color_value = int(np.clip(round((self.sim.data.qpos[0] + 1.0) * 80.0), 0, 255))
        rgb = np.full((8, 8, 3), color_value, dtype=np.uint8)
        return FrameSet(
            frames={
                "main": Frame(
                    camera=CameraFrame(
                        color=DataFrame(data=rgb),
                        depth=None,
                    ),
                )
            },
            avg_timestamp=None,
        )


class DummySimEnv(gym.Env):
    PLATFORM = RobotPlatform.SIMULATION

    def __init__(self, sim: Sim, camera_set: DummyCameraSet | None = None):
        super().__init__()
        self.sim = sim
        self.camera_set = camera_set
        self.action_space = gym.spaces.Dict(
            {
                "delta": gym.spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float64),
            }
        )
        self.observation_space = gym.spaces.Dict(
            {
                "qpos": gym.spaces.Box(low=-np.inf, high=np.inf, shape=(self.sim.model.nq,), dtype=np.float64),
                "qvel": gym.spaces.Box(low=-np.inf, high=np.inf, shape=(self.sim.model.nv,), dtype=np.float64),
            }
        )

    def _obs(self) -> dict[str, np.ndarray]:
        return {
            "qpos": self.sim.data.qpos.copy(),
            "qvel": self.sim.data.qvel.copy(),
        }

    def get_wrapper_attr(self, name: str):
        return getattr(self, name)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        mj.mj_resetData(self.sim.model, self.sim.data)
        mj.mj_forward(self.sim.model, self.sim.data)
        return self._obs(), {}

    def step(self, action: dict[str, np.ndarray]):
        self.sim.data.qpos[0] += float(action["delta"][0])
        self.sim.data.qvel[:] = 0.0
        mj.mj_forward(self.sim.model, self.sim.data)
        return self._obs(), 0.0, False, False, {}

    def close(self):
        return None


def test_record_and_replay_sim_state(tmp_path: Path):
    model_path = tmp_path / "dummy.xml"
    model_path.write_text(XML)

    dataset_path = tmp_path / "dataset"
    record_env: gym.Env = DummySimEnv(Sim(model_path))
    record_env = SimStateObservationWrapper(record_env)
    record_env = StorageWrapper(record_env, str(dataset_path), "test sim replay", batch_size=1, always_record=True)

    obs, _ = record_env.reset()
    assert SimStateObservationWrapper.DYNAMIC_JOINT_SCHEMA_KEY in obs

    record_env.step({"delta": np.array([0.125], dtype=np.float64)})
    record_env.close()

    table = ds.dataset(str(dataset_path), format="parquet").to_table().sort_by([("step", "ascending")])
    rows = table.to_pylist()
    assert len(rows) == 1

    recorded_obs = rows[0]["obs"]
    assert SimStateObservationWrapper.DYNAMIC_JOINT_SCHEMA_KEY in recorded_obs
    assert SimStateObservationWrapper.DYNAMIC_JOINT_QPOS_KEY in recorded_obs
    assert SimStateObservationWrapper.DYNAMIC_JOINT_QVEL_KEY in recorded_obs

    recorded_steps = load_trajectory(dataset_path, rows[0]["uuid"], prefer_duckdb=True)
    assert len(recorded_steps) == 1
    assert recorded_steps[0].dynamic_joint_schema is not None
    assert np.allclose(
        recorded_steps[0].dynamic_joint_state["qpos"],  # type: ignore[index]
        np.asarray(recorded_obs[SimStateObservationWrapper.DYNAMIC_JOINT_QPOS_KEY]),
    )

    replay_sim = Sim(model_path)
    replay_env: gym.Env = DummySimEnv(replay_sim, camera_set=DummyCameraSet(replay_sim))
    replay_env = SimStateObservationWrapper(replay_env)
    render_dir = tmp_path / "rendered"

    replay_env.reset()
    restore_sim_step(replay_env, recorded_steps[0], dynamic_joint_schema=recorded_steps[0].dynamic_joint_schema)
    assert np.allclose(
        replay_env.get_wrapper_attr("sim").data.qpos, np.asarray(recorded_obs["qpos"]), atol=1e-9, rtol=0
    )
    assert np.allclose(
        replay_env.get_wrapper_attr("sim").data.qvel, np.asarray(recorded_obs["qvel"]), atol=1e-9, rtol=0
    )

    replay_trajectory(replay_env, recorded_steps, output_dir=render_dir)

    rendered_files = sorted(path.name for path in render_dir.glob("*.png"))
    assert rendered_files == ["step-000000-main.png"]


def _write_scene_with_extra_fixed_body_and_camera(src: Path, dst: Path):
    tree = ET.parse(src)
    root = tree.getroot()
    for include in root.findall("include"):
        include_file = include.get("file")
        if include_file is not None and not Path(include_file).is_absolute():
            include.set("file", str((src.parent / include_file).resolve()))

    worldbody = root.find("worldbody")
    assert worldbody is not None

    worldbody.append(
        ET.Element(
            "camera",
            {
                "name": "replay_extra_cam",
                "pos": "1.4 0.0 0.9",
                "xyaxes": "0 1 0 -0.3 0 1",
            },
        )
    )
    body = ET.SubElement(worldbody, "body", {"name": "replay_extra_bg", "pos": "3 3 3"})
    ET.SubElement(body, "geom", {"name": "replay_extra_bg_geom", "type": "box", "size": "0.1 0.1 0.1"})
    tree.write(dst)


def _record_dummy_trajectory(dataset_path: Path, model_path: Path) -> tuple[list, dict[str, object]]:
    record_env: gym.Env = DummySimEnv(Sim(model_path))
    record_env = SimStateObservationWrapper(record_env)
    record_env = StorageWrapper(record_env, str(dataset_path), "test sim replay", batch_size=1, always_record=True)
    record_env.reset()
    record_env.step({"delta": np.array([0.125], dtype=np.float64)})
    record_env.close()

    table = ds.dataset(str(dataset_path), format="parquet").to_table().sort_by([("step", "ascending")])
    rows = table.to_pylist()
    recorded_steps = load_trajectory(dataset_path, rows[0]["uuid"], prefer_duckdb=True)
    return recorded_steps, rows[0]["obs"]


def test_dynamic_joint_replay_tolerates_added_and_removed_fixed_scene_elements(tmp_path: Path):
    base_model_path = tmp_path / "base.xml"
    base_model_path.write_text(XML)
    modified_model_path = tmp_path / "modified.xml"
    _write_scene_with_extra_fixed_body_and_camera(base_model_path, modified_model_path)

    for record_model_path, replay_model_path in (
        (base_model_path, modified_model_path),
        (modified_model_path, base_model_path),
    ):
        dataset_path = tmp_path / f"dataset-{record_model_path.stem}-to-{replay_model_path.stem}"
        recorded_steps, recorded_obs = _record_dummy_trajectory(dataset_path, record_model_path)

        replay_sim = Sim(replay_model_path)
        replay_env: gym.Env = DummySimEnv(replay_sim)
        replay_env = SimStateObservationWrapper(replay_env)
        replay_env.reset()
        dynamic_joint_schema = next(
            step.dynamic_joint_schema for step in recorded_steps if step.dynamic_joint_schema is not None
        )
        restore_sim_step(replay_env, recorded_steps[0], dynamic_joint_schema=dynamic_joint_schema)

        assert np.allclose(
            replay_env.get_wrapper_attr("sim").data.qpos, np.asarray(recorded_obs["qpos"]), atol=1e-9, rtol=0
        )
        assert np.allclose(
            replay_env.get_wrapper_attr("sim").data.qvel, np.asarray(recorded_obs["qvel"]), atol=1e-9, rtol=0
        )


DUAL_ARM_ROBOT2ID = {"left": "0", "right": "1"}


def _create_dual_arm_env(scene_name: str):
    robot_cfg = default_sim_robot_cfg(scene_name, idx="")
    sim_cfg = SimConfig()
    sim_cfg.async_control = False
    return SimMultiEnvCreator()(
        name2id=DUAL_ARM_ROBOT2ID,
        robot_cfg=robot_cfg,
        control_mode=ControlMode.JOINTS,
        gripper_cfg=default_sim_gripper_cfg(idx=""),
        sim_cfg=sim_cfg,
        max_relative_movement=None,
    )


def test_dynamic_joint_state_roundtrip_on_fr3_dual_arm_scene(tmp_path: Path):
    source_scene_path = REPO_ROOT / "assets/scenes/fr3_dual_arm/scene.xml"
    source_robot_path = REPO_ROOT / "assets/scenes/fr3_empty_world/robot.xml"
    source_urdf_path = REPO_ROOT / "assets/scenes/fr3_empty_world/robot.urdf"
    modified_scene_path = source_scene_path.parent / "scene_dynamic_joint_test.xml"
    _write_scene_with_extra_fixed_body_and_camera(source_scene_path, modified_scene_path)

    base_scene_name = "fr3_dual_arm_dynamic_joint_base_test"
    test_scene_name = "fr3_dual_arm_dynamic_joint_test"
    scene_kwargs = {
        "mjcf_robot": str(source_robot_path),
        "urdf": str(source_urdf_path),
        "robot_type": rcs.scenes["fr3_dual_arm"].robot_type,
        "mjb": None,
    }
    rcs.scenes[base_scene_name] = rcs.Scene(mjcf_scene=str(source_scene_path), **scene_kwargs)
    rcs.scenes[test_scene_name] = rcs.Scene(mjcf_scene=str(modified_scene_path), **scene_kwargs)

    base_env = _create_dual_arm_env(base_scene_name)
    modified_env = _create_dual_arm_env(test_scene_name)
    try:
        base_env.reset()
        base_sim = base_env.get_wrapper_attr("sim")
        dynamic_joint_schema = base_sim.get_dynamic_joint_schema()
        dynamic_joint_state = base_sim.get_dynamic_joint_state()

        modified_env.reset()
        modified_sim = modified_env.get_wrapper_attr("sim")
        modified_sim.set_dynamic_joint_state(dynamic_joint_schema, dynamic_joint_state)
        restored_dynamic_joint_state = modified_sim.get_dynamic_joint_state()

        assert dynamic_joint_schema == modified_sim.get_dynamic_joint_schema()
        assert np.allclose(restored_dynamic_joint_state["qpos"], dynamic_joint_state["qpos"], atol=1e-9, rtol=0)
        assert np.allclose(restored_dynamic_joint_state["qvel"], dynamic_joint_state["qvel"], atol=1e-9, rtol=0)
    finally:
        base_env.close()
        modified_env.close()
        del rcs.scenes[test_scene_name]
        del rcs.scenes[base_scene_name]
        modified_scene_path.unlink(missing_ok=True)
