import json
import subprocess

import numpy as np
import pytest
from rcs.envs.base import ControlMode
from rcs.envs.configs import EmptyWorldDroid, EmptyWorldFR3, EmptyWorldFR3Duo
from rcs_webui import snapshot


def _assert_close(a, b, path="cfg"):
    """Compares two serialized configs, poses may differ by float noise."""
    if isinstance(a, dict):
        assert isinstance(b, dict), path
        assert a.keys() == b.keys(), path
        for k in a:
            _assert_close(a[k], b[k], f"{path}.{k}")
    elif isinstance(a, list):
        assert len(a) == len(b), path
        for i, (x, y) in enumerate(zip(a, b, strict=True)):
            _assert_close(x, y, f"{path}[{i}]")
    elif isinstance(a, float):
        assert a == pytest.approx(b, abs=1e-9), path
    else:
        assert a == b, path


@pytest.mark.parametrize("creator", [EmptyWorldFR3(), EmptyWorldFR3Duo(), EmptyWorldDroid()])
def test_config_round_trip(creator):
    cfg = creator.config()
    text = snapshot.config_to_json(cfg)
    restored = snapshot.config_from_json(text)
    _assert_close(json.loads(text), json.loads(snapshot.config_to_json(restored)))
    assert type(restored) is type(cfg)
    assert restored.control_mode == cfg.control_mode
    name = next(iter(cfg.robot_cfgs))
    assert np.allclose(restored.robot_cfgs[name].q_home, cfg.robot_cfgs[name].q_home)
    assert restored.robot_cfgs[name].tcp_offset.is_close(cfg.robot_cfgs[name].tcp_offset)


def test_fr3_hardware_config_round_trip():
    configs = pytest.importorskip("rcs_fr3.configs")
    cfg = configs.DefaultFR3HardwareEnv().config()
    cfg.robot_cfg.record_control_states = True
    restored = snapshot.config_from_json(snapshot.config_to_json(cfg))
    assert restored.robot_cfg.ip == cfg.robot_cfg.ip
    assert restored.robot_cfg.record_control_states is True
    assert restored.gripper_cfg.force == cfg.gripper_cfg.force
    assert restored.camera_cfgs["realsense"].camera_cfgs["side"].identifier == "243522070385"


def test_overrides():
    cfg_json = snapshot.to_jsonable(EmptyWorldFR3().config())
    snapshot.apply_overrides(
        cfg_json,
        {
            "sim_cfg.frequency": 20,
            "control_mode": "JOINTS",
            "robot_cfgs.right.q_home": [0.0] * 7,
            "camera_cfgs.wrist.resolution_width": 64,
        },
    )
    cfg = snapshot.from_jsonable(cfg_json)
    assert cfg.sim_cfg.frequency == 20
    assert cfg.control_mode == ControlMode.JOINTS
    assert np.allclose(cfg.robot_cfgs["right"].q_home, 0)
    assert cfg.camera_cfgs["wrist"].resolution_width == 64
    with pytest.raises(KeyError):
        snapshot.apply_overrides(cfg_json, {"sim_cfg.does_not_exist": 1})


def test_git_state_logs_commit_and_diff(tmp_path):
    repo = tmp_path / "repo"
    repo.mkdir()
    git = ["git", "-C", str(repo), "-c", "user.name=t", "-c", "user.email=t@t"]
    subprocess.run([*git, "init", "-q"], check=True)
    (repo / "a.txt").write_text("a\n")
    subprocess.run([*git, "add", "a.txt"], check=True)
    subprocess.run([*git, "commit", "-q", "-m", "init"], check=True)
    (repo / "a.txt").write_text("b\n")
    (repo / "new.txt").write_text("new\n")
    state = snapshot.git_state(repo)
    assert state is not None
    assert len(state["commit"]) == 40
    assert "+b" in state["diff"]
    assert state["untracked"] == ["new.txt"]


def test_python_config_subclass_and_asset_paths():
    configs = pytest.importorskip("rcs_fr3.configs")
    pytest.importorskip("rcs_robotiq2f85")
    cfg = configs.DROIDEnv().config()
    cfg.gripper_cfgs["right"].serial_number = "ABC"
    data = snapshot.to_jsonable(cfg)
    robot = data["fields"]["robot_cfgs"]["right"]["fields"]
    # machine independent: asset paths are stored relative to the RCS asset prefix
    assert "__rcs_prefix__" in robot["kinematic_model_path"]
    restored = snapshot.from_jsonable(data)
    assert restored.robot_cfgs["right"].kinematic_model_path == cfg.robot_cfgs["right"].kinematic_model_path
    gripper = restored.gripper_cfgs["right"]
    assert type(gripper).__name__ == "RobotiQ2F85GripperConfig"
    assert gripper.serial_number == "ABC"
    assert gripper.async_control is True


def test_config_file_formats(tmp_path):
    """A bare config (old format) still loads together with the env, a wrapped one also without."""
    from rcs_webui import configs
    from rcs_webui.session import EnvSpec, SessionRunner, SessionSettings

    cfg_json = snapshot.to_jsonable(EmptyWorldFR3().config())
    bare = tmp_path / "bare.json"
    bare.write_text(json.dumps(cfg_json))
    wrapped = tmp_path / "wrapped.json"
    wrapped.write_text(json.dumps(configs.wrap("rcs.envs.configs:EmptyWorldFR3", cfg_json)))
    runner = SessionRunner(SessionSettings(data_root=tmp_path), EnvSpec())
    for spec in (
        EnvSpec(env="rcs/fr3", config_file=bare),
        EnvSpec(env="rcs/fr3", config_file=wrapped),
        EnvSpec(config_file=wrapped),
    ):
        cfg, name, _ = runner._resolve_config(spec)
        assert name == "rcs.envs.configs:EmptyWorldFR3"
        assert cfg.sim_cfg.frequency == EmptyWorldFR3().config().sim_cfg.frequency
    with pytest.raises(ValueError, match="no creator"):
        runner._resolve_config(EnvSpec(config_file=bare))
