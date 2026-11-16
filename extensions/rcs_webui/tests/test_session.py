"""End to end tests of the interface against the simulation (no Quest: the web buttons are used)."""

import json
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

import gymnasium as gym
import numpy as np
import pyarrow as pa
import pyarrow.dataset as ds
import pytest
from rcs_webui import snapshot
from rcs_webui.server import App, make_server, serve_in_thread
from rcs_webui.session import EnvSpec, SessionRunner, SessionSettings

SMALL_CAMERAS = {
    f"camera_cfgs.{cam}.{key}": value
    for cam in ("bird_eye", "wrist")
    for key, value in (("resolution_width", 64), ("resolution_height", 48))
}


def wait_for(predicate, timeout: float = 60.0, interval: float = 0.05):
    end = time.time() + timeout
    while time.time() < end:
        if predicate():
            return
        time.sleep(interval)
    msg = "condition not reached in time"
    raise TimeoutError(msg)


@pytest.fixture()
def interface(tmp_path):
    (tmp_path / "configs").mkdir()
    settings = SessionSettings(
        data_root=tmp_path / "data",
        mode="collect",
        instruction="pick",
        frequency=30.0,
        operator="none",
        config_dirs=[tmp_path / "configs"],
    )
    runner = SessionRunner(settings, EnvSpec(env="rcs/fr3", overrides=SMALL_CAMERAS))
    server = make_server(App(runner, tmp_path / "data"), "127.0.0.1", 0)
    serve_in_thread(server)
    runner.start()
    wait_for(lambda: runner.status()["state"] in ("ready", "error"))
    assert runner.status()["error"] is None
    yield runner, f"http://127.0.0.1:{server.server_address[1]}", tmp_path / "data"
    server.shutdown()
    runner.stop()


def call(base: str, path: str, body: dict | None = None):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(base + path, data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=30) as res:
        raw = res.read()
        return json.loads(raw) if res.headers.get_content_type() == "application/json" else raw


def record_episode(runner: SessionRunner, base: str, button: str, steps: int = 10):
    call(base, "/api/button", {"button": "A"})
    wait_for(lambda: runner.status()["episode_steps"] >= steps)
    call(base, "/api/button", {"button": button})
    wait_for(lambda: not runner.status()["episode_active"])


def read_session(session_dir: Path):
    # wait until the writer thread has flushed (it runs in the background)
    return ds.dataset(str(session_dir / snapshot.DATA_DIR), format="parquet").to_table()


def test_collect_records_episodes_with_snapshot(interface):
    runner, base, data_root = interface
    status = call(base, "/api/status")
    session_dir = Path(status["session_dir"])
    assert session_dir.parent == data_root
    for name in (snapshot.CONFIG_FILE, snapshot.SESSION_FILE, snapshot.GIT_FILE):
        assert (session_dir / name).exists()
    meta = json.loads((session_dir / snapshot.SESSION_FILE).read_text())
    assert meta["creator"] == "rcs.envs.configs:EmptyWorldFR3"
    git = json.loads((session_dir / snapshot.GIT_FILE).read_text())
    assert git["rcs"]["commit"]
    assert (session_dir / git["rcs"]["diff_file"]).exists()

    # live camera image
    assert call(base, "/api/live/wrist.jpg")[:2] == b"\xff\xd8"

    record_episode(runner, base, "B")
    record_episode(runner, base, "Y")
    # restart flushes and creates a new folder
    call(base, "/api/restart", {})
    wait_for(lambda: runner.status()["session_dir"] != str(session_dir) and runner.status()["state"] == "ready")
    new_dir = Path(runner.status()["session_dir"])
    assert new_dir != session_dir

    runner.wait_for_consolidation()
    table = read_session(session_dir)
    rows = table.to_pylist()
    episodes: dict[str, list[dict]] = {}
    for row in rows:
        episodes.setdefault(row["uuid"], []).append(row)
    assert len(episodes) == 2
    labels = []
    for ep_rows in episodes.values():
        ep_rows.sort(key=lambda r: r["step"])
        assert [r["step"] for r in ep_rows] == list(range(len(ep_rows)))
        assert len(ep_rows) >= 10
        labels.append(ep_rows[-1]["success"])
        assert all(r["instruction"] == "pick" for r in ep_rows)
        assert all(r["annotations"]["controller"] == "human" for r in ep_rows)  # teleop
        ts = [r["timestamp"] for r in ep_rows]
        assert np.all(np.diff(ts) > 0)
        states = ep_rows[-1]["info"]["right"]["control_states"]
        n = states["num_samples"]
        # 30 Hz env with a 2 ms physics step
        assert n == round(1 / 30 / 0.002)
        assert np.asarray(states["q"]).reshape(states["q_shape"]).shape == (n, 7)
        assert "wrist" in ep_rows[-1]["obs"]["frames"]
    assert sorted(labels) == [False, True]


def test_restart_from_recording_uses_stored_config(interface):
    runner, base, _ = interface
    first = Path(runner.status()["session_dir"])
    call(base, "/api/restart", {"from_recording": first.name, "overrides": {"sim_cfg.frequency": 20}})
    wait_for(lambda: runner.status()["session_dir"] != str(first) and runner.status()["state"] == "ready")
    second = Path(runner.status()["session_dir"])
    cfg, meta = snapshot.load_session_config(second)
    assert cfg.sim_cfg.frequency == 20
    assert meta["source"]["from_recording"] == str(first)
    # everything else is taken over
    cfg_first, _ = snapshot.load_session_config(first)
    assert cfg.camera_cfgs["wrist"].resolution_width == cfg_first.camera_cfgs["wrist"].resolution_width == 64


def test_viewer_relabel_and_delete(interface):
    runner, base, _ = interface
    record_episode(runner, base, "B")
    record_episode(runner, base, "Y")
    session = Path(runner.status()["session_dir"]).name
    # an active episode can not be edited
    call(base, "/api/button", {"button": "A"})
    wait_for(lambda: runner.status()["episode_steps"] >= 2)
    active = runner.status()["episode_uuid"]
    with pytest.raises(urllib.error.HTTPError) as err:
        call(base, f"/api/sessions/{session}/episodes/{active}/delete", {})
    assert err.value.code == 409
    call(base, "/api/button", {"button": "Y"})
    wait_for(lambda: not runner.status()["episode_active"])
    wait_for(lambda: len(call(base, f"/api/sessions/{session}/episodes")["episodes"]) == 3)

    episodes = call(base, f"/api/sessions/{session}/episodes")["episodes"]
    assert [e["success"] for e in episodes] == [True, False, False]
    first, second, third = (e["uuid"] for e in episodes)

    ep = call(base, f"/api/sessions/{session}/episodes/{first}")
    assert ep["steps"] == episodes[0]["steps"]
    assert episodes[0]["controllers"] == {"human": episodes[0]["steps"]}
    assert episodes[0]["intervention_steps"] == 0
    assert ep["controller"] == ["human"] * ep["steps"]
    assert ep["intervention"] == [False] * ep["steps"]
    assert ep["cameras"] == ["bird_eye", "wrist"]
    robot = ep["robots"]["right"]
    assert len(robot["step"]["joints"]) == ep["steps"]
    assert len(robot["step"]["joints"][0]) == 7
    assert "env_action.tquat" in robot["step"]
    assert robot["control"]["num_samples"] == ep["steps"] * round(1 / 30 / 0.002)
    assert len(robot["control"]["tau_J"][0]) == 7
    jpeg = call(base, f"/api/sessions/{session}/episodes/{first}/frames/wrist/3.jpg")
    assert jpeg[:2] == b"\xff\xd8"

    call(base, f"/api/sessions/{session}/episodes/{first}/label", {"success": False})
    call(base, f"/api/sessions/{session}/episodes/{second}/label", {"success": True})
    call(base, f"/api/sessions/{session}/episodes/{third}/delete", {})
    episodes = call(base, f"/api/sessions/{session}/episodes")["episodes"]
    assert [(e["uuid"], e["success"]) for e in episodes] == [(first, False), (second, True)]
    # only the last row carries the label, like the recorder writes it
    rows = [r for r in read_session(Path(runner.status()["session_dir"])).to_pylist() if r["uuid"] == second]
    rows.sort(key=lambda r: r["step"])
    assert [r["success"] for r in rows] == [False] * (len(rows) - 1) + [True]

    # viewer only mode
    viewer = make_server(App(None, Path(runner.status()["session_dir"]).parent), "127.0.0.1", 0)
    serve_in_thread(viewer)
    try:
        vbase = f"http://127.0.0.1:{viewer.server_address[1]}"
        assert call(vbase, "/api/status")["state"] == "viewer"
        assert len(call(vbase, f"/api/sessions/{session}/episodes")["episodes"]) == 2
    finally:
        viewer.shutdown()


def test_config_catalog_edit_and_start(interface):
    runner, base, data_root = interface
    config_dir = data_root.parent / "configs"
    first = Path(runner.status()["session_dir"])
    entries = {e["id"]: e for e in call(base, "/api/configs")["configs"]}
    assert "default:rcs/fr3" in entries
    assert f"recording:{first.name}" in entries

    # a config file in the config dir, e.g. made with rcs-webui dump-config
    recorded = call(base, "/api/configs/entry?id=" + urllib.parse.quote(f"recording:{first.name}"))
    assert recorded["creator"] == "rcs.envs.configs:EmptyWorldFR3"
    (config_dir / "small.json").write_text(json.dumps(recorded))
    entries = {e["id"]: e for e in call(base, "/api/configs")["configs"]}
    file_id = next(i for i in entries if i.startswith("file:") and i.endswith("small.json"))
    cfg = call(base, "/api/configs/entry?id=" + urllib.parse.quote(file_id))

    # edit and validate
    cfg["config"]["fields"]["sim_cfg"]["fields"]["frequency"] = 20.0
    assert call(base, "/api/configs/validate", cfg)["ok"]
    broken = json.loads(json.dumps(cfg))
    broken["config"]["fields"]["control_mode"]["name"] = "NOT_A_MODE"
    assert not call(base, "/api/configs/validate", broken)["ok"]
    with pytest.raises(urllib.error.HTTPError) as err:
        call(base, "/api/restart", {"config": broken})
    assert err.value.code == 400

    call(base, "/api/restart", {"config": cfg})
    wait_for(lambda: runner.status()["session_dir"] != str(first) and runner.status()["state"] == "ready")
    second = Path(runner.status()["session_dir"])
    stored, meta = snapshot.load_session_config(second)
    assert stored.sim_cfg.frequency == 20.0
    assert meta["creator"] == "rcs.envs.configs:EmptyWorldFR3"


@pytest.fixture()
def idle_interface(tmp_path):
    """The interface as `rcs-webui serve` starts it: no env until it is started from the page."""
    (tmp_path / "configs").mkdir()
    settings = SessionSettings.load(tmp_path / "data")
    settings.update({"operator": "none", "config_dirs": [str(tmp_path / "configs")], "frequency": 30.0})
    runner = SessionRunner(settings)
    server = make_server(App(runner, tmp_path / "data"), "127.0.0.1", 0)
    serve_in_thread(server)
    runner.start()
    yield runner, f"http://127.0.0.1:{server.server_address[1]}", tmp_path
    server.shutdown()
    runner.stop()


def test_env_starts_only_from_the_page(idle_interface):
    runner, base, tmp_path = idle_interface
    time.sleep(0.3)
    status = call(base, "/api/status")
    assert status["state"] == "idle"
    assert status["env"] is None
    assert not (tmp_path / "data").exists() or not any(
        (d / snapshot.CONFIG_FILE).exists() for d in (tmp_path / "data").iterdir()
    )

    # pick a default config, edit it like the page does and start it
    catalog = call(base, "/api/configs")
    assert "running:" not in {e["id"] for e in catalog["configs"]}
    entry = call(base, "/api/configs/entry?id=" + urllib.parse.quote("default:rcs/fr3"))
    assert entry["summary"]["platform"] == "simulation"
    cfg = {"creator": entry["creator"], "config": entry["config"]}
    snapshot.apply_overrides(cfg["config"], {**SMALL_CAMERAS, "sim_cfg.frequency": 30.0})
    call(base, "/api/restart", {"config": cfg, "base_id": "default:rcs/fr3"})
    wait_for(lambda: runner.status()["state"] in ("ready", "error"))
    assert runner.status()["error"] is None
    first = Path(runner.status()["session_dir"])
    _, meta = snapshot.load_session_config(first)
    assert meta["source"]["base"] == "default:rcs/fr3"
    assert meta["source"]["changed"]["camera_cfgs.wrist.resolution_width"] == 64
    assert runner.status()["summary"]["cameras"] == ["bird_eye", "wrist"]

    # the running config is offered and is the complete config of the env
    catalog = call(base, "/api/configs")
    assert catalog["configs"][0]["id"] == "running:"
    assert catalog["last"] == "default:rcs/fr3"
    running = call(base, "/api/configs/entry?id=running:")
    assert running["config"]["fields"]["camera_cfgs"]["wrist"]["fields"]["resolution_width"] == 64

    # env start settings are locked while the env runs, the others are not
    with pytest.raises(urllib.error.HTTPError) as err:
        call(base, "/api/settings", {"mode": "eval"})
    assert err.value.code == 409
    call(base, "/api/settings", {"instruction": "stack"})
    wait_for(lambda: runner.status()["instruction"] == "stack")

    call(base, "/api/stop_env", {})
    wait_for(lambda: runner.status()["state"] == "idle")
    assert runner.status()["env"] is None
    call(base, "/api/settings", {"mode": "eval"})
    wait_for(lambda: runner.status()["mode"] == "eval")

    # settings survive a restart of the interface
    stored = SessionSettings.load(tmp_path / "data")
    assert stored.mode == "eval"
    assert stored.instruction == "stack"
    assert stored.last_config == "default:rcs/fr3"
    assert stored.config_dirs == [tmp_path / "configs"]


def test_bare_config_file_in_config_dir(idle_interface):
    runner, base, tmp_path = idle_interface
    cfg_json = snapshot.apply_overrides(
        snapshot.to_jsonable(__import__("rcs.envs.configs", fromlist=["EmptyWorldFR3"]).EmptyWorldFR3().config()),
        {**SMALL_CAMERAS, "headless": True},  # the raw default opens the MuJoCo viewer
    )
    (tmp_path / "configs" / "bare.json").write_text(json.dumps(cfg_json))
    entries = call(base, "/api/configs")["configs"]
    file_id = next(e["id"] for e in entries if e["id"].endswith("bare.json"))
    entry = call(base, "/api/configs/entry?id=" + urllib.parse.quote(file_id))
    assert entry["creator"] == "rcs_webui.envs:ConfigSimEnvCreator"
    call(base, "/api/restart", {"config": {"creator": entry["creator"], "config": entry["config"]}, "base_id": file_id})
    wait_for(lambda: runner.status()["state"] in ("ready", "error"))
    assert runner.status()["error"] is None
    record_episode(runner, base, "B", steps=3)


def test_duobench_pick_place_is_offered(idle_interface):
    pytest.importorskip("duobench.tasks.pick_place")
    _, base, _ = idle_interface
    ids = {e["id"] for e in call(base, "/api/configs")["configs"]}
    assert "default:duobench/pick_place_red_on_green" in ids
    entry = call(base, "/api/configs/entry?id=" + urllib.parse.quote("default:duobench/pick_place_red_on_green"))
    summary = entry["summary"]
    assert summary["task"] == "pick_place_red_on_green"
    assert summary["cameras"] == ["front_rgb", "gripper_rgb"]
    assert summary["control_mode"] == "CARTESIAN_TQuat"
    assert summary["relative_to"] == "CONFIGURED_ORIGIN"


class _SucceedAfter(gym.Wrapper):
    """Ends the episode like a task check does: terminated with info["success"] after ``n`` steps."""

    def __init__(self, env, n: int, success: bool = True):
        super().__init__(env)
        self.n, self.count, self.success_value = n, 0, success

    def reset(self, **kwargs):
        self.count = 0
        return self.env.reset(**kwargs)

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        self.count += 1
        if self.count >= self.n:
            terminated = True
            info = {**info, "success": self.success_value}
        return obs, reward, terminated, truncated, info


@pytest.mark.parametrize("success", [True, False])
def test_env_task_check_labels_the_episode(interface, success):
    """A task that terminates the episode with info["success"] labels it like B; without success like Y."""
    runner, base, _ = interface
    storage = runner.storage
    # the task check sits below the StorageWrapper, as with duobench's TaskStageWrapper
    storage.env = _SucceedAfter(storage.env, 8, success)
    call(base, "/api/button", {"button": "A"})
    wait_for(lambda: runner.status()["episodes"]["successful"] + runner.status()["episodes"]["truncated"] == 1)
    status = runner.status()
    assert status["episodes"] == {
        "successful": int(success),
        "unsuccessful": 0,
        "truncated": int(not success),
    }
    assert (status["error"] is None) == success
    session_dir = Path(status["session_dir"])

    def flushed() -> bool:
        try:
            return read_session(session_dir).num_rows >= 8
        except (FileNotFoundError, pa.ArrowInvalid):  # the writer thread has not finished the file yet
            return False

    wait_for(flushed)
    rows = sorted(read_session(session_dir).to_pylist(), key=lambda r: r["step"])
    assert [r["success"] for r in rows] == [False] * (len(rows) - 1) + [success]
    assert rows[-1]["terminated"] is True


def test_auto_reset_off_keeps_recording_until_the_operator_ends(interface):
    """With auto reset off the env's end of episode is only reported, the episode ends with B / Y."""
    runner, base, _ = interface
    call(base, "/api/settings", {"auto_reset": False})
    wait_for(lambda: runner.settings.auto_reset is False)
    runner.storage.env = _SucceedAfter(runner.storage.env, 5, True)
    call(base, "/api/button", {"button": "A"})
    wait_for(lambda: runner.status()["env_end"] == "task success")
    wait_for(lambda: runner.status()["episode_steps"] >= 12)  # still recording after the env's end
    assert runner.status()["episode_active"]
    assert runner.status()["episodes"]["successful"] == 0
    call(base, "/api/button", {"button": "Y"})
    wait_for(lambda: not runner.status()["episode_active"])
    status = runner.status()
    assert status["env_end"] is None
    assert status["episodes"] == {"successful": 0, "unsuccessful": 1, "truncated": 0}
    session_dir = Path(status["session_dir"])

    def flushed() -> bool:
        try:
            return read_session(session_dir).num_rows >= 12
        except (FileNotFoundError, pa.ArrowInvalid):
            return False

    wait_for(flushed)
    rows = read_session(session_dir).to_pylist()
    assert not any(r["success"] for r in rows)  # the operator's Y wins over the env's success
    assert sum(r["terminated"] for r in rows) >= 7  # the env kept reporting the end


def test_client_that_disconnects_is_not_an_error(interface, caplog):
    """A browser that drops a camera image request (page re-render) must not produce error tracebacks."""
    import logging
    import socket

    runner, base, _ = interface
    host, port = base.removeprefix("http://").split(":")
    caplog.set_level(logging.DEBUG, logger="rcs_webui.server")
    for _ in range(20):
        sock = socket.create_connection((host, int(port)))
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, b"\x01\x00\x00\x00\x00\x00\x00\x00")  # RST on close
        sock.sendall(b"GET /api/live/wrist.jpg HTTP/1.1\r\nHost: x\r\n\r\n")
        sock.close()
    time.sleep(0.5)
    assert call(base, "/api/status")["state"] == "ready"
    assert not [r for r in caplog.records if r.levelno >= logging.ERROR], [r.getMessage() for r in caplog.records]


def test_stopped_session_is_consolidated_in_the_background(interface):
    """Stopping the env merges the session's small batch files into large parquet files, rows unchanged."""
    runner, base, _ = interface
    record_episode(runner, base, "B", steps=40)
    record_episode(runner, base, "Y", steps=40)
    session_dir = Path(runner.status()["session_dir"])
    data_dir = session_dir / snapshot.DATA_DIR
    wait_for(lambda: len(list(data_dir.glob("*.parquet"))) >= 3)  # batches of 32 rows
    call(base, "/api/stop_env", {})
    wait_for(lambda: runner.status()["state"] == "idle")
    runner.wait_for_consolidation()
    assert runner.status()["consolidating"] == []
    assert len(list(data_dir.glob("*.parquet"))) == 1
    assert not list(session_dir.glob("data.*"))  # no temporary folders left
    rows = read_session(session_dir).to_pylist()
    episodes: dict[str, list[dict]] = {}
    for row in rows:
        episodes.setdefault(row["uuid"], []).append(row)
    assert len(episodes) == 2
    for ep in episodes.values():
        assert [r["step"] for r in ep] == list(range(len(ep)))  # whole and in order
    assert sorted(ep[-1]["success"] for ep in episodes.values()) == [False, True]


class _ReflexOnce(gym.Wrapper):
    """Reports one robot error (like FR3HW on a reflex: truncated, message in info.<robot>.robot_error)."""

    def __init__(self, env, at: int):
        super().__init__(env)
        self.at, self.count = at, 0

    def reset(self, **kwargs):
        self.count = 0
        return self.env.reset(**kwargs)

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        self.count += 1
        if self.count == self.at:
            info = {**info, "right": {**info.get("right", {}), "robot_error": "reflex: joint_velocity_violation"}}
            truncated = True
        return obs, reward, terminated, truncated, info


@pytest.mark.parametrize("mode", ["recover", "end"])
def test_robot_error_recover_or_end(interface, mode):
    runner, base, _ = interface
    call(base, "/api/settings", {"on_robot_error": mode})
    wait_for(lambda: runner.settings.on_robot_error == mode)
    recoveries = []
    robot_env = runner.env.get_wrapper_attr("envs")["right"]
    robot_env.recover = lambda: recoveries.append(1)  # FR3HW.recover on hardware
    runner.storage.env = _ReflexOnce(runner.storage.env, at=5)
    call(base, "/api/button", {"button": "A"})
    if mode == "recover":
        wait_for(lambda: runner.status()["episode_steps"] >= 15)
        assert runner.status()["episode_active"]
        assert recoveries == [1]
        assert runner.status()["robot_errors_recovered"] == 1
        call(base, "/api/button", {"button": "B"})
        wait_for(lambda: not runner.status()["episode_active"])
        expected = {"successful": 1, "unsuccessful": 0, "truncated": 0}
    else:
        wait_for(lambda: not runner.status()["episode_active"] and runner.status()["episodes"]["truncated"] == 1)
        assert recoveries == []
        expected = {"successful": 0, "unsuccessful": 0, "truncated": 1}
    assert runner.status()["episodes"] == expected
    session_dir = Path(runner.status()["session_dir"])

    def flushed() -> bool:
        try:
            return read_session(session_dir).num_rows >= 5
        except (FileNotFoundError, pa.ArrowInvalid):
            return False

    wait_for(flushed)
    rows = sorted(read_session(session_dir).to_pylist(), key=lambda r: r["step"])
    assert [r["step"] for r in rows if r["truncated"]] == [4]  # the error row is kept in both modes
    if mode == "recover":
        assert len(rows) >= 15 and rows[-1]["success"] is True
