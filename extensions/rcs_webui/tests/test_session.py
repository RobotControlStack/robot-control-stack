"""End to end tests of the interface against the simulation (no Quest: the web buttons are used)."""

import json
import time
import urllib.error
import urllib.request
from pathlib import Path

import numpy as np
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
    settings = SessionSettings(data_root=tmp_path, mode="collect", instruction="pick", frequency=30.0, operator="none")
    runner = SessionRunner(settings, EnvSpec(env="rcs/fr3", overrides=SMALL_CAMERAS))
    server = make_server(App(runner, tmp_path), "127.0.0.1", 0)
    serve_in_thread(server)
    runner.start()
    wait_for(lambda: runner.status()["state"] in ("ready", "error"))
    assert runner.status()["error"] is None
    yield runner, f"http://127.0.0.1:{server.server_address[1]}", tmp_path
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
