"""Eval mode end to end in simulation with the mock policy server (artificial delay) and the web buttons."""

from pathlib import Path

import pytest
from rcs_webui.server import App, make_server, serve_in_thread
from rcs_webui.session import EnvSpec, PolicySettings, SessionRunner, SessionSettings
from test_session import SMALL_CAMERAS, call, read_session, wait_for


@pytest.fixture(params=[0, 2], ids=["sync", "rtc"])
def eval_interface(tmp_path, request):
    settings = SessionSettings(
        data_root=tmp_path,
        mode="eval",
        instruction="pick",
        frequency=30.0,
        operator="none",
        policy=PolicySettings(
            client="mock", host="mock", port=0, options={"delay_s": 0.05, "horizon": 8}, rtc_delay=request.param
        ),
    )
    runner = SessionRunner(settings, EnvSpec(env="rcs/fr3", overrides=SMALL_CAMERAS))
    server = make_server(App(runner, tmp_path), "127.0.0.1", 0)
    serve_in_thread(server)
    runner.start()
    wait_for(lambda: runner.status()["state"] in ("ready", "error"))
    assert runner.status()["error"] is None
    yield runner, f"http://127.0.0.1:{server.server_address[1]}", request.param
    server.shutdown()
    runner.stop()


def test_eval_rollout_records_chunk_provenance(eval_interface):
    runner, base, rtc_delay = eval_interface
    assert runner.status()["policy"]["connected"]
    call(base, "/api/button", {"button": "A"})
    wait_for(lambda: runner.status()["episode_steps"] >= 30)
    call(base, "/api/button", {"button": "B"})
    wait_for(lambda: not runner.status()["episode_active"])

    # switch the policy server at runtime (no env restart)
    session_dir = runner.status()["session_dir"]
    call(base, "/api/policy", {"port": 1234, "options": {"delay_s": 0.0, "horizon": 4}, "rtc_delay": 0})
    wait_for(lambda: runner.status()["policy"]["port"] == 1234)
    assert runner.status()["session_dir"] == session_dir
    assert runner.status()["policy"]["connected"]

    rows = sorted(read_session(Path(session_dir)).to_pylist(), key=lambda r: r["step"])
    assert len(rows) >= 30
    assert rows[-1]["success"] is True
    ann = [r["annotations"] for r in rows]
    assert all(a["source"] == "policy" for a in ann)
    assert all(a["controller"] == "scripted_policy" for a in ann)  # the mock client is a scripted policy
    assert ann[0]["chunk_id"] == 0
    assert ann[0]["chunk_index"] == 0
    assert all(a["response_received"] >= a["request_sent"] for a in ann)
    switches = [a for a in ann if a["switch_step"] >= 0]
    assert len({a["chunk_id"] for a in switches}) >= 3
    if rtc_delay:
        # after the first chunk every chunk takes over at index rtc_delay (unless the answer was late)
        later = [a for a in ann if a["chunk_id"] > 0 and a["switch_step"] == rows[ann.index(a)]["step"]]
        assert later
        assert all(a["chunk_index"] >= rtc_delay for a in later)
    else:
        assert all(a["chunk_index"] < 8 for a in ann)


def test_eval_rejects_rtc_for_vlagents(eval_interface):
    _, base, _ = eval_interface
    import urllib.error

    with pytest.raises(urllib.error.HTTPError) as err:
        call(base, "/api/policy", {"client": "vlagents", "rtc_delay": 2})
    assert err.value.code == 400
