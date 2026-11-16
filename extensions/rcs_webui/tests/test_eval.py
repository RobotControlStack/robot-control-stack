"""Eval mode end to end in simulation with the mock policy server (artificial delay) and the web buttons."""

from pathlib import Path

import pyarrow as pa
import pytest
from rcs.operator.interface import BaseOperator, BaseOperatorConfig, TeleopCommands
from rcs_webui.server import App, make_server, serve_in_thread
from rcs_webui.session import QUEST_CONTROL_MODE, EnvSpec, PolicySettings, SessionRunner, SessionSettings
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


class _FakeQuest(BaseOperator):
    """Stands in for the Quest: the test sets the trigger (engaged) and presses buttons."""

    control_mode = QUEST_CONTROL_MODE
    controller_names = ["right"]  # noqa: RUF012

    def __init__(self):
        super().__init__(BaseOperatorConfig(operator_class=BaseOperator))
        self.engaged = False
        self.pending = TeleopCommands()

    def consume_commands(self) -> TeleopCommands:
        cmds, self.pending = self.pending, TeleopCommands()
        return cmds

    def consume_action(self):
        return {}  # hold the robot where it is

    def is_engaged(self) -> bool:
        return self.engaged


def test_intervention_lasts_until_x(tmp_path):
    """Trigger held: human; trigger released: still human (robot held) until X; then the policy continues."""
    settings = SessionSettings(
        data_root=tmp_path,
        mode="eval",
        frequency=30.0,
        operator="none",
        policy=PolicySettings(client="mock", host="mock", port=0, options={"horizon": 4}),
    )
    quest = _FakeQuest()
    # interventions need the Quest action space (eval keeps the env's default otherwise)
    quest_space = {
        "control_mode": "CARTESIAN_TQuat",
        "relative_to": "CONFIGURED_ORIGIN",
        "max_relative_movement": {"__tuple__": [0.5, 1.57]},
    }
    runner = SessionRunner(settings, EnvSpec(env="rcs/fr3", overrides={**SMALL_CAMERAS, **quest_space}), operator=quest)
    server = make_server(App(runner, tmp_path), "127.0.0.1", 0)
    serve_in_thread(server)
    runner.start()
    try:
        wait_for(lambda: runner.status()["state"] in ("ready", "error"))
        assert runner.status()["error"] is None
        base = f"http://127.0.0.1:{server.server_address[1]}"
        steps = lambda: runner.status()["episode_steps"]  # noqa: E731
        call(base, "/api/button", {"button": "A"})
        wait_for(lambda: steps() >= 5)
        quest.engaged = True
        wait_for(lambda: runner.status()["policy"]["intervening"])
        n = steps()
        wait_for(lambda: steps() >= n + 5)
        quest.engaged = False  # released: the human keeps the robot
        wait_for(lambda: runner.status()["policy"]["awaiting_resume"])
        n = steps()
        wait_for(lambda: steps() >= n + 5)
        assert runner.status()["policy"]["intervening"]
        call(base, "/api/button", {"button": "X"})
        wait_for(lambda: not runner.status()["policy"]["intervening"])
        n = steps()
        wait_for(lambda: steps() >= n + 5)
        call(base, "/api/button", {"button": "B"})
        wait_for(lambda: not runner.status()["episode_active"])
        session_dir = Path(runner.status()["session_dir"])
        wait_for(lambda: _rows(session_dir) >= 20)
        rows = sorted(read_session(session_dir).to_pylist(), key=lambda r: r["step"])
        ctrl = [r["annotations"]["controller"] for r in rows]
        # policy, then one block of human (trigger held + released until X), then policy again
        blocks = [c for i, c in enumerate(ctrl) if i == 0 or c != ctrl[i - 1]]
        assert blocks == ["scripted_policy", "human", "scripted_policy"]
        human = [r for r in rows if r["annotations"]["controller"] == "human"]
        assert len(human) >= 10
        assert all(r["annotations"]["source"] == "intervention" for r in human)
    finally:
        server.shutdown()
        runner.stop()


def _rows(session_dir: Path) -> int:
    try:
        return read_session(session_dir).num_rows
    except (FileNotFoundError, pa.ArrowInvalid):
        return 0
