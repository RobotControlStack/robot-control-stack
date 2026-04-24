import multiprocessing
from pathlib import Path
from uuid import uuid4

REPO_ROOT = Path(__file__).resolve().parents[2]


def _gui_server_stop_then_step():
    import os

    os.chdir(REPO_ROOT)
    from rcs.sim import Sim

    sim = Sim(Path("assets/scenes/empty_world/scene.xml"))
    sim._gui_uuid = "rcs_test_" + str(uuid4())
    sim._start_gui_server(sim._gui_uuid)
    sim.step(2)
    sim.close_gui()
    sim.step(1)


def test_gui_server_can_be_stopped_before_later_steps():
    ctx = multiprocessing.get_context("spawn")
    proc = ctx.Process(target=_gui_server_stop_then_step)
    proc.start()
    proc.join(timeout=10)
    assert proc.exitcode == 0, f"process exited with {proc.exitcode}"
