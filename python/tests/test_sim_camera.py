import math

import mujoco
import numpy as np
from rcs.camera.sim import SimCameraSet
from rcs.envs.configs import EmptyWorldDroid
from rcs.sim import SimConfig

from rcs import sim

STATE_SPEC = mujoco.mjtState.mjSTATE_INTEGRATION


def _render(camera_cfgs) -> dict[str, np.ndarray]:
    scene = EmptyWorldDroid()
    cfg = scene.prefixed_cfg(scene.config())
    assert cfg.camera_cfgs is not None
    simulation = sim.Sim(scene.create_model(cfg), SimConfig())
    simulation.step(10)
    camera_set = SimCameraSet(simulation, {name: cfg.camera_cfgs[name] for name in camera_cfgs}, physical_units=True)
    frames = []
    for _ in range(2):
        frameset = camera_set.get_latest_frames()
        assert frameset is not None
        frames.append({name: frame.camera.color.data.astype(int) for name, frame in frameset.frames.items()})
    # rendering the same state twice gives the same images
    for name in frames[0]:
        assert np.abs(frames[0][name] - frames[1][name]).max() <= 1, name
    return frames[1]


def test_camera_images_do_not_depend_on_other_cameras():
    # The cameras share one mjvScene and mjv_updateScene places infinite planes (the floor) relative to
    # the scene's previous camera, which made images depend on which camera was rendered before.
    scene = EmptyWorldDroid()
    camera_cfgs = scene.prefixed_cfg(scene.config()).camera_cfgs
    assert camera_cfgs is not None
    names = list(camera_cfgs)
    assert len(names) > 1
    together = _render(names)
    for name in names:
        alone = _render([name])[name]
        assert np.abs(together[name] - alone).max() <= 1, name


def _droid_sim():
    scene = EmptyWorldDroid()
    cfg = scene.prefixed_cfg(scene.config())
    assert cfg.camera_cfgs is not None
    return sim.Sim(scene.create_model(cfg), SimConfig()), cfg.camera_cfgs


def test_state_capture_follows_fixed_schedule():
    simulation, _ = _droid_sim()
    dt = simulation.model.opt.timestep
    frame_rate = 30
    capture = simulation.register_state_capture(frame_rate, 1000, int(STATE_SPEC))
    # the current state is captured right away
    assert simulation.captured_state_count(capture) == 1
    first = simulation.get_captured_state(capture)
    assert first is not None
    assert first.timestamp == 0
    assert len(first.state) == mujoco.mj_stateSize(simulation.model, STATE_SPEC)

    simulation.step(round(2.0 / dt))
    timestamps: list[float] = []
    for _ in range(simulation.captured_state_count(capture)):
        captured = simulation.get_captured_state(capture, timestamps[-1] - dt / 2 if timestamps else math.inf)
        assert captured is not None
        timestamps.append(captured.timestamp)
    timestamps.reverse()
    # one capture per frame at the physics step closest to k / frame_rate, i.e. no drift over time
    assert len(timestamps) == 2 * frame_rate + 1
    expected = np.arange(len(timestamps)) / frame_rate
    assert np.abs(np.array(timestamps) - expected).max() <= dt / 2 + 1e-9


def test_state_capture_restarts_on_reset():
    simulation, _ = _droid_sim()
    capture = simulation.register_state_capture(30, 100, int(STATE_SPEC))
    simulation.step(100)
    assert simulation.captured_state_count(capture) > 1
    simulation.reset()
    assert simulation.captured_state_count(capture) == 1
    captured = simulation.get_captured_state(capture)
    assert captured is not None
    assert captured.timestamp == 0
