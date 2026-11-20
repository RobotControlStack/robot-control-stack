import math

import mujoco
import numpy as np
import pytest
from rcs.camera.sim import STATE_SPEC, SimCameraSet
from rcs.envs.configs import EmptyWorldDroid
from rcs.sim import SimConfig

from rcs import sim


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


def test_state_capture_restarts_when_time_goes_backwards():
    simulation, _ = _droid_sim()
    dt = simulation.model.opt.timestep
    capture = simulation.register_state_capture(30, 100, int(STATE_SPEC))
    simulation.step(100)
    assert simulation.captured_state_count(capture) > 1
    # set the time back without reset, e.g. when a recorded state is restored
    simulation.data.time = 0
    simulation.step(1)
    assert simulation.captured_state_count(capture) == 1
    captured = simulation.get_captured_state(capture)
    assert captured is not None
    assert captured.timestamp == pytest.approx(dt)
    # and the fixed schedule continues from there
    simulation.step(round(1 / 30 / dt))
    assert simulation.captured_state_count(capture) == 2


def test_frames_show_last_exposure():
    simulation, camera_cfgs = _droid_sim()
    dt = simulation.model.opt.timestep
    name = next(iter(camera_cfgs))
    cfg = camera_cfgs[name]
    period = 1 / cfg.frame_rate
    exposed = SimCameraSet(simulation, {name: cfg}, physical_units=True, render_on_demand=False)
    live = SimCameraSet(simulation, {name: cfg}, physical_units=True)
    simulation.data.ctrl[:] = 0.5

    # step to just after an exposure: the exposure is the live state at that time
    simulation.step(round(3 * period / dt))
    live_at_exposure = live.get_latest_frames()
    exposed_frames = exposed.get_latest_frames()
    assert live_at_exposure is not None
    assert exposed_frames is not None
    assert exposed_frames.avg_timestamp == pytest.approx(live_at_exposure.avg_timestamp)
    exposure = exposed_frames.frames[name].camera
    live_frame = live_at_exposure.frames[name].camera
    assert np.abs(exposure.color.data.astype(int) - live_frame.color.data.astype(int)).max() <= 1
    assert exposure.depth is not None
    assert live_frame.depth is not None
    assert np.array_equal(exposure.depth.data, live_frame.depth.data)
    assert exposure.color.extrinsics is not None
    assert np.allclose(exposure.color.extrinsics, live_frame.color.extrinsics)  # type: ignore[arg-type]

    # until the next exposure, the frames do not change while the live state does
    simulation.step(round(0.5 * period / dt))
    later = exposed.get_latest_frames()
    assert later is not None
    assert later.avg_timestamp == exposed_frames.avg_timestamp
    assert later.frames[name] is exposed_frames.frames[name]
    later_live = live.get_latest_frames()
    assert later_live is not None
    assert later_live.avg_timestamp is not None
    assert later.avg_timestamp is not None
    assert later_live.avg_timestamp > later.avg_timestamp

    # after the next exposure, new frames are rendered
    simulation.step(round(0.6 * period / dt))
    newer = exposed.get_latest_frames()
    assert newer is not None
    assert newer.avg_timestamp == pytest.approx(4 * period, abs=dt)
    exposed.close()
    live.close()
