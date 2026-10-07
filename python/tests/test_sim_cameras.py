import gymnasium as gym
import mujoco
import numpy as np
import pytest
from rcs.camera.sim import SimCameraSet
from rcs.envs.base import CameraSetWrapper
from rcs.envs.configs import EmptyWorldFR3
from rcs.sim import RendererBackend, SimConfig, filament

from rcs import sim

BACKENDS = [
    pytest.param(RendererBackend.CLASSIC, id="classic"),
    pytest.param(
        RendererBackend.FILAMENT,
        id="filament",
        marks=pytest.mark.skipif(not filament.is_available(), reason="Filament renderer requires mujoco >= 3.15"),
    ),
]


def make_sim(renderer: RendererBackend = RendererBackend.CLASSIC):
    scene = EmptyWorldFR3()
    cfg = scene.prefixed_cfg(scene.config())
    assert cfg.camera_cfgs is not None
    simulation = sim.Sim(scene.create_model(cfg), SimConfig(renderer=renderer))
    return simulation, cfg.camera_cfgs


@pytest.fixture(params=BACKENDS)
def fr3_sim(request):
    return make_sim(request.param)


class _EmptyObsEnv(gym.Env):
    observation_space = gym.spaces.Dict({})
    action_space = gym.spaces.Dict({})


def steps_per_period(simulation, frame_rate: int) -> int:
    return round(1 / frame_rate / simulation.model.opt.timestep)


def test_camera_set_renders_rgb_and_metric_depth(fr3_sim):
    simulation, camera_cfgs = fr3_sim
    camera_set = SimCameraSet(simulation, camera_cfgs, physical_units=True)
    try:
        assert camera_set.renderer == simulation.get_config().renderer
        assert camera_set.camera_names == list(camera_cfgs.keys())
        simulation.step(1)
        frameset = camera_set.get_latest_frames()
        assert frameset is not None
        assert frameset.frames.keys() == camera_cfgs.keys()
        for name, frame in frameset.frames.items():
            cam_cfg = camera_cfgs[name]
            assert frame.camera.depth is not None
            color = frame.camera.color.data
            depth = frame.camera.depth.data
            assert color.shape == (cam_cfg.resolution_height, cam_cfg.resolution_width, 3)
            assert color.dtype == np.uint8
            # the scene is lit and textured, so the image must not be uniform
            assert color.std() > 1
            assert depth.shape == (cam_cfg.resolution_height, cam_cfg.resolution_width, 1)
            assert depth.dtype == np.uint16
            # metric depth in millimeters: the cameras look at the robot/floor within a few meters
            assert 10 < np.median(depth) < 10_000
            assert frame.camera.color.intrinsics is not None
            assert frame.camera.color.extrinsics is not None
            assert frame.camera.color.intrinsics.shape == (3, 4)
            assert frame.camera.color.extrinsics.shape == (4, 4)

        # unchanged camera state returns the buffered frames, crossing a camera period renders again
        assert camera_set.get_latest_frames() is frameset
        simulation.step(1)
        assert camera_set.get_latest_frames() is frameset
        simulation.step(steps_per_period(simulation, 30))
        assert camera_set.get_latest_frames() is not frameset
        assert camera_set.buffer_size() == 2
    finally:
        camera_set.close()


def test_frames_are_rendered_from_frame_rate_snapshots(fr3_sim):
    simulation, camera_cfgs = fr3_sim
    rates = {cfg.frame_rate for cfg in camera_cfgs.values()}
    assert rates == {30}, "test assumes all cameras run at 30 Hz"
    period = 1 / 30
    dt = simulation.model.opt.timestep
    camera_set = SimCameraSet(simulation, camera_cfgs)
    try:
        # the first snapshot is taken in the first step
        simulation.step(1)
        frameset = camera_set.get_latest_frames()
        assert frameset is not None and frameset.avg_timestamp is not None
        assert frameset.avg_timestamp == pytest.approx(simulation.data.time)
        # stepping within one camera period keeps the old frame (the camera has not ticked yet)
        simulation.step(3)
        later = camera_set.get_latest_frames()
        assert later is frameset
        assert simulation.data.time > frameset.avg_timestamp
        # after crossing the period a new frame is rendered, lagging less than one period behind
        simulation.step(steps_per_period(simulation, 30))
        newest = camera_set.get_latest_frames()
        assert newest is not None and newest.avg_timestamp is not None
        assert newest is not frameset
        assert newest.avg_timestamp > frameset.avg_timestamp
        assert newest.avg_timestamp <= simulation.data.time
        assert simulation.data.time - newest.avg_timestamp < period + dt
        for frame in newest.frames.values():
            assert frame.camera.color.timestamp == newest.avg_timestamp
    finally:
        camera_set.close()


def test_snapshot_render_matches_current_render():
    """Rendering from a restored snapshot must give the same image as rendering the live state."""
    simulation, camera_cfgs = make_sim()
    snapshot_set = SimCameraSet(simulation, camera_cfgs, physical_units=True)
    current_set = SimCameraSet(simulation, camera_cfgs, physical_units=True, render_current=True)
    try:
        # move the robot a bit so that the state differs from the model's initial state
        simulation.data.ctrl[:] = simulation.model.key_ctrl[0] if simulation.model.nkey > 0 else 0.3
        # the snapshot is taken in the last of these steps, so it equals the live state
        simulation.step(1 + steps_per_period(simulation, 30))
        from_snapshot = snapshot_set.get_latest_frames()
        from_current = current_set.get_latest_frames()
        assert from_snapshot is not None and from_current is not None
        assert from_snapshot.avg_timestamp == pytest.approx(from_current.avg_timestamp)
        for name in camera_cfgs:
            a, b = from_snapshot.frames[name].camera, from_current.frames[name].camera
            assert np.array_equal(a.color.data, b.color.data)
            assert a.depth is not None and b.depth is not None
            assert np.array_equal(a.depth.data, b.depth.data)
            assert a.color.extrinsics is not None and b.color.extrinsics is not None
            assert np.allclose(a.color.extrinsics, b.color.extrinsics)
    finally:
        snapshot_set.close()
        current_set.close()


def test_render_current_and_reset():
    simulation, camera_cfgs = make_sim()
    current_set = SimCameraSet(simulation, camera_cfgs, render_current=True)
    snapshot_set = SimCameraSet(simulation, camera_cfgs)
    try:
        simulation.step(5)
        frameset = current_set.get_latest_frames()
        assert frameset is not None
        assert frameset.avg_timestamp == pytest.approx(simulation.data.time)
        simulation.step(1)
        assert current_set.get_latest_frames() is not frameset

        # after a reset the snapshots are cleared and the current (reset) state is rendered
        snapshot_set.get_latest_frames()
        simulation.reset()
        frameset = snapshot_set.get_latest_frames()
        assert frameset is not None
        assert frameset.avg_timestamp == 0
    finally:
        current_set.close()
        snapshot_set.close()


def test_render_on_demand_is_deprecated_alias():
    simulation, camera_cfgs = make_sim()
    with pytest.warns(DeprecationWarning):
        camera_set = SimCameraSet(simulation, camera_cfgs, render_on_demand=True)
    assert camera_set.render_current is True
    camera_set.close()


def test_camera_set_skips_depth_when_not_requested(fr3_sim):
    simulation, camera_cfgs = fr3_sim
    camera_set = SimCameraSet(simulation, camera_cfgs, render_depth=False)
    try:
        simulation.step(1)
        frameset = camera_set.get_latest_frames()
        assert frameset is not None
        assert all(frame.camera.depth is None for frame in frameset.frames.values())
        assert all(frame.camera.color.data.dtype == np.uint8 for frame in frameset.frames.values())
    finally:
        camera_set.close()


def test_camera_set_wrapper_configures_depth_rendering(fr3_sim):
    simulation, camera_cfgs = fr3_sim
    camera_set = SimCameraSet(simulation, camera_cfgs)
    try:
        CameraSetWrapper(_EmptyObsEnv(), camera_set, include_depth=False)
        assert camera_set.render_depth is False
        CameraSetWrapper(_EmptyObsEnv(), camera_set, include_depth=True)
        assert camera_set.render_depth is True
    finally:
        camera_set.close()


def test_classic_depth_matches_mujoco_renderer():
    """The raw depth buffer converted to meters must match mujoco.Renderer's metric depth."""
    simulation, camera_cfgs = make_sim()
    camera_set = SimCameraSet(simulation, camera_cfgs, physical_units=True, render_current=True)
    try:
        simulation.step(1)
        frameset = camera_set.get_latest_frames()
        assert frameset is not None
        for name, cam_cfg in camera_cfgs.items():
            # mujoco.Renderer is limited by the model's offscreen buffer size, so compare at a small
            # resolution: downsample ours by the integer factor
            w, h = cam_cfg.resolution_width // 4, cam_cfg.resolution_height // 4
            renderer = mujoco.Renderer(simulation.model, height=h, width=w)
            renderer.enable_depth_rendering()
            renderer.update_scene(simulation.data, camera=cam_cfg.identifier)
            reference = renderer.render()
            renderer.close()
            depth_frame = frameset.frames[name].camera.depth
            assert depth_frame is not None
            ours = depth_frame.data[::4, ::4, 0] / SimCameraSet.DEPTH_SCALE
            assert ours.shape == reference.shape
            # different resolutions sample slightly different rays; compare the bulk of the image
            assert np.median(np.abs(ours - reference)) < 0.01
    finally:
        camera_set.close()
