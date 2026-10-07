import gymnasium as gym
import numpy as np
import pytest
from rcs.camera.sim import FilamentSimCameraSet, SimCameraSet
from rcs.envs.base import CameraSetWrapper
from rcs.envs.configs import EmptyWorldFR3
from rcs.sim import RendererBackend, SimConfig, filament

from rcs import sim

pytestmark = pytest.mark.skipif(not filament.is_available(), reason="Filament renderer requires mujoco >= 3.15")


@pytest.fixture()
def fr3_sim():
    scene = EmptyWorldFR3()
    cfg = scene.prefixed_cfg(scene.config())
    simulation = sim.Sim(scene.create_model(cfg), SimConfig(renderer=RendererBackend.FILAMENT))
    return simulation, cfg


def test_sim_camera_set_dispatches_to_filament(fr3_sim):
    simulation, cfg = fr3_sim
    camera_set = SimCameraSet(simulation, cfg.camera_cfgs, physical_units=True)
    assert isinstance(camera_set, FilamentSimCameraSet)
    assert camera_set.camera_names == list(cfg.camera_cfgs.keys())
    camera_set.close()


def test_filament_camera_set_renders_rgb_and_metric_depth(fr3_sim):
    simulation, cfg = fr3_sim
    camera_set = FilamentSimCameraSet(simulation, cfg.camera_cfgs)
    try:
        simulation.step(1)
        frameset = camera_set.get_latest_frames()
        assert frameset is not None
        assert frameset.frames.keys() == cfg.camera_cfgs.keys()
        for name, frame in frameset.frames.items():
            cam_cfg = cfg.camera_cfgs[name]
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

        # the same simulation time returns the buffered frames, a new step renders again
        assert camera_set.get_latest_frames() is frameset
        simulation.step(1)
        assert camera_set.get_latest_frames() is not frameset
        assert camera_set.buffer_size() == 2
    finally:
        camera_set.close()


def test_filament_camera_set_skips_depth_when_not_requested(fr3_sim):
    simulation, cfg = fr3_sim
    camera_set = FilamentSimCameraSet(simulation, cfg.camera_cfgs, render_depth=False)
    try:
        simulation.step(1)
        frameset = camera_set.get_latest_frames()
        assert frameset is not None
        assert all(frame.camera.depth is None for frame in frameset.frames.values())
        assert all(frame.camera.color.data.dtype == np.uint8 for frame in frameset.frames.values())
    finally:
        camera_set.close()


class _EmptyObsEnv(gym.Env):
    observation_space = gym.spaces.Dict({})
    action_space = gym.spaces.Dict({})


def test_camera_set_wrapper_configures_depth_rendering(fr3_sim):
    simulation, cfg = fr3_sim
    camera_set = FilamentSimCameraSet(simulation, cfg.camera_cfgs)
    try:
        CameraSetWrapper(_EmptyObsEnv(), camera_set, include_depth=False)
        assert camera_set.render_depth is False
        CameraSetWrapper(_EmptyObsEnv(), camera_set, include_depth=True)
        assert camera_set.render_depth is True
    finally:
        camera_set.close()
