import numpy as np
from rcs.camera.sim import SimCameraSet
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
