"""Tests for the FR3 duo on the mount of the Franka FR3 Duo pedestal."""

import numpy as np
import pytest
from rcs.envs.configs import EmptyWorldFR3DuoFrankaPedestal

ROBOTS = ("left", "right")


@pytest.fixture(scope="module")
def duo_env():
    scene = EmptyWorldFR3DuoFrankaPedestal()
    cfg = scene.config()
    cfg.headless = True
    cfg.sim_cfg.realtime = False
    for camera_cfg in cfg.camera_cfgs.values():  # type: ignore[union-attr]
        camera_cfg.resolution_width = 128
        camera_cfg.resolution_height = 72
    env = scene.create_env(cfg)
    yield env
    env.close()


def test_mount_geometry(duo_env):
    duo_env.reset()
    sim = duo_env.get_wrapper_attr("sim")
    model, data = sim.model, sim.data
    # the FR3 Duo origin is 312 mm above the bottom of the mounting plate
    for robot, side in zip(ROBOTS, (1, -1), strict=True):
        np.testing.assert_allclose(
            data.xpos[model.body(f"robot{robot}_base").id], [0.0, side * 0.05018, 0.312], atol=1e-4
        )
    # the left lens of the head camera is centered between the arms
    assert abs(data.cam_xpos[model.camera("head").id][1]) < 2e-3


def test_no_contacts_at_home(duo_env):
    duo_env.reset()
    sim = duo_env.get_wrapper_attr("sim")
    assert sim.data.ncon == 0


def test_step_both_arms(duo_env):
    obs, _ = duo_env.reset()
    act = {robot: {"tquat": [0.02, 0, 0, 0, 0, 0, 1], "gripper": [1.0]} for robot in ROBOTS}
    obs2, _, _, _, info = duo_env.step(act)
    for robot in ROBOTS:
        assert info[robot]["ik_success"]
        assert not info[robot]["collision"]
        assert obs2[robot]["tquat"][0] > obs[robot]["tquat"][0] + 0.01
