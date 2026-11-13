"""Tests for the simulated bimanual yam setup."""

import numpy as np
import pytest
from rcs.envs.configs import EmptyWorldYamDuo

ROBOTS = ("left", "right")
CAMERAS = ("head", "left_wrist", "right_wrist")


@pytest.fixture(scope="module")
def duo_env():
    scene = EmptyWorldYamDuo()
    cfg = scene.config()
    cfg.headless = True
    cfg.sim_cfg.realtime = False
    for camera_cfg in cfg.camera_cfgs.values():  # type: ignore[union-attr]
        camera_cfg.resolution_width = 128
        camera_cfg.resolution_height = 72
    env = scene.create_env(cfg)
    yield env
    env.close()


def test_reset_obs(duo_env):
    obs, _ = duo_env.reset()
    assert set(obs["frames"]) == set(CAMERAS)
    for camera in CAMERAS:
        assert obs["frames"][camera]["rgb"]["data"].shape == (72, 128, 3)
    for robot, side in zip(ROBOTS, (1, -1), strict=True):
        assert len(obs[robot]["joints"]) == 6
        # the home pose puts both grasp points in front of their arm, mirrored about the shared base frame
        np.testing.assert_allclose(obs[robot]["tquat"][:3], [0.25, side * 0.22, 0.15], atol=5e-3)


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
