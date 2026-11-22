"""wrapper_cfg.home_noise: DROID-like randomized reset, the home TCP pose is perturbed through IK."""

import numpy as np
from rcs.envs.configs import EmptyWorldFR3

NOISE = (0.1, 0.2, 0.1, 0.3, 0.3, 0.3)


def _env(noise):
    creator = EmptyWorldFR3()
    cfg = creator.config()
    cfg.headless = True
    cfg.wrapper_cfg.home_noise = noise
    for cam in (cfg.camera_cfgs or {}).values():
        cam.resolution_width, cam.resolution_height = 64, 48
    return creator.create_env(cfg)


def _robot(env):
    return env.get_wrapper_attr("envs")["right"].get_wrapper_attr("robot")


def test_without_noise_reset_is_the_home_pose():
    env = _env(None)
    robot = _robot(env)
    q_home = np.array(robot.get_config().q_home, copy=True)
    for _ in range(2):
        env.reset()
        assert np.allclose(robot.get_joint_position(), q_home, atol=1e-3)
    env.close()


def test_noisy_home_stays_within_the_bounds_and_keeps_q_home():
    env = _env(NOISE)
    robot = _robot(env)
    cfg = robot.get_config()
    q_home = np.array(cfg.q_home, copy=True)
    home = robot.get_ik().forward(q_home, cfg.end_effector.tcp_offset)
    offsets = []
    for _ in range(10):
        env.reset()
        pose = robot.get_cartesian_position()
        d = pose.translation() - home.translation()
        assert np.all(np.abs(d) <= np.array(NOISE[:3]) + 0.01)
        offsets.append(d)
        # the configured home pose is restored after every reset (no random walk)
        assert np.allclose(robot.get_config().q_home, q_home)
    assert np.max(np.linalg.norm(offsets, axis=1)) > 0.02
    env.close()
