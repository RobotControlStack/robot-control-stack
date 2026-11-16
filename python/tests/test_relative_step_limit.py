"""max_step_movement: LimitedAbsoluteAction below RelativeActionSpace limits the commanded target per step relative to
the measured pose (DROID-like), also with RelativeTo.CONFIGURED_ORIGIN (Quest teleop)."""

import numpy as np
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.configs import EmptyWorldFR3


def _env(step=(0.075, 0.15)):
    creator = EmptyWorldFR3()
    cfg = creator.config()
    cfg.headless = True
    cfg.control_mode = ControlMode.CARTESIAN_TQuat
    cfg.relative_to = RelativeTo.CONFIGURED_ORIGIN
    cfg.max_relative_movement = (0.5, 1.57)  # only bounds the offset from the origin
    cfg.max_step_movement = step
    for cam in (cfg.camera_cfgs or {}).values():
        cam.resolution_width, cam.resolution_height = 64, 48
    return creator.create_env(cfg)


def _act(env, offset):
    obs, _, _, _, info = env.step({"right": {"tquat": np.array([*offset, 0, 0, 0, 1.0]), "gripper": np.array([1.0])}})
    return np.asarray(info["right"]["limited_absolute_action"][:3]), np.asarray(obs["right"]["tquat"][:3])


def test_target_moves_at_most_max_step_from_the_measured_pose():
    env = _env()
    obs, _ = env.reset()
    for _ in range(8):
        measured = np.asarray(
            env.get_wrapper_attr("envs")["right"].get_wrapper_attr("robot").get_cartesian_position().translation()
        )
        target, _ = _act(env, [0.30, 0, 0])  # the hand is 30 cm away from the origin
        assert np.linalg.norm(target - measured) <= 0.075 + 1e-6
    env.close()


def test_hand_inside_the_limit_is_followed_exactly():
    env = _env()
    env.reset()
    origin = np.asarray(env.get_wrapper_attr("envs")["right"].get_wrapper_attr("_origin").translation())
    target, _ = _act(env, [0.02, 0, 0])
    assert np.allclose(target, origin + np.array([0.02, 0, 0]), atol=1e-6)
    env.close()


def test_without_max_step_nothing_changes():
    env = _env(step=None)
    env.reset()
    _, _, _, _, info = env.step({"right": {"tquat": np.array([0.3, 0, 0, 0, 0, 0, 1.0]), "gripper": np.array([1.0])}})
    assert "limited_absolute_action" not in info["right"]
    env.close()
