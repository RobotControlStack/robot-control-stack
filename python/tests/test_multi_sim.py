"""Tests for multi-robot simulation with greenlet-based step coordination."""

import copy

import numpy as np
import pytest
from rcs.envs.base import ControlMode, JointsDictType, RelativeTo
from rcs.envs.configs import EmptyWorldFR3Duo

ROBOT2ID = {"left": "0", "right": "1"}


@pytest.fixture()
def multi_env():
    scene = EmptyWorldFR3Duo()
    cfg = copy.deepcopy(scene.config())
    cfg.control_mode = ControlMode.JOINTS
    cfg.task_cfg = None
    cfg.gripper_cfgs = None
    cfg.gripper_offsets = None
    cfg.camera_cfgs = None
    cfg.camera_adds = None
    cfg.headless = True
    cfg.sim_cfg.realtime = False
    cfg.sim_cfg.async_control = False
    cfg.max_relative_movement = None
    cfg.relative_to = RelativeTo.NONE
    env = scene.create_env(cfg)
    yield env
    env.close()


class TestMultiSimRobotWrapper:
    def test_reset_returns_obs_for_all_robots(self, multi_env):
        obs, info = multi_env.reset()
        assert set(obs.keys()) == {"left", "right"}
        for key in ROBOT2ID:
            assert "joints" in obs[key], f"Missing joints in obs[{key!r}]"
            assert "tquat" in obs[key], f"Missing tquat in obs[{key!r}]"
            assert len(obs[key]["joints"]) == 7

    def test_double_reset(self, multi_env):
        multi_env.reset()
        obs, info = multi_env.reset()
        assert set(obs.keys()) == {"left", "right"}

    def test_step_returns_obs_for_all_robots(self, multi_env):
        obs0, _ = multi_env.reset()
        actions = {key: JointsDictType(joints=obs0[key]["joints"]) for key in ROBOT2ID}
        obs, reward, terminated, truncated, info = multi_env.step(actions)
        assert set(obs.keys()) == {"left", "right"}
        assert set(info.keys()) == {"left", "right"}
        for key in ROBOT2ID:
            assert "joints" in obs[key]
            assert "ik_success" in info[key]
            assert "collision" in info[key]

    def test_multiple_reset_step_cycles(self, multi_env):
        """Reset → step → reset → step should not crash or produce stale obs."""
        for cycle in range(2):
            obs, info = multi_env.reset()
            assert set(obs.keys()) == set(ROBOT2ID)
            actions = {key: JointsDictType(joints=obs[key]["joints"]) for key in ROBOT2ID}
            obs2, _, _, _, info2 = multi_env.step(actions)
            assert set(obs2.keys()) == set(ROBOT2ID)
            for key in ROBOT2ID:
                assert info2[key]["ik_success"], f"IK failed on cycle {cycle} for {key!r}"

    def test_sim_stepped_once_per_multi_step(self, multi_env):
        """
        Verify the sim is stepped ONCE per multi-step call, not once per robot.
        With the same absolute target across two sequential steps, the second
        step should produce the same or smaller change than the first step
        (i.e. the robot is not being double-stepped).
        """
        obs0, _ = multi_env.reset()
        home = {key: obs0[key]["joints"].copy() for key in ROBOT2ID}

        # First step from home — physics moves the robot some amount
        actions = {key: JointsDictType(joints=home[key]) for key in ROBOT2ID}
        obs1, _, _, _, _ = multi_env.step(actions)
        delta1 = max(float(np.linalg.norm(obs1[k]["joints"] - home[k])) for k in ROBOT2ID)

        # Second step with same target — robot is now closer to or past target.
        # If the sim were stepped twice per call, delta2 would be >> delta1.
        obs2, _, _, _, info = multi_env.step(actions)
        delta2 = max(float(np.linalg.norm(obs2[k]["joints"] - home[k])) for k in ROBOT2ID)

        for key in ROBOT2ID:
            assert info[key]["ik_success"], f"IK failed for {key!r}"
        # delta2 should not be dramatically larger than delta1 (at most 3x)
        assert (
            delta2 < delta1 * 3
        ), f"Second step drifted {delta2:.3f} vs first {delta1:.3f} — sim may be double-stepped"

    def test_robots_are_independent(self, multi_env):
        """
        Moving one robot (joint 0) while holding the other should produce
        clearly different outcomes on joint 0 for the two robots.
        """
        obs0, _ = multi_env.reset()
        # Apply a large delta to 'right' joint 0; 'left' holds home
        big_delta_j0 = 0.3
        actions = {
            "left": JointsDictType(joints=obs0["left"]["joints"].copy()),
            "right": JointsDictType(joints=obs0["right"]["joints"] + np.array([big_delta_j0, 0, 0, 0, 0, 0, 0])),
        }
        obs, _, _, _, info = multi_env.step(actions)
        right_j0_change = obs["right"]["joints"][0] - obs0["right"]["joints"][0]
        left_j0_change = obs["left"]["joints"][0] - obs0["left"]["joints"][0]
        assert right_j0_change > 0.01, f"Right robot joint 0 did not move toward target: change={right_j0_change:.4f}"
        assert abs(left_j0_change) < abs(
            right_j0_change
        ), f"Left robot joint 0 moved as much as right: left={left_j0_change:.4f}, right={right_j0_change:.4f}"
