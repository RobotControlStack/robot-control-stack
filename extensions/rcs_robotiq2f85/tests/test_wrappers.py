from typing import Any

import gymnasium as gym
import numpy as np
import pytest

from rcs import common
from rcs_robotiq2f85 import wrappers
from rcs_robotiq2f85.kinematics import FingerPoseFrames, FingerPosePair


class _MultiArmEnv(gym.Env):
    def __init__(self) -> None:
        def arm_space() -> gym.spaces.Dict:
            return gym.spaces.Dict({"gripper": gym.spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32)})

        self.observation_space = gym.spaces.Dict({"left": arm_space(), "right": arm_space()})
        self.action_space = gym.spaces.Dict({})
        self.robots = {"left": _FakeRobot(), "right": _FakeRobot()}

    def get_wrapper_attr(self, name: str):
        if name == "robot":
            return self.robots
        raise AttributeError(name)


class _FakeRobot:
    def __init__(self) -> None:
        self.pinocchio = object()
        self.joints = np.zeros(7)
        self.ik_calls = 0

    def get_ik(self) -> object:
        self.ik_calls += 1
        return self.pinocchio

    def get_joint_position(self) -> np.ndarray:
        return self.joints


def test_robotiq2f85_finger_pose_wrapper(monkeypatch) -> None:
    calls: list[
        tuple[
            float,
            tuple[str, str] | None,
            tuple[str, str] | None,
            FingerPosePair | None,
        ]
    ] = []

    def fake_finger_poses(
        normalized_state: float,
        *,
        body_name: tuple[str, str] | None,
        site_name: tuple[str, str] | None,
        offsets: FingerPosePair | None = None,
        model_path: str | None,
        pinocchio: common.Kinematics | None = None,
        robot_joints: np.ndarray | None = None,
    ) -> FingerPoseFrames:
        del model_path, pinocchio, robot_joints
        calls.append((normalized_state, body_name, site_name, offsets))
        left = common.Pose(translation=np.array([normalized_state, 0.0, 0.0]))
        right = common.Pose(translation=np.array([-normalized_state, 0.0, 0.0]))
        return FingerPoseFrames(
            left_finger_wrist_frame=left,
            right_finger_wrist_frame=right,
            left_finger_robot_frame=left,
            right_finger_robot_frame=right,
        )

    monkeypatch.setattr(wrappers, "robotiq_2f85_finger_poses", fake_finger_poses)
    left_offsets = FingerPosePair(left=common.Pose(), right=common.Pose())
    wrapper = wrappers.Robotiq2F85FingerPoseWrapper(
        _MultiArmEnv(),
        site_name=("left_pad_site", "right_pad_site"),
        offsets={"left": left_offsets},
    )
    assert calls == [(0.0, None, ("left_pad_site", "right_pad_site"), None)]
    observation: dict[str, Any] = {
        "left": {"gripper": np.array([0.25], dtype=np.float32)},
        "right": {"gripper": np.array([0.75], dtype=np.float32)},
    }

    wrapped_observation, wrapped_info = wrapper.observation(observation, {})

    assert "gripper_finger_pose" not in observation["left"]
    assert wrapped_info == {}
    assert calls[1:] == [
        (0.25, None, ("left_pad_site", "right_pad_site"), left_offsets),
        (0.75, None, ("left_pad_site", "right_pad_site"), None),
    ]
    left_finger_poses = wrapped_observation["left"]["gripper_finger_pose"]
    right_finger_poses = wrapped_observation["right"]["gripper_finger_pose"]
    assert set(left_finger_poses) == {
        "left_finger_wrist_frame",
        "right_finger_wrist_frame",
        "left_finger_robot_frame",
        "right_finger_robot_frame",
    }
    assert np.allclose(left_finger_poses["left_finger_wrist_frame"][:3, 3], [0.25, 0.0, 0.0])
    assert np.allclose(right_finger_poses["right_finger_robot_frame"][:3, 3], [-0.75, 0.0, 0.0])

    left_space = wrapper.observation_space["left"]
    assert isinstance(left_space, gym.spaces.Dict)
    finger_pose_space = left_space["gripper_finger_pose"]
    assert isinstance(finger_pose_space, gym.spaces.Dict)
    assert finger_pose_space["left_finger_wrist_frame"].shape == (4, 4)
    assert finger_pose_space["right_finger_robot_frame"].shape == (4, 4)


def test_robotiq2f85_finger_pose_wrapper_uses_live_robot_fk(monkeypatch) -> None:
    class FakeRobot:
        def __init__(self, pinocchio: object, joints: list[float]) -> None:
            self.pinocchio = pinocchio
            self.joints = np.asarray(joints, dtype=np.float64)
            self.ik_calls = 0

        def get_ik(self) -> object:
            self.ik_calls += 1
            return self.pinocchio

        def get_joint_position(self) -> np.ndarray:
            return self.joints

    left_pinocchio = object()
    right_pinocchio = object()
    robots = {
        "left": FakeRobot(left_pinocchio, [1.0, 2.0]),
        "right": FakeRobot(right_pinocchio, [3.0, 4.0]),
    }

    class RobotEnv(_MultiArmEnv):
        def get_wrapper_attr(self, name: str):
            if name == "robot":
                return robots
            return super().get_wrapper_attr(name)

    contexts: list[tuple[object | None, np.ndarray | None]] = []

    def fake_finger_poses(normalized_state: float, **kwargs) -> FingerPoseFrames:
        contexts.append((kwargs.get("pinocchio"), kwargs.get("robot_joints")))
        left = common.Pose(translation=np.array([normalized_state, 0.0, 0.0]))
        right = common.Pose(translation=np.array([-normalized_state, 0.0, 0.0]))
        return FingerPoseFrames(
            left_finger_wrist_frame=left,
            right_finger_wrist_frame=right,
            left_finger_robot_frame=left,
            right_finger_robot_frame=right,
        )

    monkeypatch.setattr(wrappers, "robotiq_2f85_finger_poses", fake_finger_poses)
    wrapper = wrappers.Robotiq2F85FingerPoseWrapper(
        RobotEnv(),
        site_name=("left_pad_site", "right_pad_site"),
    )
    wrapper.observation(
        {
            "left": {"gripper": np.array([0.25], dtype=np.float32)},
            "right": {"gripper": np.array([0.75], dtype=np.float32)},
        },
        {},
    )

    assert contexts[0] == (None, None)  # The eager cache warm-up remains frame-independent.
    assert contexts[1][0] is left_pinocchio
    assert np.array_equal(contexts[1][1], [1.0, 2.0])
    assert contexts[2][0] is right_pinocchio
    assert np.array_equal(contexts[2][1], [3.0, 4.0])
    wrapper.observation(
        {
            "left": {"gripper": np.array([0.25], dtype=np.float32)},
            "right": {"gripper": np.array([0.75], dtype=np.float32)},
        },
        {},
    )
    assert robots["left"].ik_calls == 1
    assert robots["right"].ik_calls == 1


@pytest.mark.parametrize(
    ("body_name", "site_name"),
    [(None, None), (("left", "right"), ("left_site", "right_site"))],
)
def test_robotiq2f85_finger_pose_wrapper_requires_exactly_one_target_pair(body_name, site_name) -> None:
    with pytest.raises(ValueError, match="exactly one of body_name or site_name must be provided"):
        wrappers.Robotiq2F85FingerPoseWrapper(_MultiArmEnv(), body_name=body_name, site_name=site_name)
