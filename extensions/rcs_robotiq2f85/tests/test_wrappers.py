from typing import Any

import gymnasium as gym
import numpy as np
import pytest

from rcs import common
from rcs_robotiq2f85 import wrappers
from rcs_robotiq2f85.kinematics import FingerPosePair


class _MultiArmEnv(gym.Env):
    def __init__(self) -> None:
        def arm_space() -> gym.spaces.Dict:
            return gym.spaces.Dict({"gripper": gym.spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32)})

        self.observation_space = gym.spaces.Dict({"left": arm_space(), "right": arm_space()})
        self.action_space = gym.spaces.Dict({})


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
    ) -> FingerPosePair:
        del model_path
        calls.append((normalized_state, body_name, site_name, offsets))
        return FingerPosePair(
            left=common.Pose(translation=np.array([normalized_state, 0.0, 0.0])),
            right=common.Pose(translation=np.array([-normalized_state, 0.0, 0.0])),
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
    assert set(left_finger_poses) == {"left_finger", "right_finger"}
    assert np.allclose(left_finger_poses["left_finger"][:3, 3], [0.25, 0.0, 0.0])
    assert np.allclose(right_finger_poses["right_finger"][:3, 3], [-0.75, 0.0, 0.0])

    left_space = wrapper.observation_space["left"]
    assert isinstance(left_space, gym.spaces.Dict)
    finger_pose_space = left_space["gripper_finger_pose"]
    assert isinstance(finger_pose_space, gym.spaces.Dict)
    assert finger_pose_space["left_finger"].shape == (4, 4)
    assert finger_pose_space["right_finger"].shape == (4, 4)


@pytest.mark.parametrize(
    ("body_name", "site_name"),
    [(None, None), (("left", "right"), ("left_site", "right_site"))],
)
def test_robotiq2f85_finger_pose_wrapper_requires_exactly_one_target_pair(body_name, site_name) -> None:
    with pytest.raises(ValueError, match="exactly one of body_name or site_name must be provided"):
        wrappers.Robotiq2F85FingerPoseWrapper(_MultiArmEnv(), body_name=body_name, site_name=site_name)
