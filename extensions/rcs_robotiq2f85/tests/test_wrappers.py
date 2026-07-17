from typing import Any

import gymnasium as gym
import numpy as np

from rcs import common
from rcs_robotiq2f85 import wrappers
from rcs_robotiq2f85.kinematics import FingerPosePair


class _MultiArmEnv(gym.Env):
    def __init__(self) -> None:
        def arm_space() -> gym.spaces.Dict:
            return gym.spaces.Dict(
                {"gripper": gym.spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32)}
            )

        self.observation_space = gym.spaces.Dict({"left": arm_space(), "right": arm_space()})
        self.action_space = gym.spaces.Dict({})


def test_robotiq2f85_finger_pose_wrapper(monkeypatch) -> None:
    calls: list[tuple[float, FingerPosePair | None]] = []

    def fake_finger_poses(
        normalized_state: float, *, offsets: FingerPosePair | None, model_path: str | None
    ) -> FingerPosePair:
        del model_path
        calls.append((normalized_state, offsets))
        return FingerPosePair(
            left=common.Pose(translation=np.array([normalized_state, 0.0, 0.0])),
            right=common.Pose(translation=np.array([-normalized_state, 0.0, 0.0])),
        )

    monkeypatch.setattr(wrappers, "robotiq_2f85_finger_poses", fake_finger_poses)
    left_offsets = FingerPosePair(left=common.Pose(), right=common.Pose())
    wrapper = wrappers.Robotiq2F85FingerPoseWrapper(_MultiArmEnv(), offsets={"left": left_offsets})
    observation: dict[str, Any] = {
        "left": {"gripper": np.array([0.25], dtype=np.float32)},
        "right": {"gripper": np.array([0.75], dtype=np.float32)},
    }

    wrapped_observation, wrapped_info = wrapper.observation(observation, {})

    assert "gripper_finger_pose" not in observation["left"]
    assert wrapped_info == {}
    assert calls == [(0.25, left_offsets), (0.75, None)]
    assert wrapped_observation["left"]["gripper_finger_pose"].shape == (2, 4, 4)
    assert np.allclose(wrapped_observation["left"]["gripper_finger_pose"][0, :3, 3], [0.25, 0.0, 0.0])
    assert np.allclose(wrapped_observation["right"]["gripper_finger_pose"][1, :3, 3], [-0.75, 0.0, 0.0])

    left_space = wrapper.observation_space["left"]
    assert isinstance(left_space, gym.spaces.Dict)
    assert left_space["gripper_finger_pose"].shape == (2, 4, 4)
