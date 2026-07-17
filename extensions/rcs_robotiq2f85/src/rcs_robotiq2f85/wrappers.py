"""Observation wrappers for the Robotiq 2F-85 gripper."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
from rcs.envs.space_utils import ActObsInfoWrapper

from rcs_robotiq2f85.kinematics import FingerPosePair, robotiq_2f85_finger_poses


class Robotiq2F85FingerPoseWrapper(ActObsInfoWrapper):
    """Add cached left/right finger poses to every arm with a gripper state.

    The added ``gripper_finger_pose`` value has shape ``(2, 4, 4)``. Index 0
    contains the left pose and index 1 contains the right pose.
    """

    GRIPPER_KEY = "gripper"
    FINGER_POSE_KEY = "gripper_finger_pose"

    def __init__(
        self,
        env: gym.Env[dict[str, Any], dict[str, Any]],
        *,
        offsets: FingerPosePair | dict[str, FingerPosePair] | None = None,
        model_path: str | Path | None = None,
    ) -> None:
        super().__init__(env)
        self.offsets = offsets
        self.model_path = model_path
        self.observation_space = copy.deepcopy(env.observation_space)

        if not isinstance(self.observation_space, gym.spaces.Dict):
            msg = "Robotiq2F85FingerPoseWrapper requires a Dict observation space"
            raise TypeError(msg)

        self.arm_names: list[str] = []
        for arm_name, arm_space in self.observation_space.spaces.items():
            if not isinstance(arm_space, gym.spaces.Dict) or self.GRIPPER_KEY not in arm_space.spaces:
                continue
            arm_space.spaces[self.FINGER_POSE_KEY] = gym.spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=(2, 4, 4),
                dtype=np.float64,
            )
            self.arm_names.append(arm_name)

        if not self.arm_names:
            msg = "No arm with a 'gripper' observation was found"
            raise ValueError(msg)
        if isinstance(offsets, dict):
            unknown_arms = offsets.keys() - set(self.arm_names)
            if unknown_arms:
                msg = f"Offsets provided for unknown arms: {sorted(unknown_arms)}"
                raise ValueError(msg)

    def _offsets_for_arm(self, arm_name: str) -> FingerPosePair | None:
        if isinstance(self.offsets, dict):
            return self.offsets.get(arm_name)
        return self.offsets

    def observation(
        self, observation: dict[str, Any], info: dict[str, Any]
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        observation = copy.deepcopy(observation)
        for arm_name in self.arm_names:
            arm_observation = observation[arm_name]
            gripper_state = np.asarray(arm_observation[self.GRIPPER_KEY], dtype=np.float64)
            if gripper_state.size != 1:
                msg = f"Expected one gripper value for arm {arm_name!r}, got shape {gripper_state.shape}"
                raise ValueError(msg)

            poses = robotiq_2f85_finger_poses(
                float(gripper_state.item()),
                offsets=self._offsets_for_arm(arm_name),
                model_path=self.model_path,
            )
            arm_observation[self.FINGER_POSE_KEY] = np.stack(
                (poses.left.pose_matrix(), poses.right.pose_matrix())
            )

        return observation, info
