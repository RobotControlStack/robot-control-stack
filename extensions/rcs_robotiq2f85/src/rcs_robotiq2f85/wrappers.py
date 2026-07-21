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

    The added ``gripper_finger_pose`` dictionary contains ``left_finger`` and
    ``right_finger`` pose matrices, independently of the target names in the
    MuJoCo model.
    """

    GRIPPER_KEY = "gripper"
    FINGER_POSE_KEY = "gripper_finger_pose"
    LEFT_FINGER_KEY = "left_finger"
    RIGHT_FINGER_KEY = "right_finger"

    def __init__(
        self,
        env: gym.Env[dict[str, Any], dict[str, Any]],
        *,
        body_name: tuple[str, str] | None = None,
        site_name: tuple[str, str] | None = None,
        offsets: FingerPosePair | dict[str, FingerPosePair] | None = None,
        model_path: str | Path | None = None,
    ) -> None:
        super().__init__(env)
        if (body_name is None) == (site_name is None):
            msg = "exactly one of body_name or site_name must be provided"
            raise ValueError(msg)

        target_kind = "body" if body_name is not None else "site"
        target_names = body_name if body_name is not None else site_name
        if (
            not isinstance(target_names, tuple)
            or len(target_names) != 2
            or not all(isinstance(name, str) for name in target_names)
        ):
            msg = f"{target_kind}_name must be a pair of names ordered left then right"
            raise ValueError(msg)

        self.body_name = body_name
        self.site_name = site_name
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
            arm_space.spaces[self.FINGER_POSE_KEY] = gym.spaces.Dict(
                {
                    self.LEFT_FINGER_KEY: gym.spaces.Box(low=-np.inf, high=np.inf, shape=(4, 4), dtype=np.float64),
                    self.RIGHT_FINGER_KEY: gym.spaces.Box(low=-np.inf, high=np.inf, shape=(4, 4), dtype=np.float64),
                }
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

        # Build or load both complete caches now so observation calls only do
        # in-memory lookups.
        robotiq_2f85_finger_poses(
            0.0,
            body_name=self.body_name,
            site_name=self.site_name,
            model_path=self.model_path,
        )

    def _offsets_for_arm(self, arm_name: str) -> FingerPosePair | None:
        if isinstance(self.offsets, dict):
            return self.offsets.get(arm_name)
        return self.offsets

    def observation(self, observation: dict[str, Any], info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        observation = copy.deepcopy(observation)
        for arm_name in self.arm_names:
            arm_observation = observation[arm_name]
            gripper_state = np.asarray(arm_observation[self.GRIPPER_KEY], dtype=np.float64)
            if gripper_state.size != 1:
                msg = f"Expected one gripper value for arm {arm_name!r}, got shape {gripper_state.shape}"
                raise ValueError(msg)

            poses = robotiq_2f85_finger_poses(
                float(gripper_state.item()),
                body_name=self.body_name,
                site_name=self.site_name,
                offsets=self._offsets_for_arm(arm_name),
                model_path=self.model_path,
            )
            arm_observation[self.FINGER_POSE_KEY] = {
                self.LEFT_FINGER_KEY: poses.left.pose_matrix(),
                self.RIGHT_FINGER_KEY: poses.right.pose_matrix(),
            }

        return observation, info
