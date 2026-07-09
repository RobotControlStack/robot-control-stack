from typing import Optional

import numpy as np
from numpy.typing import NDArray
from rcs._core.common import Kinematics
from rcs_fr3._core.hw import Franka, FrankaConfig


class BilateralControlMode:
    bilateral: "BilateralControlMode"
    gravity_only: "BilateralControlMode"


class BilateralFrankaConfig:
    leader_cfg: FrankaConfig
    follower_cfg: FrankaConfig
    control_mode: BilateralControlMode
    update_rate_hz: float
    follower_joint_position_scale: float
    haptic_feedback_gain: float
    max_follower_joint_step: float
    relative_joint_mapping: bool
    leader_haptic_feedback: bool
    feedback_avoidance_alpha: NDArray[np.float64]

    def __init__(
        self,
        leader_cfg: FrankaConfig,
        follower_cfg: FrankaConfig,
        control_mode: BilateralControlMode = ...,
        update_rate_hz: float = 1000.0,
        follower_joint_position_scale: float = 1.0,
        haptic_feedback_gain: float = 1.0,
        max_follower_joint_step: float = 0.05,
        relative_joint_mapping: bool = True,
        leader_haptic_feedback: bool = True,
        feedback_avoidance_alpha: NDArray[np.float64] = ...,
    ) -> None: ...


class BilateralFrankaState:
    leader_q: NDArray[np.float64]
    leader_dq: NDArray[np.float64]
    follower_q: NDArray[np.float64]
    follower_dq: NDArray[np.float64]
    follower_target_q: NDArray[np.float64]
    follower_external_tau: NDArray[np.float64]
    leader_torque_command: NDArray[np.float64]
    running: bool
    has_reference: bool

    def __init__(self) -> None: ...


class BilateralFranka:
    def __init__(
        self,
        cfg: BilateralFrankaConfig,
        leader_ik: Optional[Kinematics] = None,
        follower_ik: Optional[Kinematics] = None,
    ) -> None: ...
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def move_home(self) -> None: ...
    def update_once(self) -> None: ...
    def reset(self) -> None: ...
    def is_running(self) -> bool: ...
    def get_config(self) -> BilateralFrankaConfig: ...
    def get_state(self) -> BilateralFrankaState: ...
    def get_leader(self) -> Franka: ...
    def get_follower(self) -> Franka: ...
