from __future__ import annotations

import frankik
import numpy as np
from rcs._core.common import Kinematics, Pose, RobotConfig


class PinocchioKinematics(Kinematics):
    """Numerical IK/FK for any MJCF or URDF robot, computed by frankik's Pinocchio based CLIK solver."""

    def __init__(self, path: str, tcp_frame: str, base_frame: str | None = None, dof: int | None = None):
        super().__init__()
        self.kinematics = frankik.PinocchioKinematics(path, tcp_frame=tcp_frame, base_frame=base_frame, dof=dof)

    @classmethod
    def from_robot_config(cls, cfg: RobotConfig) -> PinocchioKinematics:
        return cls(cfg.kinematic_model_path, cfg.attachment_site, cfg.base_frame, cfg.dof)

    def forward(self, q0: np.ndarray, tcp_offset: Pose | None = None) -> Pose:  # type: ignore[override]
        return Pose(pose_matrix=self.kinematics.forward(q0, self._matrix(tcp_offset)))

    def inverse(  # type: ignore[override]
        self, pose: Pose, q0: np.ndarray, tcp_offset: Pose | None = None
    ) -> np.ndarray | None:
        return self.kinematics.inverse(pose.pose_matrix(), q0, self._matrix(tcp_offset))

    @staticmethod
    def _matrix(pose: Pose | None) -> np.ndarray | None:
        return None if pose is None else pose.pose_matrix()
