# ATTENTION: auto generated from C++ code, use `make stubgen` to update!
"""
rcs robotics library module
"""
from __future__ import annotations

import rcs._core.common

__all__ = ["RoboticsLibraryIK"]

class RoboticsLibraryIK(rcs._core.common.Kinematics):
    def __init__(self, urdf_path: str, max_duration_ms: int = 300) -> None: ...
