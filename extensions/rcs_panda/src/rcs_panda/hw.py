"""The Franka hardware classes of the C++ module, and the config of the Panda.

The robot specific values (type, model, IK site, dof, joint limits, home pose)
are only defined in rcs.ROBOTS, so the robot specific config lives here and not
in C++.
"""

from typing import Any

from rcs_panda._core.hw import (
    FHConfig,
    FHState,
    Franka,
    FrankaConfig,
    FrankaHand,
    FrankaLoad,
    FrankaState,
    IKSolver,
    RobotMode,
    RobotState,
    exceptions,
)

import rcs
from rcs import common


class PandaConfig(FrankaConfig):
    """FrankaConfig of the Panda, with its values from rcs.ROBOTS.

    Further fields can be given as keyword arguments, e.g.
    PandaConfig(ip="192.168.1.11", async_control=True, policy_rate=15).
    """

    def __init__(self, ip: str, **kwargs: Any):
        super().__init__()
        rcs.ROBOTS[common.RobotType.Panda].apply(self)
        self.ip = ip
        for key, value in kwargs.items():
            if not hasattr(self, key):
                msg = f"PandaConfig has no field {key}"
                raise TypeError(msg)
            setattr(self, key, value)


__all__ = [
    "FHConfig",
    "FHState",
    "Franka",
    "FrankaConfig",
    "FrankaHand",
    "FrankaLoad",
    "FrankaState",
    "PandaConfig",
    "IKSolver",
    "RobotMode",
    "RobotState",
    "exceptions",
]
