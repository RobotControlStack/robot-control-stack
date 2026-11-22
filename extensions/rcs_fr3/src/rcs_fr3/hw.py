"""The Franka hardware classes of the C++ module, and the config of the FR3.

The robot specific values (type, model, IK site, dof, joint limits, home pose)
are only defined in rcs.ROBOTS, so the robot specific config lives here and not
in C++.
"""

from typing import Any

from rcs_fr3._core import hw as _core_hw
from rcs_fr3._core.hw import *  # noqa: F403 (all classes of the C++ module)
from rcs_fr3._core.hw import FrankaConfig

import rcs
from rcs import common


class FR3Config(FrankaConfig):
    """FrankaConfig of the FR3, with its values from rcs.ROBOTS.

    Further fields can be given as keyword arguments, e.g.
    FR3Config(ip="192.168.1.11", async_control=True, policy_rate=15).
    """

    def __init__(self, ip: str, **kwargs: Any):
        super().__init__()
        rcs.ROBOTS[common.RobotType.FR3].apply(self)
        self.ip = ip
        for key, value in kwargs.items():
            if not hasattr(self, key):
                msg = f"FR3Config has no field {key}"
                raise TypeError(msg)
            setattr(self, key, value)


__all__ = [name for name in dir(_core_hw) if not name.startswith("_")] + ["FR3Config"]
