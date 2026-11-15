"""Hardware env creators for the interface, needs the rcs_fr3 extension.

The interface always talks to a multi robot env (actions and observations keyed by robot name, like the
Quest operator produces them), so a single FR3 is wrapped as a multi robot env with one robot ``right``.
"""

import numpy as np
from rcs.envs.base import ControlMode, RelativeTo
from rcs_fr3.configs import DefaultFR3HardwareEnv
from rcs_fr3.creators import (
    FR3MultiHardwareEnvCreatorConfig,
    RCSFR3MultiConfigEnvCreator,
)

from rcs import common


class SingleFR3Env(RCSFR3MultiConfigEnvCreator):
    """One FR3 with Franka Hand, robot name ``right`` (the right Quest controller). No cameras by default,
    add them with a config file (see ``rcs-webui dump-config``)."""

    ip = "192.168.101.1"

    def config(self) -> FR3MultiHardwareEnvCreatorConfig:
        base = DefaultFR3HardwareEnv()
        base.ip = self.ip
        cfg = base.config()
        cfg.robot_cfg.async_control = True
        if cfg.gripper_cfg is not None and hasattr(cfg.gripper_cfg, "async_control"):
            cfg.gripper_cfg.async_control = True
        return FR3MultiHardwareEnvCreatorConfig(
            control_mode=ControlMode.CARTESIAN_TQuat,
            robot_cfgs={"right": cfg.robot_cfg},
            gripper_cfgs={"right": cfg.gripper_cfg},
            camera_cfgs=None,
            max_relative_movement=(0.5, np.deg2rad(90)),
            relative_to=RelativeTo.CONFIGURED_ORIGIN,
            robot_to_shared_base_frame={"right": common.Pose()},
        )
