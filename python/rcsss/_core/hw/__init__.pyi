from enum import Enum
from typing import Optional

from _core.common import (
    GConfig,
    Gripper,
    GState,
    Matrix3d,
    Pose,
    RConfig,
    Robot,
    RState,
    Vector3d,
)

class FR3(Robot):
    def __init__(self, ip: str, filename: str) -> None: ...
    def set_default_robot_behavior(self) -> None: ...
    def set_guiding_mode(self, enabled: bool) -> None: ...
    def automatic_error_recovery(self) -> None: ...
    def double_tap_robot_to_continue(self) -> None: ...
    def set_cartesian_position_internal(self, pose: Pose) -> None: ...
    def set_cartesian_position_rl(self, pose: Pose, max_time: float, elbow: float, max_force: float) -> None: ...

class FrankaHand(Gripper):
    def __init__(self, ip) -> None: ...
    def homing(self) -> None: ...

class FR3State(RState):
    pass

class FR3Load:
    load_mass: float
    f_x_cload: Optional[Vector3d]
    load_inertia: Optional[Matrix3d]

class IKController(Enum):
    internal = 0
    robotics_library = 1

class FR3Config(RConfig):
    controller: IKController
    guiding_mode_enabled: bool
    speed_scaling: float
    load_parameters: Optional[FR3Load]
    nominal_end_effector_pose: Optional[Pose]

class FHConfig(GConfig):
    grasping_width: float
    speed: float
    force: float
    epsilon_inner: float
    epsilon_outer: float

class FHState(GState):
    width: float
    is_grasped: bool
    temperature: float
