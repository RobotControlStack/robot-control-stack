# ATTENTION: auto generated from C++ code, use `make genstub` to update!
"""
hardware module
"""
from __future__ import annotations

import typing

import numpy
import rcsss._core.common

__all__ = [
    "FHConfig",
    "FHState",
    "FR3",
    "FR3Config",
    "FR3Load",
    "FR3State",
    "FrankaHand",
    "IKController",
    "internal",
    "robotics_library",
]

class FHConfig:
    epsilon_inner: float
    epsilon_outer: float
    force: float
    grasping_width: float
    speed: float

class FHState:
    @property
    def is_grasped(self) -> bool: ...
    @property
    def temperature(self) -> int: ...
    @property
    def width(self) -> float: ...

class FR3(rcsss._core.common.Robot):
    def __init__(self, ip: str, filename: str | None = None) -> None: ...
    def automatic_error_recovery(self) -> None: ...
    def double_tap_robot_to_continue(self) -> None: ...
    def set_cartesian_position_internal(self, pose: rcsss._core.common.Pose) -> None: ...
    def set_cartesian_position_rl(
        self, pose: rcsss._core.common.Pose, max_time: float, elbow: float | None, max_force: float | None = 5
    ) -> None: ...
    def set_default_robot_behavior(self) -> None: ...
    def set_guiding_mode(self, enabled: bool) -> None: ...

class FR3Config:
    controller: IKController
    guiding_mode_enabled: bool
    load_parameters: FR3Load | None
    nominal_end_effector_frame: rcsss._core.common.Pose | None

class FR3Load:
    f_x_cload: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]] | None
    load_inertia: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]] | None
    load_mass: float

class FR3State:
    pass

class FrankaHand(rcsss._core.common.Gripper):
    def __init__(self, arg0: str) -> None: ...
    def homing(self) -> bool: ...

class IKController:
    """
    Members:

      internal

      robotics_library
    """

    __members__: typing.ClassVar[
        dict[str, IKController]
    ]  # value = {'internal': <IKController.internal: 0>, 'robotics_library': <IKController.robotics_library: 1>}
    internal: typing.ClassVar[IKController]  # value = <IKController.internal: 0>
    robotics_library: typing.ClassVar[IKController]  # value = <IKController.robotics_library: 1>
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

internal: IKController  # value = <IKController.internal: 0>
robotics_library: IKController  # value = <IKController.robotics_library: 1>
