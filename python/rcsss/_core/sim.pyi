# ATTENTION: auto generated from C++ code, use `make stubgen` to update!
"""
sim module
"""
from __future__ import annotations

import typing

import numpy
import rcsss._core.common

__all__ = [
    "CameraType",
    "FR3",
    "FR3Config",
    "FR3State",
    "FrameSet",
    "Sim",
    "SimCameraConfig",
    "SimCameraSet",
    "SimCameraSetConfig",
    "default_free",
    "fixed",
    "free",
    "tracking",
]
M = typing.TypeVar("M", bound=int)

class CameraType:
    """
    Members:

      free

      tracking

      fixed

      default_free
    """

    __members__: typing.ClassVar[
        dict[str, CameraType]
    ]  # value = {'free': <CameraType.free: 0>, 'tracking': <CameraType.tracking: 1>, 'fixed': <CameraType.fixed: 2>, 'default_free': <CameraType.default_free: 3>}
    default_free: typing.ClassVar[CameraType]  # value = <CameraType.default_free: 3>
    fixed: typing.ClassVar[CameraType]  # value = <CameraType.fixed: 2>
    free: typing.ClassVar[CameraType]  # value = <CameraType.free: 0>
    tracking: typing.ClassVar[CameraType]  # value = <CameraType.tracking: 1>
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

class FR3(rcsss._core.common.Robot):
    def __init__(self, sim: Sim, id: str, rlmdl: str) -> None: ...
    def get_parameters(self) -> FR3Config: ...
    def get_state(self) -> FR3State: ...
    def reset(self) -> None: ...
    def set_parameters(self, cfg: FR3Config) -> bool: ...

class FR3Config(rcsss._core.common.RConfig):
    ik_duration_in_milliseconds: int
    joint_rotational_tolerance: float
    realtime: bool
    seconds_between_callbacks: float
    tcp_offset: rcsss._core.common.Pose
    trajectory_trace: bool
    def __init__(self) -> None: ...

class FR3State(rcsss._core.common.RState):
    def __init__(self) -> None: ...
    @property
    def collision(self) -> bool: ...
    @property
    def ik_success(self) -> bool: ...
    @property
    def inverse_tcp_offset(self) -> rcsss._core.common.Pose: ...
    @property
    def is_arrived(self) -> bool: ...
    @property
    def is_moving(self) -> bool: ...
    @property
    def previous_angles(self) -> numpy.ndarray[typing.Literal[7], numpy.dtype[numpy.float64]]: ...
    @property
    def target_angles(self) -> numpy.ndarray[typing.Literal[7], numpy.dtype[numpy.float64]]: ...

class FrameSet:
    def __init__(self) -> None: ...
    @property
    def color_frames(self) -> dict[str, numpy.ndarray[numpy.uint8[M, 1]]]: ...
    @property
    def timestamp(self) -> float: ...

class Sim:
    def __init__(self, mjmdl: int, mjdata: int) -> None: ...
    def step(self, k: int) -> None: ...
    def step_until_convergence(self) -> None: ...

class SimCameraConfig:
    identifier: str
    on_screen_render: bool
    type: CameraType
    def __init__(self) -> None: ...

class SimCameraSet:
    def __init__(self, sim: Sim, cfg: SimCameraSetConfig) -> None: ...
    def buffer_size(self) -> int: ...
    def clear_buffer(self) -> None: ...
    def get_latest_frameset(self) -> FrameSet | None: ...
    def get_timestamp_frameset(self, ts: float) -> FrameSet | None: ...

class SimCameraSetConfig:
    cameras: dict[str, SimCameraConfig]
    frame_rate: int
    resolution_height: int
    resolution_width: int
    def __init__(self) -> None: ...

default_free: CameraType  # value = <CameraType.default_free: 3>
fixed: CameraType  # value = <CameraType.fixed: 2>
free: CameraType  # value = <CameraType.free: 0>
tracking: CameraType  # value = <CameraType.tracking: 1>
