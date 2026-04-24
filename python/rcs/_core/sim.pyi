# ATTENTION: auto generated from C++ code, use `make stubgen` to update!
"""
sim module
"""
from __future__ import annotations

import typing

import numpy
import rcs._core.common

__all__: list[str] = [
    "CameraType",
    "FrameSet",
    "GuiClient",
    "Sim",
    "SimCameraConfig",
    "SimCameraSet",
    "SimConfig",
    "SimGripper",
    "SimGripperConfig",
    "SimGripperState",
    "SimRobot",
    "SimRobotConfig",
    "SimRobotState",
    "SimTilburgHand",
    "SimTilburgHandConfig",
    "SimTilburgHandState",
    "default_free",
    "fixed",
    "free",
    "tracking",
]
M = typing.TypeVar("M", bound=int)
N = typing.TypeVar("N", bound=int)

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

class FrameSet:
    def __init__(
        self,
        color_frames: dict[str, numpy.ndarray[tuple[M], numpy.dtype[numpy.uint8]]],
        depth_frames: dict[str, numpy.ndarray[tuple[M], numpy.dtype[numpy.float32]]],
        timestamp: float,
    ) -> None: ...
    @property
    def color_frames(self) -> dict[str, numpy.ndarray[tuple[M], numpy.dtype[numpy.uint8]]]: ...
    @property
    def depth_frames(self) -> dict[str, numpy.ndarray[tuple[M], numpy.dtype[numpy.float32]]]: ...
    @property
    def timestamp(self) -> float: ...

class GuiClient:
    def __init__(self, id: str) -> None: ...
    def get_model_bytes(self) -> bytes: ...
    def set_model_and_data(self, arg0: int, arg1: int) -> None: ...
    def sync(self) -> None: ...

class Sim:
    def __init__(self, mjmdl: int, mjdata: int) -> None: ...
    def _start_gui_server(self, id: str) -> None: ...
    def _stop_gui_server(self) -> None: ...
    def get_config(self) -> SimConfig: ...
    def is_converged(self) -> bool: ...
    def reset(self) -> None: ...
    def set_config(self, cfg: SimConfig) -> bool: ...
    def step(self, k: int) -> None: ...
    def step_until_convergence(self) -> None: ...

class SimCameraConfig(rcs._core.common.BaseCameraConfig):
    type: CameraType
    def __copy__(self) -> SimCameraConfig: ...
    def __deepcopy__(self, arg0: dict) -> SimCameraConfig: ...
    def __init__(
        self, identifier: str, frame_rate: int, resolution_width: int, resolution_height: int, type: CameraType = ...
    ) -> None: ...

class SimCameraSet:
    def __init__(self, sim: Sim, cameras: dict[str, SimCameraConfig], render_on_demand: bool = True) -> None: ...
    def buffer_size(self) -> int: ...
    def clear_buffer(self) -> None: ...
    def get_latest_frameset(self) -> FrameSet | None: ...
    def get_timestamp_frameset(self, ts: float) -> FrameSet | None: ...
    @property
    def _sim(self) -> Sim: ...

class SimConfig:
    async_control: bool
    frequency: float
    max_convergence_steps: int
    realtime: bool
    def __copy__(self) -> SimConfig: ...
    def __deepcopy__(self, arg0: dict) -> SimConfig: ...
    def __init__(
        self,
        async_control: bool = False,
        realtime: bool = False,
        frequency: float = 30.0,
        max_convergence_steps: int = 500,
    ) -> None: ...

class SimGripper(rcs._core.common.Gripper):
    def __init__(self, sim: Sim, cfg: SimGripperConfig) -> None: ...
    def clear_collision_flag(self) -> None: ...
    def get_config(self) -> SimGripperConfig: ...
    def get_state(self) -> SimGripperState: ...
    def set_config(self, cfg: SimGripperConfig) -> bool: ...

class SimGripperConfig(rcs._core.common.GripperConfig):
    actuator: str
    collision_geoms: list[str]
    collision_geoms_fingers: list[str]
    epsilon_inner: float
    epsilon_outer: float
    gripper_type: rcs._core.common.GripperType
    ignored_collision_geoms: list[str]
    joints: list[str]
    max_actuator_width: float
    max_joint_width: float
    min_actuator_width: float
    min_joint_width: float
    seconds_between_callbacks: float
    def __copy__(self) -> SimGripperConfig: ...
    def __deepcopy__(self, arg0: dict) -> SimGripperConfig: ...
    def __init__(
        self,
        epsilon_inner: float = 0.005,
        epsilon_outer: float = 0.005,
        seconds_between_callbacks: float = 0.05,
        ignored_collision_geoms: list[str] = [],
        collision_geoms: list[str] = ["hand_c", "d435i_collision", "finger_0_left", "finger_0_right"],
        collision_geoms_fingers: list[str] = ["finger_0_left", "finger_0_right"],
        joints: list[str] = ["finger_joint1", "finger_joint2"],
        max_joint_width: float = 0.04,
        min_joint_width: float = 0.0,
        actuator: str = "actuator8",
        max_actuator_width: float = 255.0,
        min_actuator_width: float = 0.0,
        gripper_type: rcs._core.common.GripperType = ...,
    ) -> None: ...
    def add_prefix(self, id: str) -> None: ...

class SimGripperState(rcs._core.common.GripperState):
    def __init__(self) -> None: ...
    @property
    def collision(self) -> bool: ...
    @property
    def is_moving(self) -> bool: ...
    @property
    def last_commanded_width(self) -> float: ...
    @property
    def last_width(self) -> float: ...

class SimRobot(rcs._core.common.Robot):
    def __init__(
        self, sim: Sim, ik: rcs._core.common.Kinematics, cfg: SimRobotConfig, register_convergence_callback: bool = True
    ) -> None: ...
    def clear_collision_flag(self) -> None: ...
    def get_config(self) -> SimRobotConfig: ...
    def get_state(self) -> SimRobotState: ...
    def set_config(self, cfg: SimRobotConfig) -> bool: ...
    def set_joints_hard(self, q: numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]) -> None: ...

class SimRobotConfig(rcs._core.common.RobotConfig):
    actuators: list[str]
    arm_collision_geoms: list[str]
    base: str
    dof: int
    joint_limits: numpy.ndarray[tuple[typing.Literal[2], typing.Any], numpy.dtype[numpy.float64]]
    joint_rotational_tolerance: float
    joints: list[str]
    seconds_between_callbacks: float
    trajectory_trace: bool
    def __copy__(self) -> SimRobotConfig: ...
    def __deepcopy__(self, arg0: dict) -> SimRobotConfig: ...
    def __init__(
        self,
        robot_type: rcs._core.common.RobotType = ...,
        tcp_offset: rcs._core.common.Pose = ...,
        attachment_site: str = "attachment_site",
        kinematic_model_path: str = "assets/scenes/fr3_empty_world/robot.xml",
        joint_rotational_tolerance: float = 0.0008726646259971648,
        seconds_between_callbacks: float = 0.1,
        trajectory_trace: bool = False,
        arm_collision_geoms: list[str] = [
            "fr3_link0_collision",
            "fr3_link1_collision",
            "fr3_link2_collision",
            "fr3_link3_collision",
            "fr3_link4_collision",
            "fr3_link5_collision",
            "fr3_link6_collision",
            "fr3_link7_collision",
        ],
        joints: list[str] = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ],
        q_home: numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]] | None = None,
        actuators: list[str] = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ],
        base: str = "base",
        dof: int = 7,
        joint_limits: numpy.ndarray[tuple[typing.Literal[2], typing.Any], numpy.dtype[numpy.float64]] = ...,
    ) -> None: ...
    def add_prefix(self, id: str) -> None: ...

class SimRobotState(rcs._core.common.RobotState):
    def __init__(self) -> None: ...
    @property
    def collision(self) -> bool: ...
    @property
    def ik_success(self) -> bool: ...
    @property
    def inverse_tcp_offset(self) -> rcs._core.common.Pose: ...
    @property
    def is_arrived(self) -> bool: ...
    @property
    def is_moving(self) -> bool: ...
    @property
    def previous_angles(self) -> numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]: ...
    @property
    def target_angles(self) -> numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]: ...

class SimTilburgHand(rcs._core.common.Hand):
    def __init__(self, sim: Sim, cfg: SimTilburgHandConfig) -> None: ...
    def get_config(self) -> SimTilburgHandConfig: ...
    def get_state(self) -> SimTilburgHandState: ...
    def set_config(self, cfg: SimTilburgHandConfig) -> bool: ...

class SimTilburgHandConfig(rcs._core.common.HandConfig):
    actuators: list[str]
    collision_geoms: list[str]
    collision_geoms_fingers: list[str]
    grasp_type: rcs._core.common.GraspType
    ignored_collision_geoms: list[str]
    joints: list[str]
    max_joint_position: numpy.ndarray[tuple[typing.Literal[16]], numpy.dtype[numpy.float64]]
    min_joint_position: numpy.ndarray[tuple[typing.Literal[16]], numpy.dtype[numpy.float64]]
    seconds_between_callbacks: float
    def __copy__(self) -> SimTilburgHandConfig: ...
    def __deepcopy__(self, arg0: dict) -> SimTilburgHandConfig: ...
    def __init__(
        self,
        grasp_type: rcs._core.common.GraspType = ...,
        seconds_between_callbacks: float = 0.0167,
        ignored_collision_geoms: list[str] = [],
        collision_geoms: list[str] = [],
        collision_geoms_fingers: list[str] = [],
        joints: list[str] = [
            "thumb_ip",
            "thumb_mcp",
            "thumb_mcp_rot",
            "thumb_cmc",
            "index_dip",
            "index_pip",
            "index_mcp",
            "index_mcp_abadd",
            "middle_dip",
            "middle_pip",
            "middle_mcp",
            "middle_mcp_abadd",
            "ring_dip",
            "ring_pip",
            "ring_mcp",
            "ring_mcp_abadd",
        ],
        actuators: list[str] = [
            "thumb_ip",
            "thumb_mcp",
            "thumb_mcp_rot",
            "thumb_cmc",
            "index_dip",
            "index_pip",
            "index_mcp",
            "index_mcp_abadd",
            "middle_dip",
            "middle_pip",
            "middle_mcp",
            "middle_mcp_abadd",
            "ring_dip",
            "ring_pip",
            "ring_mcp",
            "ring_mcp_abadd",
        ],
        max_joint_position: numpy.ndarray[tuple[typing.Literal[16]], numpy.dtype[numpy.float64]] = ...,
        min_joint_position: numpy.ndarray[tuple[typing.Literal[16]], numpy.dtype[numpy.float64]] = ...,
    ) -> None: ...
    def add_prefix(self, id: str) -> None: ...

class SimTilburgHandState(rcs._core.common.HandState):
    def __init__(self) -> None: ...
    @property
    def collision(self) -> bool: ...
    @property
    def is_moving(self) -> bool: ...
    @property
    def last_commanded_qpos(self) -> numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]: ...
    @property
    def last_qpos(self) -> numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]: ...

default_free: CameraType  # value = <CameraType.default_free: 3>
fixed: CameraType  # value = <CameraType.fixed: 2>
free: CameraType  # value = <CameraType.free: 0>
tracking: CameraType  # value = <CameraType.tracking: 1>
