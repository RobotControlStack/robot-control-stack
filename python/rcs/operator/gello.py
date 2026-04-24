import contextlib
import copy
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any, ClassVar, Dict, Iterator, List, Sequence, Tuple

import numpy as np

try:
    from dynamixel_sdk.group_sync_read import GroupSyncRead
    from dynamixel_sdk.group_sync_write import GroupSyncWrite
    from dynamixel_sdk.packet_handler import PacketHandler
    from dynamixel_sdk.port_handler import PortHandler
    from dynamixel_sdk.robotis_def import COMM_SUCCESS

    HAS_DYNAMIXEL_SDK = True
except ImportError:
    HAS_DYNAMIXEL_SDK = False

try:
    from pynput import keyboard

    HAS_PYNPUT = True
except ImportError:
    HAS_PYNPUT = False

from rcs.envs.base import ControlMode, RelativeTo
from rcs.operator.interface import BaseOperator, BaseOperatorConfig, TeleopCommands
from rcs.sim.sim import Sim
from rcs.utils import SimpleFrameRate

logger = logging.getLogger(__name__)

# --- Dynamixel Driver Constants and Helpers ---

XL330_CONTROL_TABLE = {
    "model_number": {"addr": 0, "len": 2},
    "operating_mode": {"addr": 11, "len": 1},
    "torque_enable": {"addr": 64, "len": 1},
    "kp_d": {"addr": 80, "len": 2},
    "kp_i": {"addr": 82, "len": 2},
    "kp_p": {"addr": 84, "len": 2},
    "goal_current": {"addr": 102, "len": 2},
    "goal_position": {"addr": 116, "len": 4},
    "present_position": {"addr": 132, "len": 4},
}


class DynamixelDriver:
    """Simplified Dynamixel driver adapted for RCS."""

    def __init__(
        self,
        ids: Sequence[int],
        port: str = "/dev/ttyUSB0",
        baudrate: int = 57600,
        pulses_per_revolution: int = 4095,
    ):
        if not HAS_DYNAMIXEL_SDK:
            msg = "dynamixel_sdk is not installed. Please install it to use GelloOperator."
            raise ImportError(msg)

        self._ids = ids
        self._port = port
        self._baudrate = baudrate
        self._pulses_per_revolution = pulses_per_revolution
        self._lock = threading.Lock()
        self._buffered_joint_positions: np.ndarray | None = None

        self._portHandler = PortHandler(self._port)
        self._packetHandler = PacketHandler(2.0)

        self._groupSyncReadHandlers = {}
        self._groupSyncWriteHandlers = {}

        for key, entry in XL330_CONTROL_TABLE.items():
            self._groupSyncReadHandlers[key] = GroupSyncRead(
                self._portHandler, self._packetHandler, entry["addr"], entry["len"]
            )
            for dxl_id in self._ids:
                self._groupSyncReadHandlers[key].addParam(dxl_id)

            if key not in {"model_number", "present_position"}:
                self._groupSyncWriteHandlers[key] = GroupSyncWrite(
                    self._portHandler, self._packetHandler, entry["addr"], entry["len"]
                )

        if not self._portHandler.openPort():
            msg = f"Failed to open port {self._port}"
            raise ConnectionError(msg)
        if not self._portHandler.setBaudRate(self._baudrate):
            msg = f"Failed to set baudrate {self._baudrate}"
            raise ConnectionError(msg)

        self._stop_thread = threading.Event()
        self._polling_thread: threading.Thread | None = None
        self._is_polling = False

    def write_value_by_name(self, name: str, values: Sequence[int | None]):
        if len(values) != len(self._ids):
            msg = f"The length of {name} must match the number of servos"
            raise ValueError(msg)

        handler = self._groupSyncWriteHandlers[name]
        value_len = XL330_CONTROL_TABLE[name]["len"]

        with self._lock:
            for dxl_id, value in zip(self._ids, values, strict=False):
                if value is None:
                    continue
                param = [(value >> (8 * i)) & 0xFF for i in range(value_len)]
                handler.addParam(dxl_id, param)

            comm_result = handler.txPacket()
            if comm_result != COMM_SUCCESS:
                handler.clearParam()
                msg = f"Failed to syncwrite {name}: {self._packetHandler.getTxRxResult(comm_result)}"
                raise RuntimeError(msg)
            handler.clearParam()

    def read_value_by_name(self, name: str) -> List[int]:
        handler = self._groupSyncReadHandlers[name]
        value_len = XL330_CONTROL_TABLE[name]["len"]
        addr = XL330_CONTROL_TABLE[name]["addr"]

        with self._lock:
            comm_result = handler.txRxPacket()
            if comm_result != COMM_SUCCESS:
                msg = f"Failed to sync read {name}: {self._packetHandler.getTxRxResult(comm_result)}"
                raise RuntimeError(msg)

            values = []
            for dxl_id in self._ids:
                if handler.isAvailable(dxl_id, addr, value_len):
                    value = handler.getData(dxl_id, addr, value_len)
                    value = int(np.int32(np.uint32(value)))
                    values.append(value)
                else:
                    msg = f"Failed to get {name} for ID {dxl_id}"
                    raise RuntimeError(msg)
            return values

    def start_joint_polling(self):
        if self._is_polling:
            return
        self._stop_thread.clear()
        self._polling_thread = threading.Thread(target=self._joint_polling_loop, daemon=True)
        self._polling_thread.start()
        self._is_polling = True

    def stop_joint_polling(self):
        if not self._is_polling:
            return
        self._stop_thread.set()
        if self._polling_thread:
            self._polling_thread.join()
        self._is_polling = False

    def _joint_polling_loop(self):
        while not self._stop_thread.is_set():
            time.sleep(0.001)
            try:
                self._buffered_joint_positions = np.array(self.read_value_by_name("present_position"), dtype=int)
            except RuntimeError as e:
                logger.warning(f"Polling error: {e}")

    def get_joints(self) -> np.ndarray:
        if self._is_polling:
            while self._buffered_joint_positions is None:
                time.sleep(0.01)
            return self._pulses_to_rad(self._buffered_joint_positions.copy())
        return self._pulses_to_rad(np.array(self.read_value_by_name("present_position"), dtype=int))

    def _pulses_to_rad(self, pulses) -> np.ndarray:
        return np.array(pulses) / self._pulses_per_revolution * 2 * np.pi

    def _rad_to_pulses(self, rad: float) -> int:
        return int(rad / (2 * np.pi) * self._pulses_per_revolution)

    def close(self):
        self.stop_joint_polling()
        if self._portHandler:
            self._portHandler.closePort()


# --- Gello Hardware Interface Logic ---


@dataclass
class GelloArmConfig:
    com_port: str = "/dev/ttyUSB0"
    num_arm_joints: int = 7
    joint_signs: List[int] = field(default_factory=lambda: [1, -1, 1, -1, 1, 1, 1])
    gripper: bool = True
    gripper_range_rad: List[float] = field(default_factory=lambda: [2.23, 3.22])
    assembly_offsets: List[float] = field(default_factory=lambda: [0.000, 0.000, 3.142, 3.142, 3.142, 4.712, 0.000])
    dynamixel_kp_p: List[int] = field(default_factory=lambda: [30, 60, 0, 30, 0, 0, 0, 50])
    dynamixel_kp_i: List[int] = field(default_factory=lambda: [0, 0, 0, 0, 0, 0, 0, 0])
    dynamixel_kp_d: List[int] = field(default_factory=lambda: [250, 100, 80, 60, 30, 10, 5, 0])
    dynamixel_torque_enable: List[int] = field(default_factory=lambda: [0, 0, 0, 0, 0, 0, 0, 0])
    dynamixel_goal_position: List[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0, -1.571, 0.0, 1.571, 0.0, 3.509]
    )


@dataclass
class DynamixelControlConfig:
    kp_p: List[int] = field(default_factory=list)
    kp_i: List[int] = field(default_factory=list)
    kp_d: List[int] = field(default_factory=list)
    torque_enable: List[int] = field(default_factory=list)
    goal_position: List[int] = field(default_factory=list)
    goal_current: List[int] = field(default_factory=list)
    operating_mode: List[int] = field(default_factory=list)

    _UPDATE_ORDER: ClassVar[list[str]] = [
        "operating_mode",
        "goal_current",
        "kp_p",
        "kp_i",
        "kp_d",
        "torque_enable",
        "goal_position",
    ]

    def __iter__(self) -> Iterator[Tuple[str, List[int]]]:
        for param_name in self._UPDATE_ORDER:
            if hasattr(self, param_name):
                yield param_name, getattr(self, param_name)

    def __getitem__(self, param_name: str) -> List[int]:
        return getattr(self, param_name)

    def __setitem__(self, param_name: str, value: List[int]) -> None:
        setattr(self, param_name, value)


class GelloHardware:
    JOINT_POSITION_LIMITS = np.array(
        [
            [-2.9007, 2.9007],
            [-1.8361, 1.8361],
            [-2.9007, 2.9007],
            [-3.0770, -0.1169],
            [-2.8763, 2.8763],
            [0.4398, 4.6216],
            [-3.0508, 3.0508],
        ]
    )
    MID_JOINT_POSITIONS = JOINT_POSITION_LIMITS.mean(axis=1)
    OPERATING_MODE = 5
    CURRENT_LIMIT = 600

    def __init__(self, config: GelloArmConfig):
        self._com_port = config.com_port
        self._num_arm_joints = config.num_arm_joints
        self._joint_signs = np.array(config.joint_signs)
        self._gripper = config.gripper
        self._num_total_joints = self._num_arm_joints + (1 if self._gripper else 0)
        self._gripper_range_rad = config.gripper_range_rad
        self._assembly_offsets = np.array(config.assembly_offsets)

        self._driver = DynamixelDriver(
            ids=list(range(1, self._num_total_joints + 1)),
            port=self._com_port,
        )

        self._initial_arm_joints_raw = self._driver.get_joints()[: self._num_arm_joints]
        initial_arm_joints = self.normalize_joint_positions(
            self._initial_arm_joints_raw, self._assembly_offsets, self._joint_signs
        )
        self._prev_arm_joints_raw = self._initial_arm_joints_raw.copy()
        self._prev_arm_joints = initial_arm_joints.copy()

        self._dynamixel_control_config = DynamixelControlConfig(
            kp_p=config.dynamixel_kp_p.copy(),
            kp_i=config.dynamixel_kp_i.copy(),
            kp_d=config.dynamixel_kp_d.copy(),
            torque_enable=config.dynamixel_torque_enable.copy(),
            goal_position=self._goal_position_to_pulses(config.dynamixel_goal_position).copy(),
            goal_current=[self.CURRENT_LIMIT] * self._num_total_joints,
            operating_mode=[self.OPERATING_MODE] * self._num_total_joints,
        )

        self._initialize_parameters()
        self._driver.start_joint_polling()

    @staticmethod
    def normalize_joint_positions(raw, offsets, signs):
        return (
            np.mod((raw - offsets) * signs - GelloHardware.MID_JOINT_POSITIONS, 2 * np.pi)
            - np.pi
            + GelloHardware.MID_JOINT_POSITIONS
        )

    def _initialize_parameters(self):
        for name, value in self._dynamixel_control_config:
            self._driver.write_value_by_name(name, value)
        time.sleep(0.1)

    def get_joint_and_gripper_positions(self) -> Tuple[np.ndarray, float]:
        joints_raw = self._driver.get_joints()
        arm_joints_raw = joints_raw[: self._num_arm_joints]

        arm_joints_delta = (arm_joints_raw - self._prev_arm_joints_raw) * self._joint_signs
        arm_joints = self._prev_arm_joints + arm_joints_delta
        self._prev_arm_joints = arm_joints.copy()
        self._prev_arm_joints_raw = arm_joints_raw.copy()

        arm_joints_clipped = np.clip(arm_joints, self.JOINT_POSITION_LIMITS[:, 0], self.JOINT_POSITION_LIMITS[:, 1])

        gripper_pos = 0.0
        if self._gripper:
            raw_grp = joints_raw[-1]
            gripper_pos = (raw_grp - self._gripper_range_rad[0]) / (
                self._gripper_range_rad[1] - self._gripper_range_rad[0]
            )
            gripper_pos = max(0.0, min(1.0, gripper_pos))

        return arm_joints_clipped, gripper_pos

    def _goal_position_to_pulses(self, goals):
        arm_goals = np.array(goals[: self._num_arm_joints])
        initial_rotations = np.floor_divide(
            self._initial_arm_joints_raw - self._assembly_offsets - self.MID_JOINT_POSITIONS, 2 * np.pi
        )
        arm_goals_raw = (initial_rotations * 2 * np.pi + arm_goals + self._assembly_offsets) * self._joint_signs + np.pi
        goals_raw = np.append(arm_goals_raw, goals[-1]) if self._gripper else arm_goals_raw
        return [self._driver._rad_to_pulses(rad) for rad in goals_raw]

    def close(self):
        with contextlib.suppress(Exception):
            self._driver.write_value_by_name("torque_enable", [0] * self._num_total_joints)
        self._driver.close()


# --- RCS Operator Implementation ---


class GelloOperator(BaseOperator):
    control_mode = (ControlMode.JOINTS, RelativeTo.NONE)

    def __init__(self, config: "GelloConfig", sim: Sim | None = None):
        super().__init__(config, sim)
        self.config: GelloConfig
        self._resource_lock = threading.Lock()
        self._cmd_lock = threading.Lock()

        self._exit_requested = False
        self._commands = TeleopCommands()

        self.controller_names = list(self.config.arms.keys())

        self._last_joints: Dict[str, np.ndarray | None] = {name: None for name in self.controller_names}
        self._last_gripper = {name: 1.0 for name in self.controller_names}
        self._hws: Dict[str, GelloHardware] = {}

        if HAS_PYNPUT:
            self._listener = keyboard.Listener(on_press=self._on_press)
            self._listener.start()
        else:
            logger.warning("pynput not found. Keyboard triggers disabled.")

    def _on_press(self, key):
        try:
            if hasattr(key, "char"):
                if key.char == "s":
                    with self._cmd_lock:
                        self._commands.sync_position = True
                elif key.char == "r":
                    with self._cmd_lock:
                        self._commands.failure = True
        except AttributeError:
            pass

    def consume_commands(self) -> TeleopCommands:
        with self._cmd_lock:
            cmds = copy.copy(self._commands)
            self._commands = TeleopCommands()
            return cmds

    def reset_operator_state(self):
        # GELLO is absolute, no internal state to reset typically
        pass

    def consume_action(self) -> Dict[str, Any]:
        actions: Dict[str, Any] = {}
        with self._resource_lock:
            for name in self.controller_names:
                joints = self._last_joints[name]
                if joints is not None:
                    actions[name] = {
                        "joints": joints.copy(),
                        "gripper": np.array([self._last_gripper[name]]),
                    }
        return actions

    def run(self):
        # Initialize all hardware instances
        for name, arm_cfg in self.config.arms.items():
            try:
                self._hws[name] = GelloHardware(arm_cfg)
            except Exception as e:
                logger.error(f"Failed to initialize GELLO hardware for {name}: {e}")

        if not self._hws:
            logger.error("No GELLO hardware initialized. Exiting.")
            return

        rate_limiter = SimpleFrameRate(self.config.read_frequency, "gello readout")

        while not self._exit_requested:
            for name, hw in self._hws.items():
                try:
                    joints, gripper = hw.get_joint_and_gripper_positions()
                    with self._resource_lock:
                        self._last_joints[name] = joints
                        self._last_gripper[name] = gripper
                except Exception as e:
                    logger.warning(f"Error reading GELLO {name}: {e}")

            rate_limiter()

    def close(self):
        self._exit_requested = True
        if HAS_PYNPUT and hasattr(self, "_listener"):
            self._listener.stop()
        for hw in self._hws.values():
            hw.close()
        if self.is_alive() and threading.current_thread() != self:
            self.join(timeout=1.0)


@dataclass(kw_only=True)
class GelloConfig(BaseOperatorConfig):
    operator_class: type[BaseOperator] = field(default=GelloOperator)
    # Dictionary for multi-arm setups: {"left": GelloArmConfig(...), "right": GelloArmConfig(...)}
    arms: Dict[str, GelloArmConfig] = field(default_factory=lambda: {"right": GelloArmConfig()})
