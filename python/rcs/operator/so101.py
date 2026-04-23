import logging
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np

from rcs.envs.base import ControlMode, RelativeTo
from rcs.operator.interface import BaseOperator, BaseOperatorConfig, TeleopCommands
from rcs.sim.sim import Sim
from rcs.utils import SimpleFrameRate

logger = logging.getLogger(__name__)

LeaderFactory = Callable[["SO101OperatorConfig"], Any]

JOINT_KEYS = (
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
)
GRIPPER_KEY = "gripper.pos"


def _load_so101_leader_classes() -> tuple[type[Any], type[Any]]:
    try:
        from lerobot.teleoperators.so_leader.config_so_leader import (
            SO101LeaderConfig as LeRobotSO101LeaderConfig,
        )
        from lerobot.teleoperators.so_leader.so_leader import (
            SO101Leader as LeRobotSO101Leader,
        )
    except ImportError as exc:
        msg = (
            "lerobot SO101 leader dependencies are not available. "
            "Install lerobot and its teleoperator dependencies to use SO101Operator."
        )
        raise ImportError(msg) from exc

    return LeRobotSO101LeaderConfig, LeRobotSO101Leader


def default_so101_leader_factory(config: "SO101OperatorConfig") -> Any:
    leader_config_cls, leader_cls = _load_so101_leader_classes()

    leader_kwargs: dict[str, Any] = {
        "id": config.id,
        "port": config.port,
        "use_degrees": config.use_degrees,
    }
    if config.calibration_dir is not None:
        leader_kwargs["calibration_dir"] = Path(config.calibration_dir)

    leader = leader_cls(leader_config_cls(**leader_kwargs))
    leader.connect()
    return leader


class SO101Operator(BaseOperator):
    control_mode = (ControlMode.JOINTS, RelativeTo.NONE)

    def __init__(self, config: "SO101OperatorConfig", sim: Sim | None = None):
        super().__init__(config, sim)
        self.config: SO101OperatorConfig
        self._resource_lock = threading.Lock()
        self._exit_requested = False
        self.controller_names = [self.config.controller_name]
        self._last_joints: np.ndarray | None = None
        self._last_gripper = 1.0
        self._leader: Any | None = None
        self._leader_factory = self.config.leader_factory or default_so101_leader_factory

    @staticmethod
    def _leader_action_to_target(action: dict[str, float], use_degrees: bool) -> tuple[np.ndarray, float]:
        joints = np.array([action[key] for key in JOINT_KEYS], dtype=np.float64)
        if use_degrees:
            joints = np.deg2rad(joints)

        gripper = float(np.clip(action.get(GRIPPER_KEY, 100.0) / 100.0, 0.0, 1.0))
        return joints, gripper

    def consume_commands(self) -> TeleopCommands:
        return TeleopCommands()

    def reset_operator_state(self):
        pass

    def consume_action(self) -> dict[str, Any]:
        with self._resource_lock:
            if self._last_joints is None:
                return {}

            return {
                self.config.controller_name: {
                    "joints": self._last_joints.copy(),
                    "gripper": np.array([self._last_gripper], dtype=np.float32),
                }
            }

    def run(self):
        try:
            self._leader = self._leader_factory(self.config)
        except Exception as exc:
            logger.error(f"Failed to initialize SO101 leader: {exc}")
            return

        rate_limiter = SimpleFrameRate(self.config.read_frequency, "so101 readout")

        while not self._exit_requested:
            try:
                leader_action = self._leader.get_action()
                joints, gripper = self._leader_action_to_target(leader_action, self.config.use_degrees)
                with self._resource_lock:
                    self._last_joints = joints
                    self._last_gripper = gripper
            except Exception as exc:
                logger.warning(f"Error reading SO101 leader state: {exc}")

            rate_limiter()

    def close(self):
        self._exit_requested = True
        if self._leader is not None:
            try:
                self._leader.disconnect()
            except Exception:
                logger.debug("Failed to disconnect SO101 leader cleanly.", exc_info=True)
        if self.is_alive() and threading.current_thread() != self:
            self.join(timeout=1.0)


@dataclass(kw_only=True)
class SO101OperatorConfig(BaseOperatorConfig):
    operator_class: type[BaseOperator] = field(default=SO101Operator)
    controller_name: str = "so101"
    id: str = "leader"
    port: str = "/dev/ttyACM1"
    calibration_dir: str | None = None
    use_degrees: bool = True
    leader_factory: LeaderFactory | None = None
