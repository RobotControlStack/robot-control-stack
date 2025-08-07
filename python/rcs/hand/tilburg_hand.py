import copy
import logging
from dataclasses import dataclass
from time import sleep

import numpy as np
from rcs._core import common
from rcs.envs.space_utils import Vec18Type
from tilburg_hand import Finger, TilburgHandMotorInterface, Unit

# Setup logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
logger.disabled = False


@dataclass(kw_only=True)
class THConfig(common.HandConfig):
    """Config for the Tilburg hand"""

    calibration_file: str | None = None
    grasp_percentage: float = 1.0
    control_unit: Unit = Unit.NORMALIZED
    hand_orientation: str = "right"
    grasp_type: common.GraspType = common.GraspType.POWER_GRASP

    def __post_init__(self):
        # 👇 satisfy pybind11 by actually calling the C++ constructor
        super().__init__()


@dataclass
class TilburgHandState(common.HandState):
    joint_positions: Vec18Type

    def __post_init__(self):
        super().__init__()


class TilburgHand(common.Hand):
    """
    Tilburg Hand Class
    This class provides an interface for controlling the Tilburg Hand.
    It allows for grasping, resetting, and disconnecting from the hand.
    """

    MAX_GRASP_JOINTS_VALS = np.array(
        [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0]
    )

    # TODO: Control mode for position control and pos+effort control

    POWER_GRASP_VALUES = np.array(
        [
            0.5, 0.5, 0.5, 1.4,  # THUMB_(IP, MCP, ABD, CMC)
            0.5, 0.5, 1.0, 0.7,  # INDEX_(DIP, PIP, MCP, ABD)
            0.5, 0.5, 1.0, 0.3,
            0.5, 0.5, 1.0, 0.0,
            0, 0
        ], dtype=np.float32
    )
    OPEN_VALUES = np.array(
        [
            0.0, 0.0, 0.5, 1.4,  # THUMB_(IP, MCP, ABD, CMC)
            0.2, 0.2, 0.2, 0.7,  # INDEX_(DIP, PIP, MCP, ABD)
            0.2, 0.2, 0.2, 0.3,
            0.2, 0.2, 0.2, 0.0,
            0, 0
        ], dtype=np.float32
    )

    def __init__(self, cfg: THConfig, verbose: bool = False):
        """
        Initializes the Tilburg Hand interface.
        """
        super().__init__()
        self._cfg = cfg

        self._motors = TilburgHandMotorInterface(
            calibration_file=self._cfg.calibration_file, hand_orientation=self._cfg.hand_orientation, verbose=verbose
        )

        re = self._motors.connect()
        assert re >= 0, "Failed to connect to the motors' board."

        logger.info("Connected to the motors' board.")

    @property
    def config(self):
        """
        Returns the configuration of the Tilburg Hand Control.
        """
        return copy.deepcopy(self._cfg)

    @config.setter
    def config(self, cfg: THConfig):
        """
        Sets the configuration of the Tilburg Hand Control.
        """
        self._cfg = cfg

    def set_pos_vector(self, pos_vector: Vec18Type):
        """
        Sets the position vector for the motors.
        """
        assert len(pos_vector) == (self._motors.n_motors), \
            f"Invalid position vector length: {len(pos_vector)}. Expected: {self._motors.n_motors}"
        self._motors.set_pos_vector(copy.deepcopy(pos_vector), unit=self._cfg.control_unit)

    def set_zero_pos(self):
        """
        Sets all finger joint positions to zero.
        """
        pos_normalized = 0 * self.MAX_GRASP_JOINTS_VALS
        self.set_pos_vector(pos_normalized)
        logger.info("All joints reset to zero position.")

    def set_joint_pos(self, finger_joint: Finger, pos_value: float):
        """
        Sets a single joint to a specific normalized position.
        """
        self._motors.set_pos_single(finger_joint, copy.deepcopy(pos_value), unit=self._cfg.control_unit)

    def reset_joint_pos(self, finger_joint: Finger):
        """
        Resets a specific joint to zero.
        """
        self._motors.set_pos_single(finger_joint, 0, unit=self._cfg.control_unit)
        logger.info(f"Reset joint {finger_joint.name} to 0")

    def disconnect(self):
        """
        Gracefully disconnects from the motor interface.
        """
        self._motors.disconnect()
        logger.info("Disconnected from the motors' board")

    def get_pos_vector(self) -> Vec18Type:
        """
        Returns the current position vector of the motors.
        """
        return np.array(self._motors.get_encoder_vector(self._cfg.control_unit))

    def get_pos_single(self, finger_joint: Finger) -> float:
        """
        Returns the current position of a single joint.
        """
        return self._motors.get_encoder_single(finger_joint, self._cfg.control_unit)

    def _grasp(self):
        if self._cfg.grasp_type == common.GraspType.POWER_GRASP:
            pos_normalized = self.POWER_GRASP_VALUES * self._cfg.grasp_percentage
        else:
            logger.warning(f"Grasp type {self._cfg.grasp_type} is not implemented. Defaulting to power grasp.")
            pos_normalized = self.POWER_GRASP_VALUES * self._cfg.grasp_percentage
        self.set_pos_vector(pos_normalized)

    def auto_recovery(self):
        if not np.array(self._motors.check_enabled_motors()).all():
            logger.warning("Some motors are not enabled. Attempting to enable them.")
            self._motors.disconnect()
            sleep(1)
            re = self._motors.connect()
            assert re >= 0, "Failed to reconnect to the motors' board."

    def set_grasp_type(self, grasp_type: common.GraspType):
        """
        Sets the grasp type for the hand.
        """
        if not isinstance(grasp_type, common.GraspType):
            error_msg = f"Invalid grasp type: {grasp_type}. Must be an instance of common.GraspType."
            raise ValueError(error_msg)
        if grasp_type == common.GraspType.POWER_GRASP:
            self._cfg.grasp_type = common.GraspType.POWER_GRASP
        elif grasp_type == common.GraspType.PRECISION_GRASP:
            logger.warning("Precision grasp is not implemented yet. Defaulting to power grasp.")
            self._cfg.grasp_type = common.GraspType.POWER_GRASP
        elif grasp_type == common.GraspType.LATERAL_GRASP:
            logger.warning("Lateral grasp is not implemented yet. Defaulting to power grasp.")
            self._cfg.grasp_type = common.GraspType.POWER_GRASP
        elif grasp_type == common.GraspType.TRIPOD_GRASP:
            logger.warning("Tripod grasp is not implemented yet. Defaulting to power grasp.")
            self._cfg.grasp_type = common.GraspType.POWER_GRASP
        else:
            error_msg = f"Unknown grasp type: {grasp_type}."
            raise ValueError(error_msg)

        logger.info(f"Grasp type set to: {self._cfg.grasp_type}")

    #### BaseHandControl Interface methods ####

    def grasp(self):
        """
        Performs a grasp with a specified intensity (0.0 to 1.0).
        """
        self._grasp()

    def open(self):
        self.set_pos_vector(self.OPEN_VALUES)

    def reset(self):
        """
        Resets the hand to its initial state.
        """
        self.auto_recovery()
        self.open()
        logger.info("Hand reset to initial state.")

    def get_state(self) -> TilburgHandState:
        """
        Returns the current state of the hand.
        """
        return TilburgHandState(joint_positions=self.get_pos_vector())

    def close(self):
        """
        Closes the hand control interface.
        """
        self.disconnect()
        logger.info("Hand control interface closed.")

    def get_normalized_joint_poses(self) -> np.ndarray:
        return self.get_pos_vector()

    def set_normalized_joint_poses(self, values: np.ndarray):
        self.set_pos_vector(values)
