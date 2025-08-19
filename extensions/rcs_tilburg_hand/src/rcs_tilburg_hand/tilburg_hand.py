import copy
import logging
from dataclasses import dataclass
from time import sleep

import numpy as np
from tilburg_hand import Finger, TilburgHandMotorInterface, Unit

from rcs.envs.space_utils import Vec18Type
from rcs.hand.interface import BaseHand

# Setup logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
logger.disabled = False


@dataclass(kw_only=True)
class THConfig:
    """Config for the Tilburg hand"""

    calibration_file: str | None = None
    grasp_percentage: float = 1.0
    control_unit: Unit = Unit.NORMALIZED
    hand_orientation: str = "right"


class TilburgHand(BaseHand):
    """
    Tilburg Hand Class
    This class provides an interface for controlling the Tilburg Hand.
    It allows for grasping, resetting, and disconnecting from the hand.
    """

    MAX_GRASP_JOINTS_VALS = np.array(
        [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0]
    )

    def __init__(self, cfg: THConfig, verbose: bool = False):
        """
        Initializes the Tilburg Hand interface.
        """
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
        assert len(pos_vector) == len(
            self._motors.n_motors
        ), f"Invalid position vector length: {len(pos_vector)}. Expected: {len(self._motors.n_motors)}"
        self._motors.set_pos_vector(copy.deepcopy(pos_vector), unit=self._cfg.control_unit)
        logger.info(f"Set pose vector: {pos_vector}")

    def set_zero_pos(self):
        """
        Sets all finger joint positions to zero.
        """
        pos_normalized = 0 * self.MAX_GRASP_JOINTS_VALS
        self._motors.set_pos_vector(pos_normalized, unit=self._cfg.control_unit)
        logger.info("All joints reset to zero position.")

    def set_joint_pos(self, finger_joint: Finger, pos_value: float):
        """
        Sets a single joint to a specific normalized position.
        """
        self._motors.set_pos_single(finger_joint, pos_value, unit=self._cfg.control_unit)

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
        pos_normalized = self._cfg.grasp_percentage * self.MAX_GRASP_JOINTS_VALS
        self._motors.set_pos_vector(pos_normalized, unit=self._cfg.control_unit)
        logger.info(f"Grasp command sent with value: {self._cfg.grasp_percentage:.2f}")

    def auto_recovery(self):
        if not np.array(self._motors.check_enabled_motors()).all():
            logger.warning("Some motors are not enabled. Attempting to enable them.")
            self._motors.disconnect()
            sleep(1)
            re = self._motors.connect()
            assert re >= 0, "Failed to reconnect to the motors' board."

    #### BaseHandControl Interface methods ####

    def grasp(self):
        """
        Performs a grasp with a specified intensity (0.0 to 1.0).
        """
        self._grasp()

    def open(self):
        self.set_zero_pos()

    def reset(self):
        """
        Resets the hand to its initial state.
        """
        self.auto_recovery()
        self.set_zero_pos()
        logger.info("Hand reset to initial state.")

    def get_state(self) -> np.ndarray:
        """
        Returns the current state of the hand.
        """
        return self.get_pos_vector()

    def close(self):
        """
        Closes the hand control interface.
        """
        self.disconnect()
        logger.info("Hand control interface closed.")

    def get_normalized_joints_poses(self) -> np.ndarray:
        return self.get_pos_vector()

    def set_normalized_joints_poses(self, values: np.ndarray):
        self.set_pos_vector(values)
