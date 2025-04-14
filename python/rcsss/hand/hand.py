import sys
import os
import logging
from time import sleep
import copy
from tilburg_hand import TilburgHandMotorInterface, Finger, Wrist, Unit

# Setup logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class TilburgHandControl:
    def __init__(self, config_folder_path="config", verbose=False):
        """
        Initializes the Tilburg Hand Control interface.
        """
        self.config_folder_path = config_folder_path
        self.tilburg_hand_config_file_path = os.path.join(config_folder_path, "config.json")
        self.tilburg_hand_calibration_file_path = os.path.join(config_folder_path, "calibration.json")

        self._motors = TilburgHandMotorInterface(
            config_file=self.tilburg_hand_config_file_path,
            calibration_file=self.tilburg_hand_calibration_file_path,
            verbose=verbose
        )

        self.pos_value_unit = Unit.NORMALIZED
        self._pos_normalized = [0] * self._motors.n_motors
        if self._motors.connect() < 0:
            logger.error("Failed to connect to the motors' board.")
            sys.exit(1)

        logger.info("Connected to the motors' board.")

        self.fingers_joints = Finger
        self.fingers_joints_list = [
            Finger.INDEX_ABD, Finger.MIDDLE_ABD, Finger.THUMB_ABD, Finger.RING_ABD,
            Finger.INDEX_DIP, Finger.MIDDLE_DIP, Finger.RING_DIP, Finger.THUMB_IP,
            Finger.INDEX_PIP, Finger.MIDDLE_PIP, Finger.RING_PIP,
            Finger.INDEX_MCP, Finger.MIDDLE_MCP, Finger.RING_MCP, Finger.THUMB_MCP,
            Finger.THUMB_CMC
        ]
        self.pos_vector_names_list = [
                                        "THUMB_IP", "THUMB_MCP", "THUMB_ABD", "THUMB_CMC",
                                        "INDEX_DIP", "INDEX_PIP", "INDEX_MCP", "INDEX_ABD",
                                        "MIDDLE_DIP", "MIDDLE_PIP", "MIDDLE_MCP", "MIDDLE_ABD",
                                        "RING_DIP", "RING_PIP", "RING_MCP", "RING_ABD", 
                                        "WRIST_PITCH", "WRIST_YAW"
                                    ]

        self.wrist_joints_list = [Wrist.PITCH, Wrist.YAW]
        self.set_zero_pos()
        logger.info("Setting all joints to zero position.")

    def grasp(self, value: float, template: list = None):
        """
        Performs a grasp with a specified intensity (0.0 to 1.0).
        """
        if template is None:
            template = [
                0.0, 0.0, 1.0, 0.0,
                1.0, 1.0, 1.0, 1.0,
                1.0, 1.0, 1.0,
                1.0, 1.0, 1.0, 1.0,
                1.0
            ]

        if len(template) != len(self.fingers_joints_list):
            logger.warning("Grasp template length mismatch. Aborting grasp.")
            return

        fingers_joints_pos_values_list = [
            val * value for val in template
        ]

        for joint, pos in zip(self.fingers_joints_list, fingers_joints_pos_values_list):
            self._pos_normalized[joint] = pos

        self._motors.set_pos_vector(copy.deepcopy(self._pos_normalized), unit=self.pos_value_unit)
        logger.info(f"Grasp command sent with value: {value}")

    def set_zero_pos(self):
        """
        Sets all finger joint positions to zero.
        """
        for joint in self.fingers_joints_list:
            self._pos_normalized[joint] = 0
        self._motors.set_pos_vector(copy.deepcopy(self._pos_normalized), unit=self.pos_value_unit)
        logger.info("All joints reset to zero position.")

    def sleep(self, seconds: float):
        """
        Delays the program for a specified number of seconds.
        """
        sleep(seconds)

    def set_joint_pos(self, finger_joint, pos_value: float):
        """
        Sets a single joint to a specific normalized position.
        """
        self._motors.set_pos_single(finger_joint, pos_value, unit=self.pos_value_unit)
        self._pos_normalized[finger_joint] = pos_value
        logger.info(f"Set joint {finger_joint.name} to {pos_value:.2f}")

    def reset_joint_pos(self, finger_joint):
        """
        Resets a specific joint to zero.
        """
        self._motors.set_pos_single(finger_joint, 0, unit=self.pos_value_unit)
        self._pos_normalized[finger_joint] = 0
        logger.info(f"Reset joint {finger_joint.name} to 0")

    def disconnect(self):
        """
        Gracefully disconnects from the motor interface.
        """
        self._motors.disconnect()
        logger.info("Disconnected from the motors' board")

    def get_pos_vector(self):
        """
        Returns the current position vector of the motors.
        """
        return self._pos_normalized
    
    def get_pos_single(self, finger_joint):
        """
        Returns the current position of a single joint.
        """
        return self._pos_normalized[finger_joint]
    
class Hand:
    """
    Dummy class to represent the hand. In a real implementation, this would
    interface with the actual hand hardware.
    """
    def __init__(self, hand: TilburgHandControl):
        self._hand = hand

    def grasp(self):
        self._hand.grasp(value = 0.9)

    def reset(self):
        self._hand.set_zero_pos()

    def get_state(self):
        pass
    
    def disconnect(self):
        self._hand.disconnect()

    def get_normalized_width(self):
        pass

    def open(self):
        self.reset()

    def __del__(self):
        self._hand.disconnect()
        logger.info("Hand object deleted and disconnected from motors' board.")

if __name__ == "__main__":
    # try:
    #     hand = TilburgHandControl(verbose=True)
    #     hand.grasp(0.9)
    #     logger.info(f"current joints poses: {hand.get_pos_vector()}")
    #     logger.info(hand.pos_vector_names_list)
    #     hand.sleep(5)
    #     hand.set_zero_pos()
    #     hand.sleep(1)
    #     hand.set_joint_pos(hand.fingers_joints.THUMB_CMC, 0.5)
    #     hand.sleep(2)
    #     logger.info(f"current joint {hand.fingers_joints.THUMB_CMC.name}: {hand.get_pos_single(hand.fingers_joints.THUMB_CMC)}")
    #     hand.reset_joint_pos(hand.fingers_joints.THUMB_CMC)
    # except Exception as e:
    #     logger.exception(f"An error occurred: {e}")
    # finally:
    #     hand.disconnect()
    hand = Hand(TilburgHandControl(verbose=True))
    hand.reset()
    hand.grasp()
    sleep(5)
    hand.reset()
    sleep(2)
    hand.disconnect()