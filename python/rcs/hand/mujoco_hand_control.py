import time
from typing import Optional
import os
from pathlib import Path

import mujoco
import mujoco.viewer
from rcs.hand.interface import BaseHand
from pydantic import BaseModel

class THMujocoConfig(BaseModel):
    """Config for the Mujoco Tilburg hand"""

    binary_action: bool = True
    mujoco_xml_path: str = os.path.join(Path.home(), "repos/tilburg-hand/src/tilburg_hand_urdf_mujoco/robot.xml")


class MujocoHandControl(BaseHand):
    """
    Tilburg Mujoco Hand Control
    This class provides an interface for controlling the Tilburg Hand in Mujoco.
    It allows for grasping, resetting, and disconnecting from the hand.
    """

    def __init__(self, mujoco_hand_cfg: THMujocoConfig):

        # Load model
        xml_path = mujoco_hand_cfg.mujoco_xml_path
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.actuator_names = [self.model.actuator(i).name for i in range(self.model.nu)]
        self.pos_vector_names_list = [
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
            "WRIST_PITCH",
            "WRIST_YAW",  # wrist joints are only in tilburg hand and not used
            # in either mujoco or tilburg hand
        ]
        # Simulation parameters
        self.running = True
        self.control_duration = 0.05  # seconds for control actions

    def _run_simulation(self, duration):
        """Helper method to run simulation for a specified duration"""
        start_time = time.time()
        while time.time() - start_time < duration and self.running:
            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()
            # time.sleep(0.001)  # prevent busy waiting

    def grasp(self, value: Optional[float] = None):
        """
        Grasp the hand by setting the actuator values.
        :param value: The value to set for the actuators.
        """
        if value is None:
            value = 1.5
        # Define the power grasp configuration
        power_grasp = {
            "thumb_cmc": value,
            "thumb_mcp": value,
            "thumb_ip": value,
            "index_mcp": value,
            "index_pip": value,
            "index_dip": value,
            "middle_mcp": value,
            "middle_pip": value,
            "middle_dip": value,
            "ring_mcp": value,
            "ring_pip": value,
            "ring_dip": value,
        }

        # Apply targets
        for _, name in enumerate(power_grasp.keys()):
            # Check if the actuator name is in the model
            actuator_index = self.actuator_names.index(name)
            if name in power_grasp:
                self.data.ctrl[actuator_index] = power_grasp[str(name)]
        # mujoco.mj_step(self.model, self.data)
        # self.viewer.sync()
        self._run_simulation(self.control_duration)

    def set_zero_pos(self):
        """Set all joints to zero position"""
        # Set all actuator values to zero
        self.data.ctrl[:] = 0.0
        # mujoco.mj_step(self.model, self.data)
        # self.viewer.sync()
        self._run_simulation(self.control_duration)

    def disconnect(self):
        """Clean up and close"""
        self.running = False
        self.viewer.close()

    def get_pos_vector(self):
        """Get the current position vector of the actuators"""
        # Get the current actuator values
        # Ensure the values fit the model's actuator count
        pos_vec = self.data.ctrl.tolist()
        pos_vec.append(0.0)
        pos_vec.append(0.0)
        return pos_vec

    def set_pos_vector(self, values: list):
        """
        Set the position vector for the actuators.
        :param values: List of actuator values to set.
        """
        # Ensure the values fit the model's actuator count
        values = values[: int(self.model.nu)]  # Ensure the values fit the model's actuator count
        for i, name in enumerate(self.pos_vector_names_list[: int(self.model.nu)]):
            # Check if the actuator name is in the model
            actuator_index = self.actuator_names.index(name)
            # if name in values:
            self.data.ctrl[actuator_index] = values[i]
        # mujoco.mj_step(self.model, self.data)
        # self.viewer.sync()
        self._run_simulation(self.control_duration)

    def reset(self):
        """
        Reset the hand to its initial state.
        """
        self.set_zero_pos()

    def get_state(self):
        """
        Returns the current state of the hand.
        """
        return self.get_pos_vector()

    def get_normalized_joints_poses(self):
        """
        Returns the current position vector.
        """
        self.get_pos_vector()

    def set_normalized_joints_poses(self, values: list):
        """
        Set the position vector for the actuators.
        :param values: List of actuator values to set.
        """
        self.set_pos_vector(values)

    def open(self):
        """
        Open the hand.
        """
        self.reset()

    def __del__(self):
        """
        Destructor to clean up resources.
        """
        self.disconnect()