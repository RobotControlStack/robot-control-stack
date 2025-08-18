from dataclasses import dataclass
import typing

import numpy as np

from rcs import common

import rtde_control
import rtde_receive
from . import robotiq_gripper

@dataclass(kw_only=True)
class UR5eConfig(common.RobotConfig):
    lookahead_time: float = 0.05
    gain: float = 300.0
    max_velocity: float = 1.0
    max_acceleration: float = 1.0

    def __post_init__(self):
        super().__init__()


@dataclass(kw_only=True)
class UR5eState(common.RobotState):
    some_custom_state: float = 0.0 #TODO(j.hechtl): what to put here)
    def __post_init__(self):
        super().__init__()


class UR5e: #(common.Robot): # should inherit and implement common.Robot, but currently there is a bug that needs to be fixed
    def __init__(self, ip: str, urdf_path: str):
        # self.ik = common.RL(urdf_path=urdf_path)
        self._config = UR5eConfig() # with default values
        self._config.robot_platform = common.RobotPlatform.HARDWARE

        # TODO(j.hechtl): this is currently blocking if connection fails
        self.ur_control = rtde_control.RTDEControlInterface(ip)
        self.ur_receive = rtde_receive.RTDEReceiveInterface(ip)

        self._config.robot_type = common.RobotType.UR5e

        if not self.ur_control.isConnected():
            raise ConnectionError(f"Could not connect to UR5e at {ip}. Please check the IP address and connection.")


    def get_cartesian_position(self) -> common.Pose:
        ur_pose = self.ur_receive.getActualTCPPose()

        rpy = common.RPY(roll=ur_pose[3], pitch=ur_pose[4], yaw=ur_pose[5])
        trans = [ur_pose[0], ur_pose[1], ur_pose[2]]
        pose = common.Pose(rpy, trans)

        return pose

    def get_ik(self) -> common.IK | None:
        return self.ik

    def get_joint_position(self) -> np.ndarray[tuple[typing.Literal[6]], np.dtype[np.float64]]:  # type: ignore
        return np.array(self.ur_receive.getActualQ())

    def get_parameters(self) -> UR5eConfig:
        return self._config

    def set_parameters(self, robot_cfg: UR5eConfig) -> None:
        self._config = robot_cfg

    def get_state(self) -> UR5eState:
        return UR5eState

    def move_home(self) -> None:
        # TODO: This function is currently blocking. Is that ok?
        home = typing.cast(
            np.ndarray[tuple[typing.Literal[6]], np.dtype[np.float64]],
            common.robots_meta_config(common.RobotType.UR5e).q_home,
        )
        # check if home position is between -2*pi and 2*pi
        if np.any((home < -2*np.pi) | (home > 2*np.pi)):
            print(f"Home position {home} is out of bounds.")
        else:
            print(f"Moving to home position: {home}")
            self.ur_control.moveJ(home, self._config.max_velocity, self._config.max_acceleration, False)

    def reset(self) -> None:
        self.ur_control.stopL()

    def set_cartesian_position(self, pose: common.Pose) -> None:
        target_pose = pose.xyzrpy()
        self.ur_control.servoL(target_pose, self._config.lookahead_time, self._config.gain)

    def set_joint_position(self, q: np.ndarray[tuple[typing.Literal[6]], np.dtype[np.float64]]) -> None:  # type: ignore
        # TODO: add safety checks
        self.ur_control.moveJ(q.tolist(), self._config.max_velocity, self._config.max_acceleration, False)



class RobotiQGripper: # (common.Gripper):
    def __init__(self,ip):
        self.gripper = robotiq_gripper.RobotiqGripper()
        try:
            self.gripper.connect(ip, 63352, socket_timeout=3.0) # default port for Robotiq gripper
        except Exception as e:
            raise RuntimeError(f"Failed to connect to Robotiq gripper at {ip}: {e}")
        self.gripper.activate()
        if not self.gripper.is_active():
            raise RuntimeError("Failed to activate Robotiq gripper.")

    def get_normalized_width(self) -> float:
        # value between 0 and 1 (0 is closed)
        return (self.gripper.get_max_position() - self.gripper.get_current_position()) / self.gripper.get_max_position()

    def grasp(self) -> None:
        """
        Close the gripper to grasp an object.
        """
        self.set_normalized_width(0.0)

    # def is_grasped(self) -> bool: ...

    def open(self) -> None:
        """
        Open the gripper to its maximum width.
        """
        self.set_normalized_width(1.0)

    def reset(self) -> None:
        pass

    def set_normalized_width(self, width: float, _: float = 0) -> None:
        """
        Set the gripper width to a normalized value between 0 and 1.
        """
        if not (0 <= width <= 1):
            msg = f"Width must be between 0 and 1, got {width}."
            raise ValueError(msg)
        abs_width = (1-width) * self.gripper.get_max_position()
        #print(f"Setting gripper width to {width:.2f} (absolute: {abs_width:.2f})")
        self.gripper.move(int(abs_width), int(self.gripper._max_speed), int(self.gripper._max_force))

    def shut(self) -> None:
        """
        Close the gripper.
        """
        self.set_normalized_width(0.0)
