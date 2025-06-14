
import typing
import numpy as np
from rcs._core import common
from lerobot.common.robots.so101_follower.so101_follower import SO101Follower
from rcs._core.common import Gripper, Robot

class SO101(Robot):
    def __init__(self, hf_robot: SO101Follower):
        self._hf_robot = hf_robot

    # def get_base_pose_in_world_coordinates(self) -> Pose: ...
    # def get_cartesian_position(self) -> Pose: ...
    # def get_ik(self) -> IK | None: ...
    def get_joint_position(self) -> np.ndarray[typing.Literal[5], np.dtype[np.float64]]:
        obs = self._hf_robot.get_observation()
        # (Pdb) robot.get_observation()
        # {'shoulder_pan.pos': -9.40612320177057, 'shoulder_lift.pos': -99.66130397967824, 'elbow_flex.pos': 99.9124726477024, 'wrist_flex.pos': 69.96996996996998, 'wrist_roll.pos': -9.095744680851055, 'gripper.pos': 1.023192360163711}
        return np.array(
            [
                obs["shoulder_pan.pos"],
                obs["shoulder_lift.pos"],
                obs["elbow_flex.pos"],
                obs["wrist_flex.pos"],
                obs["wrist_roll.pos"],
                obs["gripper.pos"],
            ],
            dtype=np.float64,
        )

    def get_parameters(self):
        return self._hf_robot.calibration

    # def get_state(self) -> RobotState: ...
    def move_home(self) -> None:
        home = common.robots_meta_config(common.RobotType.SO101).q_home
        self.set_joint_position(home)
    # def reset(self) -> None: ...
    # def set_cartesian_position(self, pose: Pose) -> None: ...
    def set_joint_position(self, q: np.ndarray[typing.Literal[5], np.dtype[np.float64]]) -> None:
        self._hf_robot.send_action({
            "shoulder_pan.pos": q[0],
            "shoulder_lift.pos": q[1],
            "elbow_flex.pos": q[2],
            "wrist_flex.pos": q[3],
            "wrist_roll.pos": q[4],
        })
    # def to_pose_in_robot_coordinates(self, pose_in_world_coordinates: Pose) -> Pose: ...
    # def to_pose_in_world_coordinates(self, pose_in_robot_coordinates: Pose) -> Pose: ...


class S0101Gripper(Gripper):
    def __init__(self, hf_robot: SO101Follower):
        self._hf_robot = hf_robot

    def get_normalized_width(self) -> float:
        obs = self._hf_robot.get_observation()
        # (Pdb) robot.get_observation()
        # {'shoulder_pan.pos': -9.40612320177057, 'shoulder_lift.pos': -99.66130397967824, 'elbow_flex.pos': 99.9124726477024, 'wrist_flex.pos': 69.96996996996998, 'wrist_roll.pos': -9.095744680851055, 'gripper.pos': 1.023192360163711}
        return obs["gripper.pos"] / 100.0

    # def get_parameters(self) -> GripperConfig: ...
    # def get_state(self) -> GripperState: ...

    def grasp(self) -> None:
        """
        Close the gripper to grasp an object.
        """
        self.shut()
    # def is_grasped(self) -> bool: ...

    def open(self) -> None:
        """
        Open the gripper to its maximum width.
        """
        self.set_normalized_width(1.0)

    # def reset(self) -> None: ...
    def set_normalized_width(self, width: float, force: float = 0) -> None:
        """
        Set the gripper width to a normalized value between 0 and 1.
        """
        if not (0 <= width <= 1):
            raise ValueError("Width must be between 0 and 1.")
        # Convert normalized width to absolute position
        abs_width = width * 100.0
        self._hf_robot.send_action({"gripper.pos": abs_width})
    def shut(self) -> None:
        """
        Close the gripper.
        """
        self.set_normalized_width(0.0)