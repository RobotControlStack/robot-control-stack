from pathlib import Path
import typing

import numpy as np
from lerobot.robots import make_robot_from_config
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so101_follower.so101_follower import SO101Follower

from rcs import common, scenes
class SO101Config(common.RobotConfig):
    id: str = "follower"
    port: str = "/dev/ttyACM0"
    calibration_dir: str = "."


class SO101(common.Robot):
    # def __init__(self, hf_robot: SO101Follower):
    def __init__(self, robot_cfg: SO101Config, ik: common.Kinematics):
        super().__init__()
        self.ik = ik
        cfg = SO101FollowerConfig(id=robot_cfg.id, calibration_dir=Path(robot_cfg.calibration_dir), port=robot_cfg.port)
        self._hf_robot = make_robot_from_config(cfg)
        self._hf_robot.connect()
        self._robot_config = robot_cfg
        # ik = rcs_robotics_library._core.rl.RoboticsLibraryIK(robot_cfg.kinematic_model_path)

    # def get_base_pose_in_world_coordinates(self) -> Pose: ...
    def get_cartesian_position(self) -> common.Pose:
        return self.ik.forward(self.get_joint_position())

    def get_ik(self) -> common.Kinematics | None:
        return self.ik

    def get_joint_position(self) -> np.ndarray[tuple[typing.Literal[5]], np.dtype[np.float64]]:  # type: ignore
        obs = self._hf_robot.get_observation()
        self.obs = obs
        return typing.cast(
            np.ndarray[tuple[typing.Literal[5]], np.dtype[np.float64]],
            np.array(
                [
                    obs["shoulder_pan.pos"],
                    obs["shoulder_lift.pos"],
                    obs["elbow_flex.pos"],
                    obs["wrist_flex.pos"],
                    obs["wrist_roll.pos"],
                ],
                dtype=np.float64,
            ),
        )

    def get_config(self):
        return self._robot_config

    def get_state(self) -> common.RobotState:
        return common.RobotState()

    def move_home(self) -> None:
        home = typing.cast(
            np.ndarray[tuple[typing.Literal[5]], np.dtype[np.float64]],
            common.robots_meta_config(common.RobotType.SO101).q_home,
        )
        self.set_joint_position(home)

    def reset(self) -> None:
        pass

    def set_cartesian_position(self, pose: common.Pose) -> None:
        joints = self.ik.inverse(pose, q0=self.get_joint_position())
        if joints is not None:
            self.set_joint_position(joints)

    def set_joint_position(self, q: np.ndarray[tuple[typing.Literal[5]], np.dtype[np.float64]]) -> None:  # type: ignore
        self._hf_robot.send_action(
            {
                "shoulder_pan.pos": q[0],
                "shoulder_lift.pos": q[1],
                "elbow_flex.pos": q[2],
                "wrist_flex.pos": q[3],
                "wrist_roll.pos": q[4],
            }
        )
        print(q)

    # def to_pose_in_robot_coordinates(self, pose_in_world_coordinates: Pose) -> Pose: ...
    # def to_pose_in_world_coordinates(self, pose_in_robot_coordinates: Pose) -> Pose: ...


# TODO: problem when we inherit from gripper then we also need to call init which doesnt exist
class SO101Gripper(common.Gripper):
    def __init__(self, hf_robot: SO101Follower, robot: SO101):
        super().__init__()
        self._hf_robot = hf_robot
        self._robot = robot

    def get_normalized_width(self) -> float:
        obs = self._robot.obs
        if obs is None:
            return 0.0
        return obs["gripper.pos"] / 100.0

    # def get_config(self) -> GripperConfig: ...
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

    def reset(self) -> None:
        pass

    def set_normalized_width(self, width: float, _: float = 0) -> None:
        """
        Set the gripper width to a normalized value between 0 and 1.
        """
        if not (0 <= width <= 1):
            msg = f"Width must be between 0 and 1, got {width}."
            raise ValueError(msg)
        # Convert normalized width to absolute position
        abs_width = width * 100.0
        self._hf_robot.send_action({"gripper.pos": abs_width})

    def shut(self) -> None:
        """
        Close the gripper.
        """
        self.set_normalized_width(0.0)
