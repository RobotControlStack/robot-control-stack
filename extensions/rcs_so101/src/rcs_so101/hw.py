import threading
import typing
from pathlib import Path

import numpy as np
from attr import dataclass
from lerobot.robots import make_robot_from_config  # noqa: F401
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from rcs.utils import SimpleFrameRate

from rcs import common


@dataclass(kw_only=True)
class SO101Config(common.RobotConfig):
    id: str = "follower"
    port: str = "/dev/ttyACM0"
    calibration_dir: str = "."

    def __post_init__(self):
        super().__init__()


class SO101:
    def __init__(self, robot_cfg: SO101Config, ik: common.IK):
        self.ik = ik
        cfg = SO101FollowerConfig(id=robot_cfg.id, calibration_dir=Path(robot_cfg.calibration_dir), port=robot_cfg.port)
        self.hf_robot = make_robot_from_config(cfg)
        self.hf_robot.connect()
        self._thread = None
        self._running = False
        self._goal = None
        self._goal_lock = threading.Lock()
        self._rate_limiter = SimpleFrameRate(60, "teleop readout")

    def get_cartesian_position(self) -> common.Pose:
        return self.ik.forward(self._get_joint_position())

    def _get_cartesian_position(self) -> common.Pose:
        return self._last_cart

    def get_ik(self) -> common.IK | None:
        return self.ik

    def get_joint_position(self) -> np.ndarray[tuple[typing.Literal[5]], np.dtype[np.float64]]:  # type: ignore
        obs = self.hf_robot.get_observation()
        joints = np.array(
            [
                obs["shoulder_pan.pos"],
                obs["shoulder_lift.pos"],
                obs["elbow_flex.pos"],
                obs["wrist_flex.pos"],
                obs["wrist_roll.pos"],
            ],
            dtype=np.float64,
        )
        # print(obs)
        joints_normalized = (joints + 100) / 200
        joints_in_rad = (
            joints_normalized
            * (
                common.robots_meta_config(common.RobotType.SO101).joint_limits[1]
                - common.robots_meta_config(common.RobotType.SO101).joint_limits[0]
            )
            + common.robots_meta_config(common.RobotType.SO101).joint_limits[0]
        )
        return joints_in_rad

    def _get_joint_position(self) -> np.ndarray[tuple[typing.Literal[5]], np.dtype[np.float64]]:  # type: ignore
        return self._last_joint

    def get_parameters(self) -> SO101Config:
        a = SO101Config()
        a.robot_platform = common.RobotPlatform.HARDWARE
        a.robot_type = common.RobotType.SO101
        return a

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
        q0 = self.get_joint_position()
        joints = self.ik.ik(pose, q0=q0)
        if joints is not None:
            self.set_joint_position(joints)
            self._last_cart = pose

    def _set_joint_position(self, q: np.ndarray[tuple[typing.Literal[5]], np.dtype[np.float64]]) -> None:  # type: ignore
        self._last_joint = q
        q_normalized = (q - common.robots_meta_config(common.RobotType.SO101).joint_limits[0]) / (
            common.robots_meta_config(common.RobotType.SO101).joint_limits[1]
            - common.robots_meta_config(common.RobotType.SO101).joint_limits[0]
        )
        q_hf = (q_normalized * 200) - 100
        self.hf_robot.send_action(
            {
                "shoulder_pan.pos": q_hf[0],
                "shoulder_lift.pos": q_hf[1],
                "elbow_flex.pos": q_hf[2],
                "wrist_flex.pos": q_hf[3],
                "wrist_roll.pos": q_hf[4],
            }
        )

    def set_joint_position(self, q: np.ndarray[tuple[typing.Literal[5]], np.dtype[np.float64]]) -> None:  # type: ignore
        if not self._running:
            self.start_controller_thread()
        with self._goal_lock:
            self._goal = q

    def _controller(self):
        if self._thread is not None and self._thread.is_alive():
            return
        while self._running:
            self._rate_limiter()
            with self._goal_lock:
                goal = self._goal
            if goal is None:
                continue
            current_pos = self.get_joint_position()
            if np.allclose(current_pos, goal, atol=np.deg2rad(1)):
                continue
            # interpolate with max 10 degree / s
            max_step = np.deg2rad(10) * self._rate_limiter.get_frame_time()
            delta = goal - current_pos
            if np.max(delta) > max_step:
                delta = delta / np.max(delta) * max_step
            self._set_joint_position(current_pos + delta)

    def start_controller_thread(self):
        self._running = True
        self._thread = threading.Thread(target=self._controller, daemon=True)
        self._thread.start()

    def stop_controller_thread(self):
        self._running = False
        with self._goal_lock:
            self._goal = None
        if self._thread is not None and self._thread.is_alive():
            self._thread.join()

    # def to_pose_in_robot_coordinates(self, pose_in_world_coordinates: Pose) -> Pose: ...
    # def to_pose_in_world_coordinates(self, pose_in_robot_coordinates: Pose) -> Pose: ...


# TODO: problem when we inherit from gripper then we also need to call init which doesnt exist
class SO101Gripper(common.Gripper):
    def __init__(self, hf_robot: SO101Follower):
        self.hf_robot = hf_robot

    def get_normalized_width(self) -> float:
        obs = self.hf_robot.get_observation()
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
        self.hf_robot.send_action({"gripper.pos": abs_width})

    def shut(self) -> None:
        """
        Close the gripper.
        """
        self.set_normalized_width(0.0)
