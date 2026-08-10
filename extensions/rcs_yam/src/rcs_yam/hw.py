"""Hardware abstraction layer for the I2RT YAM arm.

The i2rt driver exposes the arm and the gripper as a single motor chain with the gripper motor
as its last entry, and runs its PD control loop in a background thread. `Yam` owns that handle
and the shared target vector, `YamGripper` writes the gripper entry of the same target through it.
"""

import threading
import time
import typing

import numpy as np
from rcs.common_typing import RobotConfigKwargs

from rcs import common


class YamConfig(common.RobotConfig):
    """Configuration of a single YAM arm on one CAN bus."""

    def __init__(
        self,
        channel: str = "can0",
        arm_type_id: str = "yam",
        gripper_type_id: str = "linear_4310",
        arm_version: int = 1,
        async_control: bool = True,
        joint_tolerance: float = 0.01,
        command_timeout: float = 5.0,
        gripper_tolerance: float = 0.02,
        gripper_timeout: float = 2.0,
        max_joint_step: float = 0.1,
        max_joint_velocity: float = 0.5,
        move_home_duration: float = 2.0,
        gripper_limits_override: np.ndarray | None = None,
        **kwargs: typing.Unpack[RobotConfigKwargs],
    ):
        super().__init__(**kwargs)
        self.robot_platform = common.RobotPlatform.HARDWARE
        self.robot_type = common.RobotType("Yam")
        self.channel = channel
        self.arm_type_id = arm_type_id
        self.gripper_type_id = gripper_type_id
        self.arm_version = arm_version
        # If False, commands return once the target is reached or the timeout hits.
        self.async_control = async_control
        self.joint_tolerance = joint_tolerance
        self.command_timeout = command_timeout
        # A blocked gripper never reaches its target, so it gets a shorter timeout of its own.
        self.gripper_tolerance = gripper_tolerance
        self.gripper_timeout = gripper_timeout
        # Targets further away than max_joint_step are ramped instead of sent as a step, the arm
        # is direct driven and follows a plain PD controller without trajectory generation.
        self.max_joint_step = max_joint_step
        self.max_joint_velocity = max_joint_velocity
        self.move_home_duration = move_home_duration
        # If set, skips the calibration run that would otherwise drive the fingers to both stops.
        self.gripper_limits_override = gripper_limits_override


class Yam(common.Robot):
    """RCS robot backed by an i2rt motor chain."""

    POLL_INTERVAL = 0.005

    def __init__(self, cfg: YamConfig, ik: common.Kinematics):
        super().__init__()
        from i2rt.robots.get_robot import get_yam_robot
        from i2rt.robots.utils import ArmType, GripperType

        self._closed = True
        self.ik = ik
        self._config = cfg
        self._dof = int(cfg.dof)
        self._robot = get_yam_robot(
            channel=cfg.channel,
            arm_type=ArmType.from_string_name(cfg.arm_type_id),
            gripper_type=GripperType.from_string_name(cfg.gripper_type_id),
            gripper_limits_override=cfg.gripper_limits_override,
        )
        self._closed = False
        self._has_gripper = self._robot.num_dofs() > self._dof
        self._lock = threading.Lock()
        # Seeding the target from the measured state avoids a jump on the first partial command.
        self._target = np.asarray(self._robot.get_joint_pos(), dtype=np.float64).copy()
        print(f"YAM connected on {cfg.channel} with {self._robot.num_dofs()} motors.")

    def __del__(self):
        self.close()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._robot.close()

    def get_config(self) -> YamConfig:
        return self._config

    def set_config(self, robot_cfg: YamConfig) -> None:
        self._config = robot_cfg

    def get_state(self) -> common.RobotState:
        return common.RobotState()

    def get_ik(self) -> common.Kinematics | None:
        return self.ik

    def reset(self) -> None:
        pass

    def automatic_error_recovery(self) -> None:
        """Called by `RobotWrapper.reset` when homing raises. The i2rt driver has no recovery hook."""

    def get_joint_position(self) -> np.ndarray[tuple[typing.Any], np.dtype[np.float64]]:
        return np.asarray(self._robot.get_joint_pos(), dtype=np.float64)[: self._dof]

    def get_gripper_width(self) -> float:
        """Normalized gripper width, 0 is closed and 1 is open, as mapped by the i2rt joint mapper."""
        self._assert_gripper()
        return float(np.asarray(self._robot.get_joint_pos(), dtype=np.float64)[self._dof])

    def get_cartesian_position(self) -> common.Pose:
        # `Kinematics.forward` applies the inverse of the offset it is handed, so the TCP is composed
        # here instead, to match the pose `SimRobot::get_cartesian_position` reports in simulation.
        return self.get_cartesian_flange_position() * self._config.tcp_offset

    def get_cartesian_flange_position(self) -> common.Pose:
        return self.ik.forward(self.get_joint_position(), common.Pose())

    def set_cartesian_position(self, pose: common.Pose) -> None:
        q = self.ik.inverse(pose, self.get_joint_position(), self._config.tcp_offset)
        if q is None:
            print("IK failed")
            return
        # The kinematic model carries the two finger joints, only the arm joints are commanded.
        self.set_joint_position(np.asarray(q, dtype=np.float64)[: self._dof])

    def set_joint_position(self, q: np.ndarray[tuple[typing.Any], np.dtype[np.float64]]) -> None:
        self._command(arm=np.asarray(q, dtype=np.float64))

    def set_gripper_width(self, width: float) -> None:
        self._assert_gripper()
        self._command(gripper=float(np.clip(width, 0.0, 1.0)))

    def move_home(self) -> None:
        if self._config.q_home is None:
            msg = "No home position configured."
            raise ValueError(msg)
        home = np.asarray(self._config.q_home, dtype=np.float64)
        low, high = self._config.joint_limits
        if np.any((home < low) | (home > high)):
            msg = f"Home position {home} is out of joint limits."
            raise ValueError(msg)
        print(f"Moving to home position: {home}")
        with self._lock:
            self._target[: self._dof] = home
            target = self._target.copy()
        self._robot.move_joints(target, time_interval_s=self._config.move_home_duration)

    def _assert_gripper(self) -> None:
        if not self._has_gripper:
            msg = f"YAM on {self._config.channel} was created without a gripper."
            raise RuntimeError(msg)

    def _command(self, arm: np.ndarray | None = None, gripper: float | None = None) -> None:
        """Write the arm and/or gripper part of the shared target and send it to the motor chain."""
        with self._lock:
            if arm is not None:
                self._target[: self._dof] = arm
            if gripper is not None:
                self._target[self._dof] = gripper
            target = self._target.copy()

        if self._config.async_control:
            # Streaming mode: always forward the latest goal and return immediately. A blocking ramp
            # here would stall the caller while new targets keep arriving, which reads as queued lag.
            self._robot.command_joint_pos(target)
            return

        current = np.asarray(self._robot.get_joint_pos(), dtype=np.float64)
        arm_error = float(np.max(np.abs(target[: self._dof] - current[: self._dof])))
        if arm_error > self._config.max_joint_step:
            # Ramping blocks, a step of this size would jerk the arm.
            self._robot.move_joints(target, time_interval_s=arm_error / self._config.max_joint_velocity)
            return

        self._robot.command_joint_pos(target)
        self._wait_until_reached(target, wait_arm=arm is not None, wait_gripper=gripper is not None)

    def _wait_until_reached(self, target: np.ndarray, wait_arm: bool, wait_gripper: bool) -> None:
        timeout = self._config.command_timeout if wait_arm else self._config.gripper_timeout
        deadline = time.time() + timeout
        while time.time() < deadline:
            current = np.asarray(self._robot.get_joint_pos(), dtype=np.float64)
            arm_reached = np.max(np.abs(target[: self._dof] - current[: self._dof])) < self._config.joint_tolerance
            gripper_reached = (
                not self._has_gripper or abs(target[self._dof] - current[self._dof]) < self._config.gripper_tolerance
            )
            if (arm_reached or not wait_arm) and (gripper_reached or not wait_gripper):
                return
            time.sleep(self.POLL_INTERVAL)


class YamGripper(common.Gripper):
    """RCS gripper sharing the motor chain of a `Yam` instance."""

    def __init__(self, cfg: common.GripperConfig, robot: Yam):
        super().__init__()
        self._cfg = cfg
        self._robot = robot

    def get_config(self) -> common.GripperConfig:
        return self._cfg

    def get_normalized_width(self) -> float:
        return self._robot.get_gripper_width()

    def set_normalized_width(self, width: float, force: float = 0) -> None:
        if not (0 <= width <= 1):
            msg = f"Width must be between 0 and 1, got {width}."
            raise ValueError(msg)
        self._robot.set_gripper_width(width)

    def open(self) -> None:
        self.set_normalized_width(1.0)

    def grasp(self) -> None:
        self.set_normalized_width(0.0)

    def shut(self) -> None:
        self.set_normalized_width(0.0)

    def reset(self) -> None:
        self.open()

    def close(self) -> None:
        """The motor chain belongs to the robot, which closes it."""
