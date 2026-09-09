"""Hardware abstraction layer for Flexiv robots and grippers, built on the Flexiv RDK Python bindings.

The arm is driven through the RDK's non-real-time (NRT) control modes, in which the robot's internal
motion generator smoothens the discrete targets RCS sends. Sending a new target while the previous
motion is ongoing re-plans online, which is what makes the async control mode work. The controller
gains of the underlying impedance controllers are exposed through `FlexivConfig`.

The gripper (e.g. the Grav GN-01) is controlled through the RDK gripper device interface of the same
robot connection, so `FlexivGripper` is created from a `Flexiv` instance.
"""

import enum
import logging
import time
import typing

import numpy as np
from rcs import common
from rcs.common_typing import GripperConfigKwargs, RobotConfigKwargs

logger = logging.getLogger(__name__)


class FlexivControlMode(enum.Enum):
    """Which RDK controller tracks the targets RCS sends."""

    JOINT_POSITION = "joint_position"
    """NRT_JOINT_POSITION: stiff joint position control, no gains to tune."""
    JOINT_IMPEDANCE = "joint_impedance"
    """NRT_JOINT_IMPEDANCE: joint impedance control, stiffness and damping ratio per joint are configurable."""
    CARTESIAN_IMPEDANCE = "cartesian_impedance"
    """NRT_CARTESIAN_MOTION_FORCE: Cartesian impedance control with the robot's own IK, stiffness and damping
    ratio per Cartesian axis are configurable. Joint targets temporarily switch to a joint mode."""


class FlexivConfig(common.RobotConfig):
    """Configuration of a single Flexiv arm."""

    def __init__(
        self,
        robot_sn: str,
        control_mode: FlexivControlMode = FlexivControlMode.JOINT_IMPEDANCE,
        async_control: bool = True,
        joint_group: str | None = None,
        tool_name: str | None = None,
        joint_stiffness: np.ndarray | None = None,
        joint_damping_ratio: np.ndarray | None = None,
        cartesian_stiffness: np.ndarray | None = None,
        cartesian_damping_ratio: np.ndarray | None = None,
        max_contact_wrench: np.ndarray | None = None,
        null_space_posture: np.ndarray | None = None,
        max_joint_velocity: float = 1.0,
        max_joint_acceleration: float = 2.0,
        move_home_velocity: float = 0.5,
        max_linear_velocity: float = 0.5,
        max_angular_velocity: float = 1.0,
        max_linear_acceleration: float = 2.0,
        max_angular_acceleration: float = 5.0,
        joint_tolerance: float = 0.01,
        cartesian_tolerance: tuple[float, float] = (0.002, 0.02),
        command_timeout: float = 5.0,
        startup_timeout: float = 30.0,
        verbose: bool = False,
        **kwargs: typing.Unpack[RobotConfigKwargs],
    ):
        """
        Args:
            robot_sn: Serial number of the robot, e.g. "Rizon4s-123456". Same format as accepted by `flexivrdk.Robot`.
            control_mode: RDK controller used to track targets, see `FlexivControlMode`.
            async_control: If True, the setters return once the target was handed to the robot. If False, they poll
                the measured state until the target is reached within tolerance or `command_timeout` hits.
            joint_group: Name of the RDK joint group to control, e.g. "ARM_1". None picks the first single-arm
                group, which is the only one on single-arm robots like the Rizon.
            tool_name: Name of the tool (Flexiv Elements -> Settings -> Tool) to activate on the robot. The active
                tool determines the gravity compensation and the TCP the robot's Cartesian controller uses. None keeps
                the currently active tool.
            joint_stiffness: K_q in Nm/rad for the joint impedance mode, one value per joint in
                [0, nominal]. None uses the robot's nominal stiffness, 0 makes a joint free-floating.
            joint_damping_ratio: Z_q for the joint impedance mode, one value per joint in [0.3, 0.8]. None uses the
                RDK default of 0.7.
            cartesian_stiffness: K_x in N/m and Nm/rad for the Cartesian impedance mode,
                [k_x, k_y, k_z, k_Rx, k_Ry, k_Rz] in [0, nominal]. None uses the robot's nominal stiffness.
            cartesian_damping_ratio: Z_x for the Cartesian impedance mode, six values in [0.3, 0.8]. None uses the RDK
                default of 0.7.
            max_contact_wrench: Maximum contact force and moment [f_x, f_y, f_z, m_x, m_y, m_z] the Cartesian
                impedance mode regulates its output to. None disables the regulation.
            null_space_posture: Reference joint positions for the null-space posture control of the Cartesian
                impedance mode. None keeps the RDK default, the joint positions when the mode was entered.
            max_joint_velocity: Joint velocity limit in rad/s of the internal trajectory generator for joint targets.
            max_joint_acceleration: Joint acceleration limit in rad/s^2 for joint targets.
            move_home_velocity: Joint velocity limit in rad/s used for `move_home`.
            max_linear_velocity: TCP velocity limit in m/s for Cartesian targets in the Cartesian impedance mode.
            max_angular_velocity: TCP angular velocity limit in rad/s for Cartesian targets.
            max_linear_acceleration: TCP acceleration limit in m/s^2 for Cartesian targets.
            max_angular_acceleration: TCP angular acceleration limit in rad/s^2 for Cartesian targets.
            joint_tolerance: Joint error in rad below which a synchronous joint command counts as reached.
            cartesian_tolerance: Translation error in m and rotation error in rad below which a synchronous Cartesian
                command counts as reached.
            command_timeout: Seconds after which a synchronous command returns even if the target was not reached,
                e.g. because the arm is blocked by a contact.
            startup_timeout: Seconds to wait for the robot to become operational after servo on.
            verbose: Enable the info and warning prints of the RDK.
        """
        super().__init__(**kwargs)
        self.robot_platform = common.RobotPlatform.HARDWARE
        self.robot_sn = robot_sn
        self.control_mode = control_mode
        self.async_control = async_control
        self.joint_group = joint_group
        self.tool_name = tool_name
        self.joint_stiffness = joint_stiffness
        self.joint_damping_ratio = joint_damping_ratio
        self.cartesian_stiffness = cartesian_stiffness
        self.cartesian_damping_ratio = cartesian_damping_ratio
        self.max_contact_wrench = max_contact_wrench
        self.null_space_posture = null_space_posture
        self.max_joint_velocity = max_joint_velocity
        self.max_joint_acceleration = max_joint_acceleration
        self.move_home_velocity = move_home_velocity
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.max_linear_acceleration = max_linear_acceleration
        self.max_angular_acceleration = max_angular_acceleration
        self.joint_tolerance = joint_tolerance
        self.cartesian_tolerance = cartesian_tolerance
        self.command_timeout = command_timeout
        self.startup_timeout = startup_timeout
        self.verbose = verbose


class FlexivRobotState(common.RobotState):
    """Snapshot of the RDK robot states that are useful on top of the RCS observations."""

    def __init__(self, states: typing.Any, mode: str, operational: bool, fault: bool) -> None:
        super().__init__()
        self.q = np.asarray(states.q, dtype=np.float64)
        self.dq = np.asarray(states.dq, dtype=np.float64)
        self.tau = np.asarray(states.tau, dtype=np.float64)
        self.tau_ext = np.asarray(states.tau_ext, dtype=np.float64)
        self.tcp_wrench = np.asarray(states.tcp_wrench, dtype=np.float64)
        """External wrench at the TCP in the world frame [f_x, f_y, f_z, m_x, m_y, m_z]."""
        self.flange_pose = np.asarray(states.flange_pose, dtype=np.float64)
        """[x, y, z, q_w, q_x, q_y, q_z] as reported by the RDK."""
        self.mode = mode
        self.operational = operational
        self.fault = fault


def _pose_from_rdk(pose7: typing.Sequence[float]) -> common.Pose:
    """RDK poses are [x, y, z, q_w, q_x, q_y, q_z], RCS quaternions are [x, y, z, w]."""
    p = np.asarray(pose7, dtype=np.float64)
    return common.Pose(translation=p[:3], quaternion=np.array([p[4], p[5], p[6], p[3]]))


def _pose_to_rdk(pose: common.Pose) -> list[float]:
    return [*pose.translation().tolist(), *pose.rotation_q_wxyz().tolist()]


class Flexiv(common.Robot):
    """RCS robot backed by `flexivrdk.Robot`."""

    POLL_INTERVAL = 0.005

    def __init__(self, cfg: FlexivConfig, ik: common.Kinematics):
        super().__init__()
        import flexivrdk

        self._rdk = flexivrdk
        self._closed = True
        self.ik = ik
        self._config = cfg
        self._dof = int(cfg.dof)
        self._rdk_modes = {
            FlexivControlMode.JOINT_POSITION: flexivrdk.Mode.NRT_JOINT_POSITION,
            FlexivControlMode.JOINT_IMPEDANCE: flexivrdk.Mode.NRT_JOINT_IMPEDANCE,
            FlexivControlMode.CARTESIAN_IMPEDANCE: flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE,
        }

        self._robot = flexivrdk.Robot(cfg.robot_sn, cfg.verbose)
        self._closed = False
        self._make_operational()
        self._info = self._robot.info()
        self._group = self._resolve_group(cfg.joint_group)
        group_dof = int(self._info.DoF[self._group])
        if group_dof != self._dof:
            msg = f"Joint group {self._group_name} has {group_dof} joints, but the config expects dof={self._dof}."
            raise ValueError(msg)

        self._tool = flexivrdk.Tool(self._robot)
        if cfg.tool_name is not None:
            self._tool.Switch(self._group, cfg.tool_name)
        self._tool_tcp = self._read_tool_tcp()

        self._ensure_mode(cfg.control_mode)
        logger.info(
            "Connected to %s (%s, RDK %s), controlling %s with tool %s in %s.",
            self._info.serial_num,
            flexivrdk.kProductModelNames[self._info.product_model],
            self._info.software_ver,
            self._group_name,
            self._tool.name(self._group),
            cfg.control_mode.value,
        )

    def __del__(self):
        self.close()

    # ------------------------------------------------------------------ RDK access for the gripper

    @property
    def rdk_robot(self) -> typing.Any:
        """The underlying `flexivrdk.Robot`, shared with the gripper interface."""
        return self._robot

    @property
    def joint_group(self) -> typing.Any:
        """The `flexivrdk.JointGroup` this instance controls."""
        return self._group

    # ------------------------------------------------------------------ common.Robot interface

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            # Stop() halts any ongoing motion and puts the robot into IDLE.
            self._robot.Stop()
        except Exception as e:
            logger.warning("Stopping the robot on close failed: %s", e)

    def get_config(self) -> FlexivConfig:
        return self._config

    def set_config(self, robot_cfg: FlexivConfig) -> None:
        """Replaces the config and re-applies the impedance settings if the robot is in the affected mode."""
        self._config = robot_cfg
        if robot_cfg.tool_name is not None and robot_cfg.tool_name != self._tool.name(self._group):
            self._tool.Switch(self._group, robot_cfg.tool_name)
            self._tool_tcp = self._read_tool_tcp()
        self._ensure_mode(robot_cfg.control_mode)

    def get_state(self) -> FlexivRobotState:
        return FlexivRobotState(
            self._states(), self._rdk.kModeNames[self._robot.mode()], self._robot.operational(), self._robot.fault()
        )

    def get_ik(self) -> common.Kinematics | None:
        return self.ik

    def reset(self) -> None:
        pass

    def automatic_error_recovery(self) -> None:
        """Called by `RobotWrapper.reset` when homing raises: clears faults, servos on and restores the mode."""
        self._make_operational()
        self._ensure_mode(self._config.control_mode)

    def get_joint_position(self) -> np.ndarray[tuple[typing.Any], np.dtype[np.float64]]:
        return np.asarray(self._states().q, dtype=np.float64)

    def get_cartesian_flange_position(self) -> common.Pose:
        # The RDK reports poses in the robot's world frame, which coincides with the base frame unless a
        # different world frame was configured in Flexiv Elements.
        return _pose_from_rdk(self._states().flange_pose)

    def get_cartesian_position(self) -> common.Pose:
        return self.get_cartesian_flange_position() * self._config.tcp_offset

    def set_joint_position(self, q: np.ndarray[tuple[typing.Any], np.dtype[np.float64]]) -> None:
        q = np.asarray(q, dtype=np.float64)
        if q.shape != (self._dof,):
            msg = f"Expected {self._dof} joint positions, got shape {q.shape}."
            raise ValueError(msg)
        self._send_joint_target(q, self._config.max_joint_velocity)
        if not self._config.async_control:
            self._wait_for_joints(q, self._config.command_timeout)

    def set_cartesian_position(self, pose: common.Pose) -> None:
        if self._config.control_mode != FlexivControlMode.CARTESIAN_IMPEDANCE:
            q = self.ik.inverse(pose, self.get_joint_position(), self._config.tcp_offset)
            if q is None:
                logger.warning("IK failed for %s", pose)
                return
            self.set_joint_position(np.asarray(q, dtype=np.float64))
            return

        self._ensure_mode(FlexivControlMode.CARTESIAN_IMPEDANCE)
        # The RDK tracks the pose of the active tool's TCP, RCS commands the pose of its own TCP offset.
        rdk_target = pose * self._config.tcp_offset.inverse() * self._tool_tcp
        cmd = self._rdk.NrtCartesianCmd(
            _pose_to_rdk(rdk_target),
            [0.0] * 6,
            [0.0] * 6,
            self._config.max_linear_velocity,
            self._config.max_angular_velocity,
            self._config.max_linear_acceleration,
            self._config.max_angular_acceleration,
        )
        self._robot.SendCartesianMotionForce({self._group: cmd})
        if not self._config.async_control:
            self._wait_for_pose(pose, self._config.command_timeout)

    def move_home(self) -> None:
        """Moves to `q_home` with the reduced `move_home_velocity`, or runs the RDK Home primitive if unset.

        Always blocks until the home pose is reached, also in async mode.
        """
        if self._config.q_home is None:
            logger.info("No q_home configured, running the RDK Home primitive.")
            self._robot.Home([self._group])
            return
        home = np.asarray(self._config.q_home, dtype=np.float64)
        low, high = self._config.joint_limits
        if np.any((home < low) | (home > high)):
            msg = f"Home position {home} is out of joint limits."
            raise ValueError(msg)
        logger.info("Moving to home position: %s", home)
        distance = float(np.max(np.abs(home - self.get_joint_position())))
        self._send_joint_target(home, self._config.move_home_velocity)
        # Generous timeout: the trajectory generator also has to accelerate and decelerate.
        self._wait_for_joints(home, distance / self._config.move_home_velocity + self._config.command_timeout)

    # ------------------------------------------------------------------ impedance settings

    @property
    def nominal_joint_stiffness(self) -> np.ndarray:
        """Nominal (maximum) joint stiffness K_q of the robot in Nm/rad."""
        return np.asarray(self._info.K_q_nom[self._group], dtype=np.float64)

    @property
    def nominal_cartesian_stiffness(self) -> np.ndarray:
        """Nominal (maximum) Cartesian stiffness K_x of the robot in N/m and Nm/rad."""
        return np.asarray(self._info.K_x_nom[self._group], dtype=np.float64)

    def set_joint_impedance(self, stiffness: np.ndarray | None, damping_ratio: np.ndarray | None = None) -> None:
        """Updates K_q and Z_q of the joint impedance controller, applied immediately if that mode is active.

        None restores the nominal stiffness or the default damping ratio.
        """
        self._config.joint_stiffness = None if stiffness is None else np.asarray(stiffness, dtype=np.float64)
        self._config.joint_damping_ratio = (
            None if damping_ratio is None else np.asarray(damping_ratio, dtype=np.float64)
        )
        if self._robot.mode() == self._rdk_modes[FlexivControlMode.JOINT_IMPEDANCE]:
            self._apply_joint_impedance()

    def set_cartesian_impedance(self, stiffness: np.ndarray | None, damping_ratio: np.ndarray | None = None) -> None:
        """Updates K_x and Z_x of the Cartesian impedance controller, applied immediately if that mode is active.

        None restores the nominal stiffness or the default damping ratio.
        """
        self._config.cartesian_stiffness = None if stiffness is None else np.asarray(stiffness, dtype=np.float64)
        self._config.cartesian_damping_ratio = (
            None if damping_ratio is None else np.asarray(damping_ratio, dtype=np.float64)
        )
        if self._robot.mode() == self._rdk_modes[FlexivControlMode.CARTESIAN_IMPEDANCE]:
            self._apply_cartesian_impedance()

    # ------------------------------------------------------------------ internals

    @property
    def _group_name(self) -> str:
        return str(self._rdk.kJointGroupNames[self._group])

    def _resolve_group(self, name: str | None) -> typing.Any:
        groups = self._info.single_arm_groups
        if not groups:
            msg = "The connected robot has no single-arm joint group."
            raise RuntimeError(msg)
        if name is None:
            return next(iter(groups))
        for group in groups:
            if self._rdk.kJointGroupNames[group] == name:
                return group
        available = [self._rdk.kJointGroupNames[g] for g in groups]
        msg = f"Joint group {name} not found, available single-arm groups: {available}."
        raise ValueError(msg)

    def _states(self) -> typing.Any:
        return self._robot.states()[self._group]

    def _make_operational(self) -> None:
        if self._robot.fault():
            logger.warning("Robot is in fault state, trying to clear it.")
            if not self._robot.ClearFault():
                msg = "Could not clear the robot fault, check Flexiv Elements."
                raise RuntimeError(msg)
        if not self._robot.operational():
            if not self._robot.estop_released():
                msg = "The emergency stop is pressed, release it and try again."
                raise RuntimeError(msg)
            self._robot.ServoOn()
            deadline = time.time() + self._config.startup_timeout
            while not self._robot.operational():
                if time.time() > deadline:
                    status = self._rdk.kOpStatusNames[self._robot.operational_status()]
                    msg = f"Robot did not become operational within {self._config.startup_timeout} s, status: {status}."
                    raise RuntimeError(msg)
                time.sleep(0.5)
        logger.info("Robot is operational.")

    def _read_tool_tcp(self) -> common.Pose:
        """Pose of the active tool's TCP in the flange frame, identity if it cannot be read."""
        try:
            return _pose_from_rdk(self._tool.params(self._group).tcp_location)
        except Exception as e:
            logger.warning("Could not read the active tool parameters, assuming the TCP is at the flange: %s", e)
            return common.Pose()

    def _ensure_mode(self, mode: FlexivControlMode) -> None:
        """Switches the robot into the RDK mode of `mode` if needed and applies the matching settings."""
        rdk_mode = self._rdk_modes[mode]
        if self._robot.mode() != rdk_mode:
            # SwitchMode stops any ongoing motion and blocks until the transition is done.
            self._robot.SwitchMode(rdk_mode)
        if mode == FlexivControlMode.JOINT_IMPEDANCE:
            self._apply_joint_impedance()
        elif mode == FlexivControlMode.CARTESIAN_IMPEDANCE:
            self._apply_cartesian_impedance()

    def _joint_mode(self) -> FlexivControlMode:
        """The mode used for joint targets: the configured one, or joint impedance when Cartesian is configured."""
        if self._config.control_mode == FlexivControlMode.CARTESIAN_IMPEDANCE:
            return FlexivControlMode.JOINT_IMPEDANCE
        return self._config.control_mode

    def _apply_joint_impedance(self) -> None:
        cfg = self._config
        k_q = self.nominal_joint_stiffness if cfg.joint_stiffness is None else np.asarray(cfg.joint_stiffness)
        z_q = [] if cfg.joint_damping_ratio is None else np.asarray(cfg.joint_damping_ratio).tolist()
        self._robot.SetJointImpedance(self._group, k_q.tolist(), z_q)

    def _apply_cartesian_impedance(self) -> None:
        cfg = self._config
        k_x = (
            self.nominal_cartesian_stiffness if cfg.cartesian_stiffness is None else np.asarray(cfg.cartesian_stiffness)
        )
        if cfg.cartesian_damping_ratio is None:
            self._robot.SetCartesianImpedance(self._group, k_x.tolist())
        else:
            self._robot.SetCartesianImpedance(
                self._group, k_x.tolist(), np.asarray(cfg.cartesian_damping_ratio).tolist()
            )
        max_wrench = (
            [float("inf")] * 6 if cfg.max_contact_wrench is None else np.asarray(cfg.max_contact_wrench).tolist()
        )
        self._robot.SetMaxContactWrench(self._group, max_wrench)
        if cfg.null_space_posture is not None:
            self._robot.SetNullSpacePosture(self._group, np.asarray(cfg.null_space_posture).tolist())

    def _send_joint_target(self, q: np.ndarray, max_velocity: float) -> None:
        self._ensure_mode(self._joint_mode())
        cmd = self._rdk.NrtJointPositionCmd(
            q.tolist(),
            [0.0] * self._dof,
            [float(max_velocity)] * self._dof,
            [float(self._config.max_joint_acceleration)] * self._dof,
        )
        self._robot.SendJointPosition({self._group: cmd})

    def _wait_for_joints(self, q: np.ndarray, timeout: float) -> None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._robot.fault():
                msg = "Robot fault while waiting for a joint target."
                raise RuntimeError(msg)
            if np.max(np.abs(self.get_joint_position() - q)) < self._config.joint_tolerance:
                return
            time.sleep(self.POLL_INTERVAL)
        logger.warning("Joint target not reached within %.1f s.", timeout)

    def _wait_for_pose(self, pose: common.Pose, timeout: float) -> None:
        eps_t, eps_r = self._config.cartesian_tolerance
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._robot.fault():
                msg = "Robot fault while waiting for a Cartesian target."
                raise RuntimeError(msg)
            if self.get_cartesian_position().is_close(pose, eps_r=eps_r, eps_t=eps_t):
                return
            time.sleep(self.POLL_INTERVAL)
        logger.warning("Cartesian target not reached within %.1f s.", timeout)


class FlexivGripperConfig(common.GripperConfig):
    """Configuration of a gripper controlled through the RDK gripper device interface."""

    def __init__(
        self,
        device_name: str = "Flexiv-GN01",
        async_control: bool = True,
        manual_init: bool = False,
        init_timeout: float = 15.0,
        velocity: float | None = None,
        force_limit: float | None = None,
        grasp_force: float | None = None,
        force_control_grasp: bool = False,
        width_tolerance: float = 0.002,
        command_timeout: float = 3.0,
        **kwargs: typing.Unpack[GripperConfigKwargs],
    ) -> None:
        """
        Args:
            device_name: Name of the gripper device in Flexiv Elements -> Settings -> Device.
            async_control: If True, commands return once they were handed to the robot. If False, they poll the
                gripper state until the target width is reached, the fingers stopped on an object or
                `command_timeout` hits.
            manual_init: Trigger the gripper's initialization sequence on startup. Needed for grippers that do not
                initialize themselves on power-on, such as the 48 V Grav.
            init_timeout: Seconds to wait for the initialization sequence to finish.
            velocity: Finger velocity in m/s within the gripper's limits. None uses the maximum.
            force_limit: Maximum contact force in N for width commands. None uses half of the gripper's maximum.
            grasp_force: Force in N used by `grasp`. None uses `force_limit`.
            force_control_grasp: If True, `grasp` uses the RDK's direct force control (the fingers close until
                `grasp_force` is reached). If False, `grasp` is a width command to the closed position with
                `grasp_force` as force limit, which works on every gripper.
            width_tolerance: Width error in m below which a synchronous command counts as reached.
            command_timeout: Seconds after which a synchronous command returns even if the width was not reached.
        """
        super().__init__(**kwargs)
        self.gripper_type = common.GripperType("FlexivGrav")
        self.device_name = device_name
        self.async_control = async_control
        self.manual_init = manual_init
        self.init_timeout = init_timeout
        self.velocity = velocity
        self.force_limit = force_limit
        self.grasp_force = grasp_force
        self.force_control_grasp = force_control_grasp
        self.width_tolerance = width_tolerance
        self.command_timeout = command_timeout


class FlexivGripperState(common.GripperState):
    def __init__(self, states: typing.Any) -> None:
        super().__init__()
        self.width = float(states.width)
        """Finger opening in m."""
        self.force = float(states.force)
        """Finger force in N, positive when opening, negative when closing, 0 without force sensing."""
        self.is_moving = bool(states.is_moving)


class FlexivGripper(common.Gripper):
    """RCS gripper controlled through the RDK gripper interface of a `Flexiv` robot."""

    POLL_INTERVAL = 0.01

    def __init__(self, cfg: FlexivGripperConfig, robot: Flexiv):
        super().__init__()
        import flexivrdk

        self._cfg = cfg
        self._robot = robot
        self._group = robot.joint_group
        self._gripper = flexivrdk.Gripper(robot.rdk_robot)
        self._gripper.Enable(self._group, cfg.device_name)
        if cfg.manual_init:
            self._initialize()
        params = self._gripper.params()[self._group]
        self._min_width = float(params.min_width)
        self._max_width = float(params.max_width)
        self._velocity = float(np.clip(cfg.velocity or params.max_vel, params.min_vel, params.max_vel))
        force_limit = cfg.force_limit if cfg.force_limit is not None else 0.5 * params.max_force
        self._force_limit = float(np.clip(force_limit, params.min_force, params.max_force))
        grasp_force = cfg.grasp_force if cfg.grasp_force is not None else self._force_limit
        self._grasp_force = float(np.clip(grasp_force, params.min_force, params.max_force))
        self._closing = False
        logger.info(
            "Gripper %s enabled: width %.3f..%.3f m, velocity %.3f m/s, force limit %.1f N.",
            params.name,
            self._min_width,
            self._max_width,
            self._velocity,
            self._force_limit,
        )

    def get_config(self) -> FlexivGripperConfig:
        return self._cfg

    def get_state(self) -> FlexivGripperState:
        return FlexivGripperState(self._states())

    def get_normalized_width(self) -> float:
        width = (self._states().width - self._min_width) / (self._max_width - self._min_width)
        return float(np.clip(width, 0.0, 1.0))

    def set_normalized_width(self, width: float, force: float = 0) -> None:
        if not (0 <= width <= 1):
            msg = f"Width must be between 0 and 1, got {width}."
            raise ValueError(msg)
        target = self._min_width + width * (self._max_width - self._min_width)
        self._closing = target < self._states().width
        self._gripper.Move(self._group, target, self._velocity, force if force > 0 else self._force_limit)
        if not self._cfg.async_control:
            self._wait_for_width(target)

    def grasp(self) -> None:
        if self._cfg.force_control_grasp:
            self._closing = True
            self._gripper.Grasp(self._group, self._grasp_force)
            if not self._cfg.async_control:
                self._wait_for_width(self._min_width)
            return
        self.set_normalized_width(0.0, force=self._grasp_force)

    def open(self) -> None:
        self.set_normalized_width(1.0)

    def shut(self) -> None:
        self.set_normalized_width(0.0)

    def reset(self) -> None:
        self.open()

    def is_grasped(self) -> bool:
        """True if the fingers stopped on something after a closing command, i.e. before reaching the closed width."""
        states = self._states()
        return self._closing and not states.is_moving and states.width > self._min_width + self._cfg.width_tolerance

    def close(self) -> None:
        """Stops the fingers, the robot connection belongs to the robot and is closed there."""
        try:
            self._gripper.Stop(self._group)
        except Exception as e:
            logger.warning("Stopping the gripper on close failed: %s", e)

    # ------------------------------------------------------------------ internals

    def _states(self) -> typing.Any:
        return self._gripper.states()[self._group]

    def _initialize(self) -> None:
        logger.info("Initializing gripper %s, the fingers will move.", self._cfg.device_name)
        self._gripper.Init(self._group)
        # Init returns once the request is delivered, the sequence itself takes a few seconds.
        deadline = time.time() + self._cfg.init_timeout
        time.sleep(1.0)
        while time.time() < deadline and self._states().is_moving:
            time.sleep(self.POLL_INTERVAL)
        if self._states().is_moving:
            logger.warning("Gripper initialization did not finish within %.1f s.", self._cfg.init_timeout)

    def _wait_for_width(self, target: float) -> None:
        deadline = time.time() + self._cfg.command_timeout
        was_moving = False
        while time.time() < deadline:
            states = self._states()
            if abs(states.width - target) < self._cfg.width_tolerance:
                return
            if was_moving and not states.is_moving:
                # The fingers stopped before reaching the target, e.g. on an object.
                return
            was_moving = was_moving or states.is_moving
            time.sleep(self.POLL_INTERVAL)
        logger.warning("Gripper width %.3f m not reached within %.1f s.", target, self._cfg.command_timeout)
