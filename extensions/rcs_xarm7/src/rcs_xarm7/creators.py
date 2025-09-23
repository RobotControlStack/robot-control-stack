import logging
import typing
from os import PathLike
from pathlib import Path
from typing import Type

import gymnasium as gym
from gymnasium.envs.registration import EnvCreator
from rcs.camera.hw import HardwareCameraSet
from rcs.camera.interface import BaseCameraSet
from rcs.camera.sim import SimCameraSet
from rcs.envs.base import (
    CameraSetWrapper,
    ControlMode,
    GripperWrapper,
    HandWrapper,
    RelativeActionSpace,
    RelativeTo,
    RobotEnv,
)
from rcs.envs.creators import RCSHardwareEnvCreator
from rcs.envs.sim import CollisionGuard, GripperWrapperSim, RobotSimWrapper, SimWrapper
from rcs.hand.tilburg_hand import THConfig, TilburgHand
from rcs.sim import SimCameraConfig, SimGripperConfig, SimRobotConfig
from rcs_xarm7.hw import XArm7

import rcs
from rcs import common, sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class RCSXArm7EnvCreator(RCSHardwareEnvCreator):
    def __call__(  # type: ignore
        self,
        control_mode: ControlMode,
        ip: str,
        calibration_dir: PathLike | str | None = None,
        camera_set: HardwareCameraSet | None = None,
        hand_cfg: THConfig | None = None,
        max_relative_movement: float | tuple[float, float] | None = None,
        relative_to: RelativeTo = RelativeTo.LAST_STEP,
    ) -> gym.Env:
        if isinstance(calibration_dir, str):
            calibration_dir = Path(calibration_dir)
        robot = XArm7(ip=ip)
        env: gym.Env = RobotEnv(robot, control_mode, home_on_reset=True)

        if camera_set is not None:
            camera_set.start()
            camera_set.wait_for_frames()
            logger.info("CameraSet started")
            env = CameraSetWrapper(env, camera_set, include_depth=True)
        if hand_cfg is not None and isinstance(hand_cfg, THConfig):
            hand = TilburgHand(cfg=hand_cfg, verbose=True)
            env = HandWrapper(env, hand, True)

        if max_relative_movement is not None:
            env = RelativeActionSpace(env, max_mov=max_relative_movement, relative_to=relative_to)

        return env


class XArm7SimEnvCreator(EnvCreator):
    def __call__(  # type: ignore
        self,
        control_mode: ControlMode,
        robot_cfg: SimRobotConfig,
        urdf_path: str,
        mjcf: str,
        collision_guard: bool = False,
        gripper_cfg: SimGripperConfig | None = None,
        cameras: dict[str, SimCameraConfig] | None = None,
        max_relative_movement: float | tuple[float, float] | None = None,
        relative_to: RelativeTo = RelativeTo.LAST_STEP,
        sim_wrapper: Type[SimWrapper] | None = None,
    ) -> gym.Env:
        """
        Creates a simulation environment for the FR3 robot.
        Args:
            control_mode (ControlMode): Control mode for the robot.
            robot_cfg (rcs.sim.SimRobotConfig): Configuration for the FR3 robot.
            collision_guard (bool): Whether to use collision guarding. If True, the same mjcf scene is used for collision guarding.
            gripper_cfg (rcs.sim.SimGripperConfig | None): Configuration for the gripper. If None, no gripper is used.
            camera_set_cfg (SimCameraSetConfig | None): Configuration for the camera set. If None, no cameras are used.
            max_relative_movement (float | tuple[float, float] | None): Maximum allowed movement. If float, it restricts
                translational movement in meters. If tuple, it restricts both translational (in meters) and rotational
                (in radians) movements. If None, no restriction is applied.
            relative_to (RelativeTo): Specifies whether the movement is relative to a configured origin or the last step.
            urdf_path (str | PathLike | None): Path to the URDF file. If None, the URDF file is automatically deduced
                which requires a UTN compatible lab scene to be present.
            mjcf (str | PathLike): Path to the Mujoco scene XML file. Defaults to "fr3_empty_world".
            sim_wrapper (Type[SimWrapper] | None): Wrapper to be applied before the simulation wrapper. This is useful
                for reset management e.g. resetting objects in the scene with correct observations. Defaults to None.
        Returns:
            gym.Env: The configured simulation environment for the FR3 robot.
        """
        simulation = sim.Sim(mjcf)

        ik = rcs.common.RL(str(urdf_path))
        robot = rcs.sim.SimRobot(simulation, ik, robot_cfg)
        env: gym.Env = RobotEnv(robot, control_mode)
        env = RobotSimWrapper(env, simulation, sim_wrapper)

        if cameras is not None:
            camera_set = typing.cast(
                BaseCameraSet, SimCameraSet(simulation, cameras, physical_units=True, render_on_demand=True)
            )
            env = CameraSetWrapper(env, camera_set, include_depth=True)

        if gripper_cfg is not None and isinstance(gripper_cfg, SimGripperConfig):
            gripper = sim.SimGripper(simulation, gripper_cfg)
            env = GripperWrapper(env, gripper, binary=False)
            env = GripperWrapperSim(env, gripper)

        if collision_guard:
            env = CollisionGuard.env_from_xml_paths(
                env,
                str(rcs.scenes.get(str(mjcf), mjcf)),
                str(urdf_path),
                gripper=gripper_cfg is not None,
                check_home_collision=False,
                control_mode=control_mode,
                tcp_offset=rcs.common.Pose(common.FrankaHandTCPOffset()),
                sim_gui=True,
                truncate_on_collision=True,
            )
        if max_relative_movement is not None:
            env = RelativeActionSpace(env, max_mov=max_relative_movement, relative_to=relative_to)

        return env
