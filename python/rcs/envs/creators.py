import logging
import typing
from os import PathLike
from typing import Type

import gymnasium as gym
import numpy as np
from gymnasium.envs.registration import EnvCreator
from rcs._core.sim import CameraType
from rcs.camera.interface import BaseCameraSet
from rcs.camera.sim import SimCameraConfig, SimCameraSet
from rcs.envs.base import (
    CameraSetWrapper,
    ControlMode,
    GripperWrapper,
    RelativeActionSpace,
    RelativeTo,
    RobotEnv,
)
from rcs.envs.sim import (
    CollisionGuard,
    GripperWrapperSim,
    PickCubeSuccessWrapper,
    RandomCubePos,
    RobotSimWrapper,
    SimWrapper,
)
from rcs.envs.utils import default_sim_gripper_cfg, default_sim_robot_cfg

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class RCSHardwareEnvCreator(EnvCreator):
    pass


class SimEnvCreator(EnvCreator):
    def __call__(  # type: ignore
        self,
        control_mode: ControlMode,
        robot_cfg: rcs.sim.SimRobotConfig,
        robot_kinematics_path: str | PathLike,
        mjcf: str | PathLike,
        kinematics_frame_id: str = "attachment_site",
        collision_guard: bool = False,
        gripper_cfg: rcs.sim.SimGripperConfig | None = None,
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
            kinematics_frame_id (str): Frame name of the flange. Defaults to "attachment_site". Note: if name doesnt exist
                then pinocchio just segfaults.
            relative_to (RelativeTo): Specifies whether the movement is relative to a configured origin or the last step.
            robot_kinematics_path (str | PathLike | None): Path to either urdf or mjcf file that describes the robot kinematic.
                This is used for ik.
            mjcf (str | PathLike): Path to the Mujoco scene XML file. Defaults to "fr3_empty_world".
            sim_wrapper (Type[SimWrapper] | None): Wrapper to be applied before the simulation wrapper. This is useful
                for reset management e.g. resetting objects in the scene with correct observations. Defaults to None.

        Returns:
            gym.Env: The configured simulation environment for the FR3 robot.
        """
        if mjcf not in rcs.scenes:
            logger.info("mjcf not found as key in scenes, interpreting mjcf as path the mujoco scene xml")
        if mjcf in rcs.scenes:
            assert isinstance(mjcf, str)
            mjcf = rcs.scenes[mjcf]["mjb"]
        simulation = sim.Sim(mjcf)

        ik = rcs.common.Pin(
            str(robot_kinematics_path), kinematics_frame_id, urdf=robot_kinematics_path.endswith(".urdf")
        )

        robot = rcs.sim.SimRobot(simulation, ik, robot_cfg)
        env: gym.Env = RobotEnv(robot, control_mode)
        env = RobotSimWrapper(env, simulation, sim_wrapper)

        if cameras is not None:
            camera_set = typing.cast(
                BaseCameraSet, SimCameraSet(simulation, cameras, physical_units=True, render_on_demand=True)
            )
            env = CameraSetWrapper(env, camera_set, include_depth=True)

        if gripper_cfg is not None and isinstance(gripper_cfg, rcs.sim.SimGripperConfig):
            gripper = sim.SimGripper(simulation, gripper_cfg)
            env = GripperWrapper(env, gripper, binary=True)
            env = GripperWrapperSim(env, gripper)

        if collision_guard:
            env = CollisionGuard.env_from_xml_paths(
                env,
                str(rcs.scenes.get(str(mjcf), mjcf)),
                str(robot_kinematics_path),
                gripper=gripper_cfg is not None,
                check_home_collision=False,
                control_mode=control_mode,
                tcp_offset=rcs.common.Pose(rcs.common.FrankaHandTCPOffset()),
                sim_gui=True,
                truncate_on_collision=True,
            )
        if max_relative_movement is not None:
            env = RelativeActionSpace(env, max_mov=max_relative_movement, relative_to=relative_to)

        return env


class SimTaskEnvCreator(EnvCreator):
    def __call__(  # type: ignore
        self,
        mjcf: str,
        render_mode: str = "human",
        control_mode: ControlMode = ControlMode.CARTESIAN_TRPY,
        delta_actions: bool = True,
        cameras: dict[str, SimCameraConfig] | None = None,
    ) -> gym.Env:

        env_rel = SimEnvCreator()(
            control_mode=control_mode,
            robot_cfg=default_sim_robot_cfg(rcs.scenes[mjcf]["mjcf_scene"]),
            collision_guard=False,
            gripper_cfg=default_sim_gripper_cfg(),
            cameras=cameras,
            max_relative_movement=(0.2, np.deg2rad(45)) if delta_actions else None,
            relative_to=RelativeTo.LAST_STEP,
            mjcf=rcs.scenes[mjcf]["mjcf_scene"],
            sim_wrapper=RandomCubePos,
            robot_kinematics_path=rcs.scenes[mjcf]["mjcf_robot"],
        )
        env_rel = PickCubeSuccessWrapper(env_rel)
        if render_mode == "human":
            env_rel.get_wrapper_attr("sim").open_gui()

        return env_rel


class FR3SimplePickUpSimEnvCreator(EnvCreator):
    def __call__(  # type: ignore
        self,
        render_mode: str = "human",
        control_mode: ControlMode = ControlMode.CARTESIAN_TRPY,
        resolution: tuple[int, int] | None = None,
        frame_rate: int = 0,
        delta_actions: bool = True,
    ) -> gym.Env:
        if resolution is None:
            resolution = (256, 256)

        cameras = {
            "wrist": SimCameraConfig(
                identifier="wrist_0",
                type=CameraType.fixed,
                resolution_height=resolution[1],
                resolution_width=resolution[0],
                frame_rate=frame_rate,
            ),
            "bird_eye": SimCameraConfig(
                identifier="bird_eye_cam",
                type=CameraType.fixed,
                resolution_height=resolution[1],
                resolution_width=resolution[0],
                frame_rate=frame_rate,
            ),
            "side": SimCameraConfig(
                identifier="side_view",
                type=CameraType.fixed,
                resolution_height=resolution[1],
                resolution_width=resolution[0],
                frame_rate=frame_rate,
            ),
            "right_side": SimCameraConfig(
                identifier="right_side",
                type=CameraType.fixed,
                resolution_height=resolution[1],
                resolution_width=resolution[0],
                frame_rate=frame_rate,
            ),
            "left_side": SimCameraConfig(
                identifier="left_side",
                type=CameraType.fixed,
                resolution_height=resolution[1],
                resolution_width=resolution[0],
                frame_rate=frame_rate,
            ),
            "front": SimCameraConfig(
                identifier="front",
                type=CameraType.fixed,
                resolution_height=resolution[1],
                resolution_width=resolution[0],
                frame_rate=frame_rate,
            ),
        }

        return SimTaskEnvCreator()("fr3_simple_pick_up", render_mode, control_mode, delta_actions, cameras)
