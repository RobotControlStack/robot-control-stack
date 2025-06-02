import logging
from os import PathLike
from typing import Type

import gymnasium as gym
import numpy as np
import rcs
import rcs.hand.tilburg_hand
from gymnasium.envs.registration import EnvCreator
from rcs import sim
from rcs._core.sim import CameraType
from rcs.camera.hw import BaseHardwareCameraSet
from rcs.camera.sim import SimCameraConfig, SimCameraSet, SimCameraSetConfig
from rcs.digit_cam.digit_cam import DigitCam
from rcs.envs.base import (
    CameraSetWrapper,
    ControlMode,
    DigitCameraSetWrapper,
    GripperWrapper,
    HandWrapper,
    MultiRobotWrapper,
    RelativeActionSpace,
    RelativeTo,
    RobotEnv,
)
from rcs.envs.hw import FR3HW
from rcs.envs.sim import (
    CamRobot,
    CollisionGuard,
    GripperWrapperSim,
    PickCubeSuccessWrapper,
    RandomCubePos,
    RobotSimWrapper,
    SimWrapper,
)
from rcs.envs.space_utils import VecType
from rcs.envs.utils import (
    default_fr3_hw_gripper_cfg,
    default_fr3_hw_robot_cfg,
    default_fr3_sim_gripper_cfg,
    default_fr3_sim_robot_cfg,
    default_realsense,
    get_urdf_path,
)
from rcs.hand.tilburg_hand import TilburgHand

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class RCSHardwareEnvCreator(EnvCreator):
    pass


class RCSFR3EnvCreator(RCSHardwareEnvCreator):
    def __call__(  # type: ignore
        self,
        ip: str,
        control_mode: ControlMode,
        robot_cfg: rcs.hw.FR3Config,
        collision_guard: str | PathLike | None = None,
        gripper_cfg: rcs.hw.FHConfig | rcs.hand.tilburg_hand.THConfig | None = None,
        camera_set: BaseHardwareCameraSet | None = None,
        digit_set: DigitCam | None = None,
        max_relative_movement: float | tuple[float, float] | None = None,
        relative_to: RelativeTo = RelativeTo.LAST_STEP,
        urdf_path: str | PathLike | None = None,
    ) -> gym.Env:
        """
        Creates a hardware environment for the FR3 robot.

        Args:
            ip (str): IP address of the robot.
            control_mode (ControlMode): Control mode for the robot.
            robot_cfg (rcs.hw.FR3Config): Configuration for the FR3 robot.
            collision_guard (str | PathLike | None): Key to a scene (requires UTN compatible scene package to be present)
                or the path to a mujoco scene for collision guarding. If None, collision guarding is not used.
            gripper_cfg (rcs.hw.FHConfig | None): Configuration for the gripper. If None, no gripper is used.
            camera_set (BaseHardwareCameraSet | None): Camera set to be used. If None, no cameras are used.
            max_relative_movement (float | tuple[float, float] | None): Maximum allowed movement. If float, it restricts
                translational movement in meters. If tuple, it restricts both translational (in meters) and rotational
                (in radians) movements. If None, no restriction is applied.
            relative_to (RelativeTo): Specifies whether the movement is relative to a configured origin or the last step.
            urdf_path (str | PathLike | None): Path to the URDF file. If None, the URDF file is automatically deduced
                which requires a UTN compatible lab scene to be present. If no URDF file is found, the environment will
                still work but set_cartesian methods might throw an error. A URDF file is needed for collision guarding.

        Returns:
            gym.Env: The configured hardware environment for the FR3 robot.
        """
        urdf_path = get_urdf_path(urdf_path, allow_none_if_not_found=collision_guard is not None)
        ik = rcs.common.IK(str(urdf_path)) if urdf_path is not None else None
        robot = rcs.hw.FR3(ip, ik)
        robot.set_parameters(robot_cfg)

        env: gym.Env = RobotEnv(robot, ControlMode.JOINTS if collision_guard is not None else control_mode)

        env = FR3HW(env)
        if isinstance(gripper_cfg, rcs.hw.FHConfig):
            gripper = rcs.hw.FrankaHand(ip, gripper_cfg)
            env = GripperWrapper(env, gripper, binary=True)
        elif isinstance(gripper_cfg, rcs.hand.tilburg_hand.THConfig):
            hand = TilburgHand(gripper_cfg)
            env = HandWrapper(env, hand, binary=True)

        if camera_set is not None:
            camera_set.start()
            camera_set.wait_for_frames()
            logger.info("CameraSet started")
            env = CameraSetWrapper(env, camera_set)

        if digit_set is not None:
            digit_set.start()
            digit_set.wait_for_frames()
            logger.info("DigitCameraSet started")
            env = DigitCameraSetWrapper(env, digit_set)

        if collision_guard is not None:
            assert urdf_path is not None
            env = CollisionGuard.env_from_xml_paths(
                env,
                str(rcs.scenes.get(str(collision_guard), collision_guard)),
                urdf_path,
                gripper=True,
                check_home_collision=False,
                control_mode=control_mode,
                tcp_offset=rcs.common.Pose(rcs.common.FrankaHandTCPOffset()),
                sim_gui=True,
                truncate_on_collision=False,
            )
        if max_relative_movement is not None:
            env = RelativeActionSpace(env, max_mov=max_relative_movement, relative_to=relative_to)

        return env


class RCSFR3MultiEnvCreator(RCSHardwareEnvCreator):
    def __call__(  # type: ignore
        ips: list[str],
        control_mode: ControlMode,
        robot_cfg: rcs.hw.FR3Config,
        gripper_cfg: rcs.hw.FHConfig | None = None,
        camera_set: BaseHardwareCameraSet | None = None,
        max_relative_movement: float | tuple[float, float] | None = None,
        relative_to: RelativeTo = RelativeTo.LAST_STEP,
        urdf_path: str | PathLike | None = None,
    ) -> gym.Env:

        urdf_path = get_urdf_path(urdf_path, allow_none_if_not_found=False)
        ik = rcs.common.IK(str(urdf_path)) if urdf_path is not None else None
        robots: dict[str, rcs.hw.FR3] = {}
        for ip in ips:
            robots[ip] = rcs.hw.FR3(ip, ik)
            robots[ip].set_parameters(robot_cfg)

        envs = {}
        for ip in ips:
            env: gym.Env = RobotEnv(robots[ip], control_mode)
            env = FR3HW(env)
            if gripper_cfg is not None:
                gripper = rcs.hw.FrankaHand(ip, gripper_cfg)
                env = GripperWrapper(env, gripper, binary=True)

            if max_relative_movement is not None:
                env = RelativeActionSpace(env, max_mov=max_relative_movement, relative_to=relative_to)
            envs[ip] = env

        env = MultiRobotWrapper(envs)
        if camera_set is not None:
            camera_set.start()
            camera_set.wait_for_frames()
            logger.info("CameraSet started")
            env = CameraSetWrapper(env, camera_set)
        return env


class RCSFR3DefaultEnvCreator(RCSHardwareEnvCreator):
    def __call__(  # type: ignore
        self,
        robot_ip: str,
        control_mode: ControlMode = ControlMode.CARTESIAN_TRPY,
        delta_actions: bool = True,
        camera_config: dict[str, str] | None = None,
        gripper: bool = True,
    ) -> gym.Env:
        camera_set = default_realsense(camera_config)
        return RCSFR3EnvCreator()(
            ip=robot_ip,
            camera_set=camera_set,
            control_mode=control_mode,
            robot_cfg=default_fr3_hw_robot_cfg(),
            collision_guard=None,
            gripper_cfg=default_fr3_hw_gripper_cfg() if gripper else None,
            max_relative_movement=(0.2, np.deg2rad(45)) if delta_actions else None,
            relative_to=RelativeTo.LAST_STEP,
        )


class RCSSimEnvCreator(EnvCreator):
    def __call__(  # type: ignore
        self,
        control_mode: ControlMode,
        robot_cfg: rcs.sim.SimRobotConfig,
        collision_guard: bool = False,
        gripper_cfg: rcs.sim.SimGripperConfig | None = None,
        camera_set_cfg: SimCameraSetConfig | None = None,
        max_relative_movement: float | tuple[float, float] | None = None,
        relative_to: RelativeTo = RelativeTo.LAST_STEP,
        urdf_path: str | PathLike | None = None,
        mjcf: str | PathLike = "fr3_empty_world",
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
        urdf_path = get_urdf_path(urdf_path, allow_none_if_not_found=False)
        assert urdf_path is not None
        if mjcf not in rcs.scenes:
            logger.warning("mjcf not found as key in scenes, interpreting mjcf as path the mujoco scene xml")
        mjb_file = rcs.scenes.get(str(mjcf), mjcf)
        simulation = sim.Sim(mjb_file)

        ik = rcs.common.IK(urdf_path)
        robot = rcs.sim.SimRobot(simulation, ik, robot_cfg)
        env: gym.Env = RobotEnv(robot, control_mode)
        env = RobotSimWrapper(env, simulation, sim_wrapper)

        if camera_set_cfg is not None:
            camera_set = SimCameraSet(simulation, camera_set_cfg)
            env = CameraSetWrapper(env, camera_set, include_depth=True)

        if gripper_cfg is not None and isinstance(gripper_cfg, rcs.sim.SimGripperConfig):
            gripper = sim.SimGripper(simulation, gripper_cfg)
            env = GripperWrapper(env, gripper, binary=True)
            env = GripperWrapperSim(env, gripper)

        if collision_guard:
            env = CollisionGuard.env_from_xml_paths(
                env,
                str(rcs.scenes.get(str(mjcf), mjcf)),
                urdf_path,
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
        camera_cfg: SimCameraSetConfig | None = None,
    ) -> gym.Env:

        env_rel = RCSSimEnvCreator()(
            control_mode=control_mode,
            robot_cfg=default_fr3_sim_robot_cfg(mjcf),
            collision_guard=False,
            gripper_cfg=default_fr3_sim_gripper_cfg(),
            camera_set_cfg=camera_cfg,
            max_relative_movement=(0.2, np.deg2rad(45)) if delta_actions else None,
            relative_to=RelativeTo.LAST_STEP,
            mjcf=mjcf,
            sim_wrapper=RandomCubePos,
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
            "wrist": SimCameraConfig(identifier="wrist_0", type=int(CameraType.fixed)),
            "bird_eye": SimCameraConfig(identifier="bird_eye_cam", type=int(CameraType.fixed)),
            "side": SimCameraConfig(identifier="side_view", type=int(CameraType.fixed)),
            "right_side": SimCameraConfig(identifier="right_side", type=int(CameraType.fixed)),
            "left_side": SimCameraConfig(identifier="left_side", type=int(CameraType.fixed)),
            "front": SimCameraConfig(identifier="front", type=int(CameraType.fixed)),
        }

        camera_cfg = SimCameraSetConfig(
            cameras=cameras,
            resolution_width=resolution[0],
            resolution_height=resolution[1],
            frame_rate=frame_rate,
            physical_units=True,
        )
        return SimTaskEnvCreator()("fr3_simple_pick_up", render_mode, control_mode, delta_actions, camera_cfg)


class FR3SimplePickUpSimDigitHandEnvCreator(EnvCreator):
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

        cameras = {"wrist": SimCameraConfig(identifier="wrist_0", type=int(CameraType.fixed))}

        camera_cfg = SimCameraSetConfig(
            cameras=cameras,
            resolution_width=resolution[0],
            resolution_height=resolution[1],
            frame_rate=frame_rate,
            physical_units=True,
        )
        return SimTaskEnvCreator()(
            "fr3_simple_pick_up_digit_hand", render_mode, control_mode, delta_actions, camera_cfg
        )


class FR3LabPickUpSimDigitHandEnvCreator(EnvCreator):
    def __call__(  # type: ignore
        self,
        cam_robot_joints: VecType,
        render_mode: str = "human",
        control_mode: ControlMode = ControlMode.CARTESIAN_TRPY,
        resolution: tuple[int, int] | None = None,
        frame_rate: int = 0,
        delta_actions: bool = True,
    ) -> gym.Env:
        if resolution is None:
            resolution = (256, 256)

        cameras = {
            "wrist": SimCameraConfig(identifier="wrist_0", type=int(CameraType.fixed)),
            "side": SimCameraConfig(identifier="wrist_1", type=int(CameraType.fixed)),
        }

        camera_cfg = SimCameraSetConfig(
            cameras=cameras,
            resolution_width=resolution[0],
            resolution_height=resolution[1],
            frame_rate=frame_rate,
            physical_units=True,
        )
        env_rel = SimTaskEnvCreator()(
            "lab_simple_pick_up_digit_hand",
            render_mode,
            control_mode,
            delta_actions,
            camera_cfg,
        )
        return CamRobot(env_rel, cam_robot_joints, "lab_simple_pick_up_digit_hand")
