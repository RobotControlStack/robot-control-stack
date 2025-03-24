import logging
from os import PathLike
from typing import Type

import gymnasium as gym
import numpy as np
import rcsss
from gymnasium.envs.registration import EnvCreator
from rcsss import sim
from rcsss._core.sim import CameraType
from rcsss.camera.hw import BaseHardwareCameraSet
from rcsss.camera.sim import SimCameraConfig, SimCameraSet, SimCameraSetConfig
from rcsss.envs.base import (
    CameraSetWrapper,
    ControlMode,
    FR3Env,
    GripperWrapper,
    RelativeActionSpace,
    RelativeTo,
)
from rcsss.envs.hw import FR3HW
from rcsss.envs.sim import (
    CollisionGuard,
    FR3LabPickUpSimSuccessWrapper,
    FR3Sim,
    FR3SimplePickUpSimSuccessWrapper,
    RandomCubePos,
    RandomCubePosLab,
    SimWrapper,
)
from rcsss.envs.utils import (
    default_fr3_hw_gripper_cfg,
    default_fr3_hw_robot_cfg,
    default_fr3_sim_gripper_cfg,
    default_fr3_sim_robot_cfg,
    default_realsense,
    get_urdf_path,
)

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def fr3_hw_env(
    ip: str,
    control_mode: ControlMode,
    robot_cfg: rcsss.hw.FR3Config,
    collision_guard: str | PathLike | None = None,
    gripper_cfg: rcsss.hw.FHConfig | None = None,
    camera_set: BaseHardwareCameraSet | None = None,
    max_relative_movement: float | tuple[float, float] | None = None,
    relative_to: RelativeTo = RelativeTo.LAST_STEP,
    urdf_path: str | PathLike | None = None,
) -> gym.Env:
    """
    Creates a hardware environment for the FR3 robot.

    Args:
        ip (str): IP address of the robot.
        control_mode (ControlMode): Control mode for the robot.
        robot_cfg (rcsss.hw.FR3Config): Configuration for the FR3 robot.
        collision_guard (str | PathLike | None): Key to a scene (requires UTN compatible scene package to be present)
            or the path to a mujoco scene for collision guarding. If None, collision guarding is not used.
        gripper_cfg (rcsss.hw.FHConfig | None): Configuration for the gripper. If None, no gripper is used.
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
    ik = rcsss.common.IK(str(urdf_path)) if urdf_path is not None else None
    robot = rcsss.hw.FR3(ip, ik)
    robot.set_parameters(robot_cfg)

    env: gym.Env = FR3Env(robot, ControlMode.JOINTS if collision_guard is not None else control_mode)

    env = FR3HW(env)
    if gripper_cfg is not None:
        gripper = rcsss.hw.FrankaHand(ip, gripper_cfg)
        env = GripperWrapper(env, gripper, binary=True)

    if camera_set is not None:
        camera_set.start()
        camera_set.wait_for_frames()
        logger.info("CameraSet started")
        env = CameraSetWrapper(env, camera_set)

    if collision_guard is not None:
        assert urdf_path is not None
        env = CollisionGuard.env_from_xml_paths(
            env,
            str(rcsss.scenes.get(str(collision_guard), collision_guard)),
            urdf_path,
            gripper=True,
            check_home_collision=False,
            control_mode=control_mode,
            tcp_offset=rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset()),
            sim_gui=True,
            truncate_on_collision=False,
        )
    if max_relative_movement is not None:
        env = RelativeActionSpace(env, max_mov=max_relative_movement, relative_to=relative_to)

    return env


def fr3_sim_env(
    control_mode: ControlMode,
    robot_cfg: rcsss.sim.FR3Config,
    collision_guard: bool = False,
    gripper_cfg: rcsss.sim.FHConfig | None = None,
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
        robot_cfg (rcsss.sim.FR3Config): Configuration for the FR3 robot.
        collision_guard (bool): Whether to use collision guarding. If True, the same mjcf scene is used for collision guarding.
        gripper_cfg (rcsss.sim.FHConfig | None): Configuration for the gripper. If None, no gripper is used.
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
    if mjcf not in rcsss.scenes:
        logger.warning("mjcf not found as key in scenes, interpreting mjcf as path the mujoco scene xml")
    mjb_file = rcsss.scenes.get(str(mjcf), mjcf)
    simulation = sim.Sim(mjb_file)
    """
    todo, without a simulation step, sim->d is not populated with the correct values in the robot. 
    check robot base world coordinates without this line to confirm.
    """
    simulation.step(1)

    ik = rcsss.common.IK(urdf_path)
    robot = rcsss.sim.FR3(simulation, "0", ik)
    robot.set_parameters(robot_cfg)
    env: gym.Env = FR3Env(robot, control_mode)
    env = FR3Sim(env, simulation, sim_wrapper)

    if camera_set_cfg is not None:
        camera_set = SimCameraSet(simulation, camera_set_cfg)
        env = CameraSetWrapper(env, camera_set, include_depth=True)

    if gripper_cfg is not None:
        gripper = sim.FrankaHand(simulation, "0", gripper_cfg)
        env = GripperWrapper(env, gripper, binary=True)

    if collision_guard:
        env = CollisionGuard.env_from_xml_paths(
            env,
            str(rcsss.scenes.get(str(mjcf), mjcf)),
            urdf_path,
            gripper=gripper_cfg is not None,
            check_home_collision=False,
            control_mode=control_mode,
            tcp_offset=rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset()),
            sim_gui=True,
            truncate_on_collision=True,
        )
    if max_relative_movement is not None:
        env = RelativeActionSpace(env, max_mov=max_relative_movement, relative_to=relative_to)

    return env


class FR3Real(EnvCreator):
    def __call__(  # type: ignore
        self,
        robot_ip: str,
        control_mode: str = "xyzrpy",
        delta_actions: bool = True,
        camera_config: dict[str, str] | None = None,
        gripper: bool = True,
    ) -> gym.Env:
        camera_set = default_realsense(camera_config)
        return fr3_hw_env(
            ip=robot_ip,
            camera_set=camera_set,
            control_mode=(
                ControlMode.CARTESIAN_TRPY
                if control_mode == "xyzrpy"
                else ControlMode.JOINTS if control_mode == "joints" else ControlMode.CARTESIAN_TQuart
            ),
            robot_cfg=default_fr3_hw_robot_cfg(),
            collision_guard=None,
            gripper_cfg=default_fr3_hw_gripper_cfg() if gripper else None,
            max_relative_movement=(0.2, np.deg2rad(45)) if delta_actions else None,
            relative_to=RelativeTo.LAST_STEP,
        )


class FR3SimplePickUpSim(EnvCreator):
    def __call__(  # type: ignore
        self,
        render_mode: str = "human",
        control_mode: str = "xyzrpy",
        resolution: tuple[int, int] | None = None,
        frame_rate: int = 10,
        delta_actions: bool = True,
    ) -> gym.Env:
        if resolution is None:
            resolution = (256, 256)

        cameras = {
            "wrist": SimCameraConfig(identifier="eye-in-hand_0", type=int(CameraType.fixed)),
            "bird_eye": SimCameraConfig(identifier="bird-eye-cam", type=int(CameraType.fixed)),
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
        env_rel = fr3_sim_env(
            control_mode=(
                ControlMode.CARTESIAN_TRPY
                if control_mode == "xyzrpy"
                else ControlMode.JOINTS if control_mode == "joints" else ControlMode.CARTESIAN_TQuart
            ),
            robot_cfg=default_fr3_sim_robot_cfg(),
            collision_guard=False,
            gripper_cfg=default_fr3_sim_gripper_cfg(),
            camera_set_cfg=camera_cfg,
            max_relative_movement=(0.2, np.deg2rad(45)) if delta_actions else None,
            relative_to=RelativeTo.LAST_STEP,
            mjcf="fr3_simple_pick_up",
            sim_wrapper=RandomCubePos,
        )
        env_rel = FR3SimplePickUpSimSuccessWrapper(env_rel)
        if render_mode == "human":
            env_rel.get_wrapper_attr("sim").open_gui()

        return env_rel


class FR3SimplePickUpSimDigitHand(EnvCreator):
    def __call__(  # type: ignore
        self,
        render_mode: str = "human",
        control_mode: str = "xyzrpy",
        resolution: tuple[int, int] | None = None,
        frame_rate: int = 10,
        delta_actions: bool = True,
    ) -> gym.Env:
        if resolution is None:
            resolution = (256, 256)

        cameras = {"wrist": SimCameraConfig(identifier="eye-in-hand_0", type=int(CameraType.fixed))}

        camera_cfg = SimCameraSetConfig(
            cameras=cameras,
            resolution_width=resolution[0],
            resolution_height=resolution[1],
            frame_rate=frame_rate,
            physical_units=True,
        )

        env_rel = fr3_sim_env(
            control_mode=(
                ControlMode.CARTESIAN_TRPY
                if control_mode == "xyzrpy"
                else ControlMode.JOINTS if control_mode == "joints" else ControlMode.CARTESIAN_TQuart
            ),
            robot_cfg=default_fr3_sim_robot_cfg("fr3_simple_pick_up_digit_hand"),
            collision_guard=False,
            gripper_cfg=default_fr3_sim_gripper_cfg(),
            camera_set_cfg=camera_cfg,
            max_relative_movement=(0.2, np.deg2rad(45)) if delta_actions else None,
            relative_to=RelativeTo.LAST_STEP,
            mjcf="fr3_simple_pick_up_digit_hand",
            sim_wrapper=RandomCubePos,
        )
        env_rel = FR3SimplePickUpSimSuccessWrapper(env_rel)
        if render_mode == "human":
            env_rel.get_wrapper_attr("sim").open_gui()

        return env_rel


class FR3LabPickUpSimDigitHand(EnvCreator):
    def __call__(  # type: ignore
        self,
        render_mode: str = "human",
        control_mode: str = "xyzrpy",
        resolution: tuple[int, int] | None = None,
        frame_rate: int = 10,
        delta_actions: bool = True,
    ) -> gym.Env:
        if resolution is None:
            resolution = (256, 256)

        cameras = {
            "eye-in-hand_0": SimCameraConfig(identifier="eye-in-hand_0", type=int(CameraType.fixed)),
            "eye-in-hand_1": SimCameraConfig(identifier="eye-in-hand_1", type=int(CameraType.fixed)),
        }

        camera_cfg = SimCameraSetConfig(
            cameras=cameras,
            resolution_width=resolution[0],
            resolution_height=resolution[1],
            frame_rate=frame_rate,
            physical_units=True,
        )

        env_rel = fr3_sim_env(
            control_mode=(
                ControlMode.CARTESIAN_TRPY
                if control_mode == "xyzrpy"
                else ControlMode.JOINTS if control_mode == "joints" else ControlMode.CARTESIAN_TQuart
            ),
            robot_cfg=default_fr3_sim_robot_cfg("lab_simple_pick_up_digit_hand"),
            collision_guard=False,
            gripper_cfg=default_fr3_sim_gripper_cfg(),
            camera_set_cfg=camera_cfg,
            max_relative_movement=(0.2, np.deg2rad(45)) if delta_actions else None,
            relative_to=RelativeTo.LAST_STEP,
            mjcf="lab_simple_pick_up_digit_hand",
            sim_wrapper=RandomCubePosLab,
        )
        env_rel = FR3LabPickUpSimSuccessWrapper(env_rel)
        sim = env_rel.get_wrapper_attr("sim")
        if render_mode == "human":
            sim.open_gui()

        return env_rel
