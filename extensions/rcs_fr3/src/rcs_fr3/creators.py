import logging
from os import PathLike

import gymnasium as gym
import numpy as np
import rcs.hand.tilburg_hand
from rcs.camera.hw import HardwareCameraSet
from rcs.envs.base import (
    CameraSetWrapper,
    ControlMode,
    GripperWrapper,
    HandWrapper,
    MultiRobotWrapper,
    RelativeActionSpace,
    RelativeTo,
    RobotEnv,
)
from rcs.envs.creators import RCSHardwareEnvCreator
from rcs.envs.sim import CollisionGuard
from rcs.hand.tilburg_hand import TilburgHand
from rcs_fr3 import hw
from rcs_fr3.envs import FR3HW
from rcs_fr3.utils import default_fr3_hw_gripper_cfg, default_fr3_hw_robot_cfg

import rcs

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class RCSFR3EnvCreator(RCSHardwareEnvCreator):
    def __call__(  # type: ignore
        self,
        ip: str,
        control_mode: ControlMode,
        robot_cfg: hw.FR3Config,
        collision_guard: str | PathLike | None = None,
        gripper_cfg: hw.FHConfig | rcs.hand.tilburg_hand.THConfig | None = None,
        camera_set: HardwareCameraSet | None = None,
        max_relative_movement: float | tuple[float, float] | None = None,
        relative_to: RelativeTo = RelativeTo.LAST_STEP,
        urdf_path: str | PathLike | None = None,
    ) -> gym.Env:
        """
        Creates a hardware environment for the FR3 robot.

        Args:
            ip (str): IP address of the robot.
            control_mode (ControlMode): Control mode for the robot.
            robot_cfg (hw.FR3Config): Configuration for the FR3 robot.
            collision_guard (str | PathLike | None): Key to a built-in scene
            robot_cfg (hw.FR3Config): Configuration for the FR3 robot.
            collision_guard (str | PathLike | None): Key to a scene (requires UTN compatible scene package to be present)
                or the path to a mujoco scene for collision guarding. If None, collision guarding is not used.
            gripper_cfg (hw.FHConfig | None): Configuration for the gripper. If None, no gripper is used.
            camera_set (BaseHardwareCameraSet | None): Camera set to be used. If None, no cameras are used.
            max_relative_movement (float | tuple[float, float] | None): Maximum allowed movement. If float, it restricts
                translational movement in meters. If tuple, it restricts both translational (in meters) and rotational
                (in radians) movements. If None, no restriction is applied.
            relative_to (RelativeTo): Specifies whether the movement is relative to a configured origin or the last step.
            urdf_path (str | PathLike | None): Path to the URDF file. If None the included one is used. A URDF file is needed for collision guarding.

        Returns:
            gym.Env: The configured hardware environment for the FR3 robot.
        """
        if urdf_path is None:
            urdf_path = rcs.scenes["fr3_empty_world"]["urdf"]
        ik = rcs.common.IK(str(urdf_path)) if urdf_path is not None else None
        robot = hw.FR3(ip, ik)
        robot.set_parameters(robot_cfg)

        env: gym.Env = RobotEnv(robot, ControlMode.JOINTS if collision_guard is not None else control_mode)

        env = FR3HW(env)
        if isinstance(gripper_cfg, hw.FHConfig):
            gripper = hw.FrankaHand(ip, gripper_cfg)
            env = GripperWrapper(env, gripper, binary=True)
        elif isinstance(gripper_cfg, rcs.hand.tilburg_hand.THConfig):
            hand = TilburgHand(gripper_cfg)
            env = HandWrapper(env, hand, binary=True)

        if camera_set is not None:
            camera_set.start()
            camera_set.wait_for_frames()
            logger.info("CameraSet started")
            env = CameraSetWrapper(env, camera_set)

        if collision_guard is not None:
            assert urdf_path is not None
            env = CollisionGuard.env_from_xml_paths(
                env,
                str(rcs.scenes.get(str(collision_guard), collision_guard)),
                str(urdf_path),
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
        robot_cfg: hw.FR3Config,
        gripper_cfg: hw.FHConfig | None = None,
        camera_set: HardwareCameraSet | None = None,
        max_relative_movement: float | tuple[float, float] | None = None,
        relative_to: RelativeTo = RelativeTo.LAST_STEP,
        urdf_path: str | PathLike | None = None,
    ) -> gym.Env:

        urdf_path = rcs.scenes["fr3_empty_world"]["urdf"]
        ik = rcs.common.IK(str(urdf_path)) if urdf_path is not None else None
        robots: dict[str, hw.FR3] = {}
        for ip in ips:
            robots[ip] = hw.FR3(ip, ik)
            robots[ip].set_parameters(robot_cfg)

        envs = {}
        for ip in ips:
            env: gym.Env = RobotEnv(robots[ip], control_mode)
            env = FR3HW(env)
            if gripper_cfg is not None:
                gripper = hw.FrankaHand(ip, gripper_cfg)
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
        camera_set: HardwareCameraSet | None = None,
        gripper: bool = True,
    ) -> gym.Env:
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
