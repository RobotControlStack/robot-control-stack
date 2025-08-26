import logging
import typing
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
    HandWrapper,
    RelativeActionSpace,
    RelativeTo,
    RobotEnv,
)
from rcs.envs.sim import (
    GripperWrapperSim,
    HandWrapperSim,
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
        collision_guard: bool = False,
        gripper_cfg: rcs.sim.SimGripperConfig | None = None,
        hand_cfg: rcs.sim.SimTilburgHandConfig | None = None,
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
                                                           Cannot be used together with hand_cfg.
            hand_cfg (rcs.sim.SimHandConfig | None): Configuration for the hand. If None, no hand is used.
                                                     Cannot be used together with gripper_cfg.
            camera_set_cfg (SimCameraSetConfig | None): Configuration for the camera set. If None, no cameras are used.
            max_relative_movement (float | tuple[float, float] | None): Maximum allowed movement. If float, it restricts
                translational movement in meters. If tuple, it restricts both translational (in meters) and rotational
                (in radians) movements. If None, no restriction is applied.
            relative_to (RelativeTo): Specifies whether the movement is relative to a configured origin or the last step.
            sim_wrapper (Type[SimWrapper] | None): Wrapper to be applied before the simulation wrapper. This is useful
                for reset management e.g. resetting objects in the scene with correct observations. Defaults to None.

        Returns:
            gym.Env: The configured simulation environment for the FR3 robot.
        """
        simulation = sim.Sim(robot_cfg.mjcf_scene_path)

        ik = rcs.common.Pin(
            robot_cfg.kinematic_model_path,
            robot_cfg.attachment_site,
            urdf=robot_cfg.kinematic_model_path.endswith(".urdf"),
        )
        # ik = rcs.common.RL(robot_cfg.kinematic_model_path)

        robot = rcs.sim.SimRobot(simulation, ik, robot_cfg)
        env: gym.Env = RobotEnv(robot, control_mode)
        env = RobotSimWrapper(env, simulation, sim_wrapper)

        if cameras is not None:
            camera_set = typing.cast(
                BaseCameraSet, SimCameraSet(simulation, cameras, physical_units=True, render_on_demand=True)
            )
            env = CameraSetWrapper(env, camera_set, include_depth=True)

        assert not (
            hand_cfg is not None and gripper_cfg is not None
        ), "Hand and gripper configurations cannot be used together."

        if hand_cfg is not None and isinstance(hand_cfg, rcs.sim.SimTilburgHandConfig):
            hand = sim.SimTilburgHand(simulation, hand_cfg)
            env = HandWrapper(env, hand, binary=True)
            env = HandWrapperSim(env, hand)

        if gripper_cfg is not None and isinstance(gripper_cfg, rcs.sim.SimGripperConfig):
            gripper = sim.SimGripper(simulation, gripper_cfg)
            env = GripperWrapper(env, gripper, binary=True)
            env = GripperWrapperSim(env, gripper)

        # TODO: collision guard not working atm
        # if collision_guard:
        #     env = CollisionGuard.env_from_xml_paths(
        #         env,
        #         mjcf,
        #         robot_kinematics,
        #         gripper=gripper_cfg is not None,
        #         check_home_collision=False,
        #         control_mode=control_mode,
        #         tcp_offset=rcs.common.Pose(rcs.common.FrankaHandTCPOffset()),
        #         sim_gui=True,
        #         truncate_on_collision=True,
        #     )
        if max_relative_movement is not None:
            env = RelativeActionSpace(env, max_mov=max_relative_movement, relative_to=relative_to)

        return env


class SimTaskEnvCreator(EnvCreator):
    def __call__(  # type: ignore
        self,
        robot_cfg: rcs.sim.SimRobotConfig,
        render_mode: str = "human",
        control_mode: ControlMode = ControlMode.CARTESIAN_TRPY,
        delta_actions: bool = True,
        cameras: dict[str, SimCameraConfig] | None = None,
        hand_cfg: rcs.sim.SimTilburgHandConfig | None = None,
        gripper_cfg: rcs.sim.SimGripperConfig | None = None,
    ) -> gym.Env:
        mode = "gripper"
        if gripper_cfg is None and hand_cfg is None:
            _gripper_cfg = default_sim_gripper_cfg()
            _hand_cfg = None
            logger.info("Using default gripper configuration.")
        elif hand_cfg is not None:
            _gripper_cfg = None
            _hand_cfg = hand_cfg
            mode = "hand"
            logger.info("Using hand configuration.")
        else:
            # Either both cfgs are set, or only gripper_cfg is set
            _gripper_cfg = gripper_cfg
            _hand_cfg = None
            logger.info("Using gripper configuration.")

        env_rel = SimEnvCreator()(
            control_mode=control_mode,
            robot_cfg=robot_cfg,
            collision_guard=False,
            gripper_cfg=_gripper_cfg,
            hand_cfg=_hand_cfg,
            cameras=cameras,
            max_relative_movement=(0.2, np.deg2rad(45)) if delta_actions else None,
            relative_to=RelativeTo.LAST_STEP,
            sim_wrapper=RandomCubePos,
        )
        if mode == "gripper":
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
        cam_list: list[str] = ["wrist", "bird_eye", "side", "right_side", "left_side", "front"]
    ) -> gym.Env:
        if resolution is None:
            resolution = (256, 256)
        cameras = {
            cam: SimCameraConfig(
                identifier=cam,
                type=CameraType.fixed,
                resolution_height=resolution[1],
                resolution_width=resolution[0],
                frame_rate=frame_rate,
            )
            for cam in cam_list
        }
        robot_cfg = default_sim_robot_cfg(scene="fr3_simple_pick_up")

        return SimTaskEnvCreator()(robot_cfg, render_mode, control_mode, delta_actions, cameras)

class FR3LabDigitGripperPickUpSimEnvCreator(EnvCreator):
    def __call__(  # type: ignore
        self,
        render_mode: str = "human",
        control_mode: ControlMode = ControlMode.CARTESIAN_TRPY,
        resolution: tuple[int, int] | None = None,
        frame_rate: int = 0,
        delta_actions: bool = True,
        cam_list: list[str] = [],
        mjcf_path: str = ''
    ) -> gym.Env:
        if resolution is None:
            resolution = (256, 256)
        if cam_list is None or len(cam_list) == 0:
            error_msg = "cam_list must contain at least one camera name."
            raise ValueError(error_msg)
        cameras = {
            cam: SimCameraConfig(
                identifier=cam,
                type=CameraType.fixed,
                resolution_height=resolution[1],
                resolution_width=resolution[0],
                frame_rate=frame_rate,
            )
            for cam in cam_list
        }
        robot_cfg = rcs.sim.SimRobotConfig()
        robot_cfg.tcp_offset = rcs.common.Pose(translation=np.array([0.0, 0.0, 0.15]), rotation=np.array([[0.707, 0.707, 0], [-0.707, 0.707, 0], [0, 0, 1]]))
        robot_cfg.robot_type = rcs.common.RobotType.FR3
        robot_cfg.realtime = False
        robot_cfg.add_id("0") # only required for fr3
        robot_cfg.mjcf_scene_path = mjcf_path
        robot_cfg.kinematic_model_path = rcs.scenes["fr3_empty_world"].mjcf_robot # .urdf (in case for urdf)
        print(f"Creating FR3LabDigitGripperPickUpSim with the following parameters: \n"
                    f"  render_mode: {render_mode}\n"
                    f"  control_mode: {control_mode}\n"
                    f"  resolution: {resolution}\n"
                    f"  frame_rate: {frame_rate}\n"
                    f"  delta_actions: {delta_actions}\n"
                    f"  cameras: {cameras}\n"
                    f"  mjcf_path: {mjcf_path}\n"
                    )

        return SimTaskEnvCreator()(robot_cfg, render_mode, control_mode, delta_actions, cameras)