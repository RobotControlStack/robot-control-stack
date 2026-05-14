import copy
import time
from typing import ClassVar, Literal

import gymnasium as gym
import numpy as np
from rcs._core.common import FrankaHandTCPOffset, GripperType, RobotType
from rcs._core.sim import (
    CameraType,
    SimCameraConfig,
    SimConfig,
    SimGripperConfig,
    SimRobotConfig,
)
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.scenes import (
    CameraAdderConfig,
    SimEnvCreator,
    SimEnvCreatorConfig,
    WrapperConfig,
)

import rcs
from rcs import (
    CAMERA_PATHS,
    DEFAULT_TRANSFORMS,
    GRIPPER_OFFSETS,
    OBJECT_PATHS,
    SCENE_PATHS,
)


class EmptyWorldFR3(SimEnvCreator):
    robot_prefix_template = "robot"
    gripper_prefix_template = "gripper"

    def config(self) -> SimEnvCreatorConfig:
        q_home = rcs.ROBOTS[RobotType.FR3].q_home
        q_home[-1] = np.pi / 4
        robot_cfg: SimRobotConfig[Literal[7]] = SimRobotConfig(
            robot_type=RobotType.FR3,
            tcp_offset=GRIPPER_OFFSETS[rcs.common.GripperType.FrankaHand],
            attachment_site=rcs.ROBOTS[RobotType.FR3].attachment_site,
            kinematic_model_path=rcs.ROBOTS[RobotType.FR3].mjcf_model_path,
            joint_rotational_tolerance=0.05 * (np.pi / 180.0),
            seconds_between_callbacks=0.1,
            trajectory_trace=False,
            arm_collision_geoms=[
                "fr3_link0_collision",
                "fr3_link1_collision",
                "fr3_link2_collision",
                "fr3_link3_collision",
                "fr3_link4_collision",
                "fr3_link5_collision",
                "fr3_link6_collision",
                "fr3_link7_collision",
            ],
            joints=[
                "fr3_joint1",
                "fr3_joint2",
                "fr3_joint3",
                "fr3_joint4",
                "fr3_joint5",
                "fr3_joint6",
                "fr3_joint7",
            ],
            actuators=[
                "fr3_joint1",
                "fr3_joint2",
                "fr3_joint3",
                "fr3_joint4",
                "fr3_joint5",
                "fr3_joint6",
                "fr3_joint7",
            ],
            base="base",
            dof=rcs.ROBOTS[RobotType.FR3].dof,
            joint_limits=rcs.ROBOTS[RobotType.FR3].joint_limits,
            q_home=q_home,
        )

        robot_cfgs: dict[str, SimRobotConfig] = {"robot": robot_cfg}
        sim_cfg: SimConfig = SimConfig(async_control=False, realtime=True, frequency=1, max_convergence_steps=500)

        control_mode: ControlMode = ControlMode.CARTESIAN_TQuat
        task_cfg = None
        scene: str = SCENE_PATHS["empty_world"]
        gripper_cfg = SimGripperConfig(
            epsilon_inner=0.005,
            epsilon_outer=0.005,
            seconds_between_callbacks=0.1,
            ignored_collision_geoms=[],
            collision_geoms=["hand_c", "finger_0_left", "finger_0_right"],
            collision_geoms_fingers=["finger_0_left", "finger_0_right"],
            joints=["finger_joint1", "finger_joint2"],
            max_joint_width=0.04,
            min_joint_width=0.0,
            actuator="hand_actuator",
            max_actuator_width=255.0,
            min_actuator_width=0.0,
            gripper_type=GripperType.FrankaHand,
        )
        gripper_cfgs: dict[str, SimGripperConfig] = {"robot": gripper_cfg}
        camera_cfgs: dict[str, SimCameraConfig] | None = {
            "bird_eye": SimCameraConfig(
                identifier="bird_eye",
                type=CameraType.fixed,
                resolution_width=1280,
                resolution_height=720,
                frame_rate=30,
            ),
            "wrist": SimCameraConfig(
                identifier="wrist",
                type=CameraType.fixed,
                resolution_width=1280,
                resolution_height=720,
                frame_rate=30,
            ),
        }
        max_relative_movement: float | tuple[float, float] | None = None
        relative_to: RelativeTo = RelativeTo.LAST_STEP

        robot_to_shared_base_frame: dict[str, rcs.common.Pose] | None = {"robot": rcs.common.Pose()}
        wrapper_cfg: WrapperConfig = WrapperConfig(binary_gripper=True, home_on_reset=True)
        headless = False
        add_gravcomp = True

        shared_base_frame_to_root_frame = rcs.common.Pose()
        root_frame_to_world = rcs.common.Pose()

        alternative_combined_robot_mjcf: str | None = None

        world_frame_objects: dict[str, tuple[str, rcs.common.Pose]] | None = None
        root_frame_objects: dict[str, tuple[str, rcs.common.Pose]] | None = None

        add_camera_adds: dict[str, CameraAdderConfig] | None = {
            "bird_eye": CameraAdderConfig(
                fovy=60.0,
                offset=rcs.common.Pose(
                    translation=np.array([0.271, -0.000, 2.080]),
                    quaternion=np.array([0.0060, -0.0060, -0.7067, 0.7074]),
                ),
            ),
            "wrist": CameraAdderConfig(
                fovy=60.0,
                offset=rcs.common.Pose(
                    translation=np.array([0.0, 0.0, 0.0]), quaternion=np.array([0.0, 0.0, -0.3826834, 0.9238795])
                )
                * rcs.common.Pose(
                    translation=np.array([0.062, -0.009, 0.05245]), rpy_vector=np.array([0.0, np.pi, -np.pi / 2])
                ),
                robot_name="robot",
            ),
        }
        gripper_offsets: dict[str, rcs.common.Pose] | None = {
            "robot": rcs.common.Pose(rotation=FrankaHandTCPOffset()[:3, :3], translation=np.array([0.0, 0.0, 0.0]))
        }
        return SimEnvCreatorConfig(
            robot_cfgs=robot_cfgs,
            sim_cfg=sim_cfg,
            control_mode=control_mode,
            task_cfg=task_cfg,
            scene=scene,
            gripper_cfgs=gripper_cfgs,
            camera_cfgs=camera_cfgs,
            max_relative_movement=max_relative_movement,
            relative_to=relative_to,
            robot_to_shared_base_frame=robot_to_shared_base_frame,
            wrapper_cfg=wrapper_cfg,
            headless=headless,
            add_gravcomp=add_gravcomp,
            shared_base_frame_to_root_frame=shared_base_frame_to_root_frame,
            root_frame_to_world=root_frame_to_world,
            alternative_combined_robot_mjcf=alternative_combined_robot_mjcf,
            world_frame_objects=world_frame_objects,
            root_frame_objects=root_frame_objects,
            camera_adds=add_camera_adds,
            gripper_offsets=gripper_offsets,
        )


class EmptyWorldFR3Duo(SimEnvCreator):
    gripper_mesh_quaternion_offset: ClassVar[list[float]] = [0, 0, 0.7071068, 0.7071068]

    def config(self) -> SimEnvCreatorConfig:
        robot_cfg: SimRobotConfig[Literal[7]] = SimRobotConfig(
            tcp_offset=GRIPPER_OFFSETS[rcs.common.GripperType("Robotiq2F85")],
            robot_type=RobotType.FR3,
            attachment_site=rcs.ROBOTS[RobotType.FR3].attachment_site,
            kinematic_model_path=rcs.ROBOTS[RobotType.FR3].mjcf_model_path,
            joint_rotational_tolerance=0.05 * (np.pi / 180.0),
            seconds_between_callbacks=0.1,
            trajectory_trace=False,
            arm_collision_geoms=[
                "fr3_link0_collision",
                "fr3_link1_collision",
                "fr3_link2_collision",
                "fr3_link3_collision",
                "fr3_link4_collision",
                "fr3_link5_collision",
                "fr3_link6_collision",
                "fr3_link7_collision",
            ],
            joints=[
                "fr3_joint1",
                "fr3_joint2",
                "fr3_joint3",
                "fr3_joint4",
                "fr3_joint5",
                "fr3_joint6",
                "fr3_joint7",
            ],
            actuators=[
                "fr3_joint1",
                "fr3_joint2",
                "fr3_joint3",
                "fr3_joint4",
                "fr3_joint5",
                "fr3_joint6",
                "fr3_joint7",
            ],
            base="base",
            dof=rcs.ROBOTS[RobotType.FR3].dof,
            joint_limits=rcs.ROBOTS[RobotType.FR3].joint_limits,
            q_home=rcs.HOME_POSITIONS["FR3_DUO_LEFT"],
        )
        robot_cfg_right: SimRobotConfig[Literal[7]] = copy.deepcopy(robot_cfg)
        robot_cfg_right.q_home = rcs.HOME_POSITIONS["FR3_DUO_RIGHT"]

        robot_cfgs: dict[str, SimRobotConfig] = {"left": robot_cfg, "right": robot_cfg_right}
        sim_cfg: SimConfig = SimConfig(async_control=False, realtime=True, frequency=1, max_convergence_steps=500)

        control_mode: ControlMode = ControlMode.CARTESIAN_TQuat
        task_cfg = None
        scene: str = SCENE_PATHS["empty_world"]
        gripper_cfg = SimGripperConfig(
            epsilon_inner=0.005,
            epsilon_outer=0.005,
            seconds_between_callbacks=0.1,
            ignored_collision_geoms=[],
            collision_geoms=[],
            collision_geoms_fingers=[],
            joints=["right_driver_joint", "left_driver_joint"],
            max_joint_width=0.005,
            min_joint_width=1.0,
            actuator="fingers_actuator",
            max_actuator_width=0,
            min_actuator_width=255,
            gripper_type=GripperType("Robotiq2F85"),
        )

        gripper_cfg_right = copy.deepcopy(gripper_cfg)
        gripper_cfgs: dict[str, SimGripperConfig] = {"left": gripper_cfg, "right": gripper_cfg_right}

        camera_cfgs: dict[str, SimCameraConfig] | None = {
            "head": SimCameraConfig(
                identifier="head",
                type=CameraType.fixed,
                resolution_width=1280,
                resolution_height=720,
                frame_rate=30,
            ),
            "left_wrist": SimCameraConfig(
                identifier="left_wrist",
                type=CameraType.fixed,
                resolution_width=1280,
                resolution_height=720,
                frame_rate=30,
            ),
            "right_wrist": SimCameraConfig(
                identifier="right_wrist",
                type=CameraType.fixed,
                resolution_width=1280,
                resolution_height=720,
                frame_rate=30,
            ),
        }
        max_relative_movement: float | tuple[float, float] | None = None
        relative_to: RelativeTo = RelativeTo.LAST_STEP
        robot_to_shared_base_frame: dict[str, rcs.common.Pose] | None = {
            "left": DEFAULT_TRANSFORMS["FR3_DUOMOUNT_LEFT_ROBOT"],
            "right": DEFAULT_TRANSFORMS["FR3_DUOMOUNT_RIGHT_ROBOT"],
        }
        wrapper_cfg: WrapperConfig = WrapperConfig(binary_gripper=False, home_on_reset=True)
        headless = False
        add_gravcomp = True
        shared_base_frame_to_root_frame = DEFAULT_TRANSFORMS["FR3_DUOMOUNT_HEIGHT_OFFSET"]
        root_frame_to_world = rcs.common.Pose()
        alternative_combined_robot_mjcf: str | None = None
        world_frame_objects: dict[str, tuple[str, rcs.common.Pose]] | None = None
        root_frame_objects: dict[str, tuple[str, rcs.common.Pose]] | None = {
            "duo_mount": (OBJECT_PATHS["fr3_duo_mount"], DEFAULT_TRANSFORMS["FR3_DUOMOUNT_BASE"]),
            # "green_cube": (OBJECT_PATHS["green_cube"], Pose(translation=[0.5, 0, 0.5], quaternion=[0, 0, 0, 1])),
        }
        robot_frame_objects: dict[str, dict[str, tuple[str, rcs.common.Pose]]] | None = {
            "left": {
                "left_d405_mount": (
                    OBJECT_PATHS["robotiq_d405_mount"],
                    DEFAULT_TRANSFORMS["FR3_ROBOTIQ_WRIST_D405_MOUNT"],
                )
            },
            "right": {
                "right_d405_mount": (
                    OBJECT_PATHS["robotiq_d405_mount"],
                    DEFAULT_TRANSFORMS["FR3_ROBOTIQ_WRIST_D405_MOUNT"],
                )
            },
        }
        add_camera_adds: dict[str, CameraAdderConfig] | None = {
            "head": CameraAdderConfig(
                xml_path=CAMERA_PATHS["zed_mini"],
                offset=rcs.common.Pose(
                    # if duo_mount is spawned at [0, 0, 0.342], these are the offsets
                    DEFAULT_TRANSFORMS["FR3_DUOMOUNT_ZEDMINI_CAMERA"]
                ),
            ),
            "left_wrist": CameraAdderConfig(
                xml_path=CAMERA_PATHS["d405"],
                offset=rcs.common.Pose(DEFAULT_TRANSFORMS["FR3_ROBOTIQ_WRIST_D405_CAMERA"]),  # 20deg offset from normal
                robot_name="left",
            ),
            "right_wrist": CameraAdderConfig(
                xml_path=CAMERA_PATHS["d405"],
                offset=rcs.common.Pose(DEFAULT_TRANSFORMS["FR3_ROBOTIQ_WRIST_D405_CAMERA"]),  # 20deg offset from normal
                robot_name="right",
            ),
        }
        gripper_offset = rcs.common.Pose(
            quaternion=np.array(self.gripper_mesh_quaternion_offset), translation=np.array([0.0, 0.0, 0.0])
        )
        return SimEnvCreatorConfig(
            robot_cfgs=robot_cfgs,
            sim_cfg=sim_cfg,
            control_mode=control_mode,
            task_cfg=task_cfg,
            scene=scene,
            gripper_cfgs=gripper_cfgs,
            camera_cfgs=camera_cfgs,
            max_relative_movement=max_relative_movement,
            relative_to=relative_to,
            robot_to_shared_base_frame=robot_to_shared_base_frame,
            wrapper_cfg=wrapper_cfg,
            headless=headless,
            add_gravcomp=add_gravcomp,
            shared_base_frame_to_root_frame=shared_base_frame_to_root_frame,
            root_frame_to_world=root_frame_to_world,
            alternative_combined_robot_mjcf=alternative_combined_robot_mjcf,
            world_frame_objects=world_frame_objects,
            root_frame_objects=root_frame_objects,
            robot_frame_objects=robot_frame_objects,
            camera_adds=add_camera_adds,
            gripper_offsets={"left": gripper_offset, "right": gripper_offset},
        )


class EmptyWorldUR5e(EmptyWorldFR3):

    def config(self) -> SimEnvCreatorConfig:
        rt = RobotType("UR5e")
        cfg = super().config()
        lead_robot_name = self.lead_robot_name(cfg)

        robot_cfg = cfg.robot_cfgs[lead_robot_name]
        robot_cfg.tcp_offset = GRIPPER_OFFSETS[rcs.common.GripperType("Robotiq2F85")]
        robot_cfg.attachment_site = rcs.ROBOTS[rt].attachment_site
        robot_cfg.kinematic_model_path = rcs.ROBOTS[rt].mjcf_model_path
        robot_cfg.arm_collision_geoms = []
        robot_cfg.joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        robot_cfg.actuators = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
        robot_cfg.dof = rcs.ROBOTS[rt].dof
        robot_cfg.joint_limits = rcs.ROBOTS[rt].joint_limits
        robot_cfg.q_home = rcs.ROBOTS[rt].q_home
        robot_cfg.base = "base"

        assert cfg.gripper_cfgs is not None
        gripper_cfg = cfg.gripper_cfgs[lead_robot_name]

        gripper_cfg.actuator = "fingers_actuator"
        gripper_cfg.joints = ["right_driver_joint", "left_driver_joint"]
        gripper_cfg.collision_geoms = []
        gripper_cfg.collision_geoms_fingers = []
        gripper_cfg.max_actuator_width = 0
        gripper_cfg.min_actuator_width = 255
        gripper_cfg.max_joint_width = 0.005
        gripper_cfg.min_joint_width = 1.0
        gripper_cfg.gripper_type = GripperType("Robotiq2F85")

        cfg.camera_cfgs = None
        cfg.camera_adds = None
        cfg.gripper_offsets = None

        return cfg


class EmptyWorldXArm7(EmptyWorldFR3):

    def config(self) -> SimEnvCreatorConfig:
        rt = RobotType("XArm7")
        cfg = super().config()
        lead_robot_name = self.lead_robot_name(cfg)

        robot_cfg = cfg.robot_cfgs[lead_robot_name]
        robot_cfg.robot_type = rt
        robot_cfg.tcp_offset = rcs.common.Pose()
        robot_cfg.attachment_site = rcs.ROBOTS[rt].attachment_site
        robot_cfg.kinematic_model_path = rcs.ROBOTS[rt].mjcf_model_path
        robot_cfg.arm_collision_geoms = []
        robot_cfg.joints = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        robot_cfg.actuators = [
            "act1",
            "act2",
            "act3",
            "act4",
            "act5",
            "act6",
            "act7",
        ]
        robot_cfg.dof = rcs.ROBOTS[rt].dof
        robot_cfg.joint_limits = rcs.ROBOTS[rt].joint_limits
        robot_cfg.q_home = rcs.ROBOTS[rt].q_home
        robot_cfg.base = "base"

        cfg.gripper_cfgs = None
        cfg.camera_cfgs = None
        cfg.camera_adds = None
        cfg.gripper_offsets = None

        return cfg


class EmptyWorldSO101(EmptyWorldFR3):
    gripper_prefix_template = "robot"

    def config(self) -> SimEnvCreatorConfig:
        rt = RobotType("SO101")
        cfg = super().config()
        lead_robot_name = self.lead_robot_name(cfg)

        robot_cfg = cfg.robot_cfgs[lead_robot_name]
        robot_cfg.robot_type = rt
        robot_cfg.tcp_offset = rcs.common.Pose()
        robot_cfg.attachment_site = "gripper"
        robot_cfg.kinematic_model_path = rcs.ROBOTS[rt].mjcf_model_path
        robot_cfg.arm_collision_geoms = []
        robot_cfg.joints = ["1", "2", "3", "4", "5"]
        robot_cfg.actuators = ["1", "2", "3", "4", "5"]
        robot_cfg.dof = rcs.ROBOTS[rt].dof
        robot_cfg.joint_limits = rcs.ROBOTS[rt].joint_limits
        robot_cfg.q_home = rcs.ROBOTS[rt].q_home
        robot_cfg.base = "base"

        assert cfg.gripper_cfgs is not None
        gripper_cfg = cfg.gripper_cfgs[lead_robot_name]
        gripper_cfg.min_actuator_width = -0.17453292519943295
        gripper_cfg.max_actuator_width = 1.7453292519943295
        gripper_cfg.min_joint_width = -0.17453292519943295
        gripper_cfg.max_joint_width = 1.7453292519943295
        gripper_cfg.actuator = "6"
        gripper_cfg.joints = ["6"]
        gripper_cfg.collision_geoms = []
        gripper_cfg.collision_geoms_fingers = []
        gripper_cfg.gripper_type = GripperType("SO101")

        cfg.camera_cfgs = None
        cfg.camera_adds = None
        cfg.gripper_offsets = None

        return cfg


gym.register(id="rcs/fr3", entry_point=EmptyWorldFR3())
gym.register(id="rcs/duo", entry_point=EmptyWorldFR3Duo())
gym.register(id="rcs/ur5e", entry_point=EmptyWorldUR5e())
gym.register(id="rcs/xarm7", entry_point=EmptyWorldXArm7())
gym.register(id="rcs/so101", entry_point=EmptyWorldSO101())


if __name__ == "__main__":
    env = gym.make("rcs/duo")
    obs, info = env.reset()
    print(obs)
    # Duo
    for _ in range(100):
        for _ in range(10):
            # move 1cm in x direction (forward) and close gripper
            act = {
                "left": {"tquat": [0.01, 0, 0, 0, 0, 0, 1], "gripper": [0]},
                "right": {"tquat": [0.01, 0, 0, 0, 0, 0, 1], "gripper": [0]},
            }
            obs, reward, terminated, truncated, info = env.step(act)
            # print(obs)
            print(reward, terminated, truncated, info)
            time.sleep(1.0)
        for _ in range(10):
            # move 1cm in negative x direction (backward) and open gripper
            act = {
                "left": {"tquat": [-0.01, 0, 0, 0, 0, 0, 1], "gripper": [1]},
                "right": {"tquat": [-0.01, 0, 0, 0, 0, 0, 1], "gripper": [1]},
            }
            obs, reward, terminated, truncated, info = env.step(act)
            # print(obs)
            print(reward, terminated, truncated, info)
            time.sleep(1.0)
    # # Single arm
    # for _ in range(100):
    #     for _ in range(10):
    #         # move 1cm in x direction (forward) and close gripper
    #         act = {"robot": {"tquat": [0.01, 0, 0, 0, 0, 0, 1], "gripper": [0]}}
    #         obs, reward, terminated, truncated, info = env.step(act)
    #         print(obs)
    #         time.sleep(1.0)
    #     for _ in range(10):
    #         # move 1cm in negative x direction (backward) and open gripper
    #         act = {"robot": {"tquat": [-0.01, 0, 0, 0, 0, 0, 1], "gripper": [1]}}
    #         obs, reward, terminated, truncated, info = env.step(act)
    #         print(obs)
    #         time.sleep(1.0)
