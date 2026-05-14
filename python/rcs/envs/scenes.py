import copy
import typing
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from os import PathLike

import gymnasium as gym
from gymnasium.envs.registration import EnvCreator
from rcs._core.sim import SimCameraConfig, SimConfig, SimGripperConfig, SimRobotConfig
from rcs.camera.interface import BaseCameraSet
from rcs.camera.sim import SimCameraSet
from rcs.envs.base import (
    CameraSetWrapper,
    ControlMode,
    CoverWrapper,
    GripperWrapper,
    MultiRobotWrapper,
    RelativeActionSpace,
    RelativeTo,
    RobotWrapper,
    SimEnv,
)
from rcs.envs.sim import GripperWrapperSim, RobotSimWrapper
from rcs.sim.composer import ModelComposer
from rcs.sim.sim import Sim

import rcs
from rcs import GRIPPER_PATHS, SCENE_PATHS, TASKS

RCSEnvCreatorConfig = typing.TypeVar("RCSEnvCreatorConfig")


class RCSEnvCreator(ABC, EnvCreator, typing.Generic[RCSEnvCreatorConfig]):

    @abstractmethod
    def create_env(self, cfg: RCSEnvCreatorConfig) -> gym.Env:
        raise NotImplementedError

    @abstractmethod
    def config(self) -> RCSEnvCreatorConfig:
        raise NotImplementedError

    def __call__(self, **kwargs) -> gym.Env:
        cfg: RCSEnvCreatorConfig = kwargs.get("cfg", self.config())
        return self.create_env(cfg)


@dataclass(kw_only=True)
class WrapperConfig:
    binary_gripper: bool = True
    home_on_reset: bool = True
    include_depth: bool = False


#### SIM SPECIFIC ####


@dataclass(kw_only=True)
class BaseTaskConfig:
    task_id: str
    # root_frame_to_world: rcs.common.Pose = field(default_factory=rcs.common.Pose)


TaskConfig = typing.TypeVar("TaskConfig", bound=BaseTaskConfig)


class Task(typing.Generic[TaskConfig]):

    @staticmethod
    def add_task_mujoco(cfg: TaskConfig, composer: ModelComposer, env_cfg: "SimEnvCreatorConfig"):
        """Add task-specific elements to the Mujoco scene."""

    @staticmethod
    def add_task_env(_cfg: TaskConfig, env: gym.Env, _simulation: Sim, _env_cfg: "SimEnvCreatorConfig") -> gym.Env:
        """Add task-specific wrappers to the environment."""
        return env


@dataclass(kw_only=True)
class CameraAdderConfig:
    xml_path: str | None = None
    """path to where the camera xml is, should have world frame, geoms and one camera defined in it, 
    if None, a camera will be added according to the camera_cfgs provided in the scene config and the other parameters in this config"""
    fovy: float = 60.0
    """only used if no xml_path is provided and a camera is added to the model"""
    offset: rcs.common.Pose = field(default_factory=rcs.common.Pose)
    attachment_site: str = "attachment_site"
    """only used when a robot name to attach the camera to is provided"""
    robot_name: str | None = None


@dataclass(kw_only=True)
class SimEnvCreatorConfig(typing.Generic[TaskConfig]):
    robot_cfgs: dict[str, SimRobotConfig]
    sim_cfg: SimConfig
    control_mode: ControlMode
    task_cfg: TaskConfig | None = None
    scene: str = SCENE_PATHS["empty_world"]
    """path or key to load the mujoco scene, e.g. from SCENE_PATHS, will be passed to load_scene()"""
    gripper_cfgs: dict[str, SimGripperConfig] | None = None
    camera_cfgs: dict[str, SimCameraConfig] | None = None
    max_relative_movement: float | tuple[float, float] | None = None
    relative_to: RelativeTo = RelativeTo.LAST_STEP
    robot_to_shared_base_frame: dict[str, rcs.common.Pose] | None = None
    """shared base frame is a common reference frame for all robots in the scene and the origin for all actions and observations, e.g. the middle of franka duo
    thus this transformation defines the offset of each robot's base to this shared base frame."""
    add_gravcomp: bool = False
    wrapper_cfg: WrapperConfig = field(default_factory=WrapperConfig)
    headless: bool = False
    shared_base_frame_to_root_frame: rcs.common.Pose = field(default_factory=rcs.common.Pose)
    """shared base frame is a common reference frame for all robots in the scene and the origin for all actions and observations, e.g. the middle of franka duo
    root_frame defines the origin where the parent of the robot assets are placed"""
    root_frame_to_world: rcs.common.Pose = field(default_factory=rcs.common.Pose)
    """root_frame defines the origin where the parent of the robot assets are placed
    world frame is the mujoco world frame"""
    alternative_combined_robot_mjcf: str | None = None
    """If you dont want to compose your scene with ModelComposer API,
    you can directly provide a combined robot mjcf with correct prefixing and the composer will add it as is without modifications.
    Prefixes need to as follows: robot{robot_name} where robot_name is the key in robot_cfgs, e.g. robot0, robot1, etc.
    root_frame_to_world will be used to place the robot in the world and
    shared_base_frame_to_root_frame will be used to determine the origin of the robot's action space."""
    world_frame_objects: dict[str, tuple[str, rcs.common.Pose]] | None = None
    """dict of object_id to tuple of (object_xml, object2world), will be added to the scene, object2world is the pose of the object in the mujoco world frame"""
    root_frame_objects: dict[str, tuple[str, rcs.common.Pose]] | None = None
    """dict of object_id to tuple of (object_xml, object2root_frame), will be added to the robot object2root_frame is the pose of the object in the root frame of the robot,
    which is defined by shared_base_frame_to_root_frame, object_id must be unique across all objects and robots in the scene"""
    robot_frame_objects: dict[str, dict[str, tuple[str, rcs.common.Pose]]] | None = None
    """dict of robot_name to dict of object_id to tuple of (object_xml, object2robot_frame), objects will be attached to that robot's attachment site,
    object2robot_frame is the pose of the object in the robot attachment-site frame, object_id must be unique across all objects and robots in the scene"""
    camera_adds: dict[str, CameraAdderConfig] | None = None
    """dict of camera_name to CameraAdderConfig, cameras will be added to the scene according to the config, camera_name must be unique across all cameras in the scene"""
    gripper_offsets: dict[str, rcs.common.Pose] | None = None
    """optional offsets for the gripper from the robot's attachment site"""
    _original_cfg: typing.Any = None
    """this will hold the original config in case this config has been prefixed"""


# MjScene = typing.TypeVar("MjScene", ModelComposer, str, Path)
MjModel = ModelComposer | str | PathLike


class SimEnvCreator(RCSEnvCreator[SimEnvCreatorConfig], typing.Generic[TaskConfig]):
    robot_prefix_template: str = "robot{robot_name}_"
    gripper_prefix_template: str = "gripper{robot_name}_"

    def __call__(self, **kwargs) -> gym.Env:
        cfg: SimEnvCreatorConfig = kwargs.get("cfg", self.config())
        task_cfg = kwargs.get("task_cfg", cfg.task_cfg)
        cfg.task_cfg = task_cfg
        return self.create_env(cfg)

    def is_prefixed(self, cfg: SimEnvCreatorConfig) -> bool:
        return cfg._original_cfg is not None

    def prefixed_cfg(self, cfg: SimEnvCreatorConfig) -> SimEnvCreatorConfig:
        """Adds prefixing to geom, joint, actuators names etc"""
        if cfg._original_cfg is not None:
            return cfg
        prefixed_cfg = copy.deepcopy(cfg)
        prefixed_cfg._original_cfg = cfg
        for robot_name in self.robot_names(prefixed_cfg):
            prefixed_cfg.robot_cfgs[robot_name].add_prefix(self.robot_prefix_template.format(robot_name=robot_name))
            if prefixed_cfg.gripper_cfgs is not None:
                prefixed_cfg.gripper_cfgs[robot_name].add_prefix(
                    self.gripper_prefix_template.format(robot_name=robot_name)
                )
        return prefixed_cfg

    def kinematics_cfg(self, cfg: SimEnvCreatorConfig) -> dict[str, tuple[str, str]]:
        """
        Returns the kinematic configuration for each robot in the scene.
        Returns:
            dict[str, tuple[str, str]]: A dictionary mapping robot names to a tuple of (kinematic_model_path, attachment_site).
        """
        o_cfg = cfg if cfg._original_cfg is None else cfg._original_cfg

        return {
            robot_name: (rcfg.kinematic_model_path, rcfg.attachment_site)
            for robot_name, rcfg in o_cfg.robot_cfgs.items()
        }

    def robot_names(self, cfg: SimEnvCreatorConfig) -> list[str]:
        return list(cfg.robot_cfgs)

    def lead_robot_name(self, cfg: SimEnvCreatorConfig) -> str:
        return next(iter(cfg.robot_cfgs))

    def create_env(self, cfg: SimEnvCreatorConfig) -> gym.Env:
        mjmodel = self.create_model(cfg)
        return self.create_env_from_model(cfg, mjmodel)

    def create_model(self, cfg: SimEnvCreatorConfig) -> MjModel:
        """Loads the mujoco scene from the given config

        Returns:
            MjModel: path to scene file (mjcf or mjb), or composer object
        """
        # ensure unprefixed config
        if cfg._original_cfg is not None:
            cfg = cfg._original_cfg
        composer = ModelComposer(
            model_name="RCS Scene",
            add_gravcomp=cfg.add_gravcomp,
        )
        composer.load_base_scene(cfg.scene)

        self.add_task_mujoco(cfg.task_cfg, composer, cfg)

        if cfg.alternative_combined_robot_mjcf is not None:
            # robot is in one mjcf
            self.add_robot_mujoco(
                composer,
                robot_name=self.lead_robot_name(cfg),
                robot_xml=cfg.alternative_combined_robot_mjcf,
                robot2world=cfg.root_frame_to_world,
            )
        else:
            # robot is composed by composer
            for robot_name in self.robot_names(cfg):
                robot_to_shared_frame = (
                    cfg.robot_to_shared_base_frame[robot_name]
                    if cfg.robot_to_shared_base_frame is not None
                    else rcs.common.Pose()
                )
                # robot2world = robot_to_shared_frame * cfg.shared_base_frame_to_root_frame * cfg.root_frame_to_world
                robot2world = cfg.root_frame_to_world * cfg.shared_base_frame_to_root_frame * robot_to_shared_frame
                self.add_robot_mujoco(
                    composer, robot_name, cfg.robot_cfgs[robot_name].kinematic_model_path, robot2world
                )

        if cfg.gripper_cfgs is not None:
            # add gripper to each robot
            for robot_name in self.robot_names(cfg):
                gripper_xml = GRIPPER_PATHS.get(cfg.gripper_cfgs[robot_name].gripper_type)
                if gripper_xml is None:
                    continue
                self.add_gripper_mujoco(
                    cfg,
                    composer,
                    robot_name,
                    gripper_xml,
                    cfg.robot_cfgs[robot_name].attachment_site,
                )

        # add robot-specific objects
        if cfg.root_frame_objects is not None:
            for object_id, (object_xml, object2root_frame) in cfg.root_frame_objects.items():
                object2world = cfg.root_frame_to_world * object2root_frame
                self.add_object_mujoco(
                    composer,
                    object_id,
                    object_xml,
                    object2world,
                    register_root_relative_replay_free_joints=True,
                )
        # add external objects
        if cfg.world_frame_objects is not None:
            for object_id, (object_xml, object2world) in cfg.world_frame_objects.items():
                self.add_object_mujoco(composer, object_id, object_xml, object2world)
        # add robot-frame objects
        if cfg.robot_frame_objects is not None:
            for robot_name, robot_objects in cfg.robot_frame_objects.items():
                attachment_site = cfg.robot_cfgs[robot_name].attachment_site
                for object_id, (object_xml, object2robot_frame) in robot_objects.items():
                    self.add_object_robot_frame_mujoco(
                        composer,
                        robot_name=robot_name,
                        object_id=object_id,
                        object_xml=object_xml,
                        object2robot_frame=object2robot_frame,
                        attachment_site=attachment_site,
                    )

        # camera adds
        if cfg.camera_adds is not None:
            for camera_name, camera_add_cfg in cfg.camera_adds.items():
                camera_pose = (
                    cfg.root_frame_to_world * camera_add_cfg.offset
                    if camera_add_cfg.robot_name is None
                    else camera_add_cfg.offset
                )
                if camera_add_cfg.xml_path is not None:
                    composer.add_camera_xml(
                        xml_path=camera_add_cfg.xml_path,
                        name=camera_name,
                        pose=camera_pose,
                        robot_prefix=(
                            self.robot_prefix_template.format(robot_name=camera_add_cfg.robot_name)
                            if camera_add_cfg.robot_name is not None
                            else None
                        ),
                        attachment_site_name=(
                            cfg.robot_cfgs[camera_add_cfg.robot_name].attachment_site
                            if camera_add_cfg.robot_name is not None
                            else camera_add_cfg.attachment_site
                        ),
                    )
                    continue

                assert cfg.camera_cfgs is not None, "Camera configs must be provided to add cameras."
                assert (
                    camera_name in cfg.camera_cfgs
                ), f"Camera config for camera {camera_name} must be provided to add camera {camera_name} to the scene"
                composer.add_camera(
                    resolution=(
                        cfg.camera_cfgs[camera_name].resolution_width,
                        cfg.camera_cfgs[camera_name].resolution_height,
                    ),
                    fovy=camera_add_cfg.fovy,
                    name=camera_name,
                    pose=camera_pose,
                    robot_prefix=(
                        self.robot_prefix_template.format(robot_name=camera_add_cfg.robot_name)
                        if camera_add_cfg.robot_name is not None
                        else None
                    ),
                    attachment_site_name=(
                        cfg.robot_cfgs[camera_add_cfg.robot_name].attachment_site
                        if camera_add_cfg.robot_name is not None
                        else camera_add_cfg.attachment_site
                    ),
                )

        return composer

    def create_env_from_model(self, cfg: SimEnvCreatorConfig, mjmodel: MjModel) -> gym.Env:
        # save the composed scene for debugging
        # mjmodel.save_mjcf("scene.xml")
        # you can also apply a scene path e.g. the saved one
        # mjmodel = "scene.xml"

        # ensure we have prefixed and original config
        if cfg._original_cfg is not None:
            prefixed_cfg = cfg
            cfg = cfg._original_cfg
        else:
            prefixed_cfg = self.prefixed_cfg(cfg)

        simulation = Sim(mjmodel, prefixed_cfg.sim_cfg)
        if isinstance(mjmodel, ModelComposer):
            simulation.configure_state_encodings(
                root_frame_to_world=cfg.root_frame_to_world,
                root_relative_free_joints=mjmodel.root_relative_replay_free_joints,
            )

        envs: dict[str, gym.Env] = {}
        env: gym.Env
        kinematics_cfg = self.kinematics_cfg(cfg)
        for robot_name in self.robot_names(cfg):
            env = SimEnv(simulation)
            kinematic_model_path, attachment_site = kinematics_cfg[robot_name]
            ik = rcs.common.Pin(
                kinematic_model_path,
                attachment_site,
            )
            # ik = rcs_robotics_library._core.rl.RoboticsLibraryIK(cfg.robot_cfgs[lead_robot_name].kinematic_model_path)

            env = self.add_robot_env(prefixed_cfg, robot_name, env, simulation, ik)
            if prefixed_cfg.gripper_cfgs is not None:
                env = self.add_gripper_env(prefixed_cfg, robot_name, simulation, env)

            if prefixed_cfg.relative_to != RelativeTo.NONE:
                env = RelativeActionSpace(
                    env, max_mov=prefixed_cfg.max_relative_movement, relative_to=prefixed_cfg.relative_to
                )
            envs[robot_name] = env

        env = MultiRobotWrapper(envs, prefixed_cfg.robot_to_shared_base_frame)
        if prefixed_cfg.camera_cfgs is not None:
            camera_set = typing.cast(
                BaseCameraSet,
                SimCameraSet(simulation, prefixed_cfg.camera_cfgs, physical_units=True, render_on_demand=True),
            )
            env = CameraSetWrapper(env, camera_set, include_depth=cfg.wrapper_cfg.include_depth)
        env = self.add_task_env(prefixed_cfg.task_cfg, env, simulation, cfg)
        if not prefixed_cfg.headless:
            env.get_wrapper_attr("sim").open_gui()
        return CoverWrapper(env)

    def add_task_mujoco(self, task_cfg: TaskConfig | None, composer: ModelComposer, cfg: SimEnvCreatorConfig):
        """Add task-specific elements to the Mujoco scene."""
        if task_cfg is not None:
            TASKS[task_cfg.task_id].add_task_mujoco(task_cfg, composer, cfg)

    def add_task_env(
        self, task_cfg: TaskConfig | None, env: gym.Env, simulation: Sim, cfg: SimEnvCreatorConfig
    ) -> gym.Env:
        """Add task-specific wrappers to the environment."""
        if task_cfg is not None:
            return TASKS[task_cfg.task_id].add_task_env(task_cfg, env, simulation, cfg)
        return env

    def add_object_mujoco(
        self,
        composer: ModelComposer,
        object_id: str,
        object_xml: str,
        object2world: rcs.common.Pose,
        *,
        register_root_relative_replay_free_joints: bool = False,
    ):
        """Add an object to the Mujoco scene."""
        composer.add_object_world_frame(
            object_xml,
            object_prefix=object_id + "_",
            pose=object2world,
            register_root_relative_replay_free_joints=register_root_relative_replay_free_joints,
        )

    def add_object_robot_frame_mujoco(
        self,
        composer: ModelComposer,
        robot_name: str,
        object_id: str,
        object_xml: str,
        object2robot_frame: rcs.common.Pose,
        attachment_site: str,
    ):
        """Add an object to the Mujoco scene in a robot attachment-site frame."""
        composer.add_object_robot_frame(
            xml_path=object_xml,
            robot_prefix=self.robot_prefix_template.format(robot_name=robot_name),
            object_prefix=object_id + "_",
            attachment_site_name=attachment_site,
            pose=object2robot_frame,
        )

    def add_robot_mujoco(
        self,
        composer: ModelComposer,
        robot_name: str,
        robot_xml: str,
        robot2world: rcs.common.Pose | None = None,
    ):
        if robot2world is None:
            robot2world = rcs.common.Pose()
        robot_prefix = self.robot_prefix_template.format(robot_name=robot_name)
        composer.add_robot(
            robot_xml,
            robot_prefix,
            pose=robot2world,
        )

    def add_robot_env(
        self,
        prefixed_cfg: SimEnvCreatorConfig,
        robot_name: str,
        env: gym.Env,
        simulation: Sim,
        ik: rcs.common.Kinematics,
    ):
        # rcs wrapper composition
        robot = rcs.sim.SimRobot(sim=simulation, ik=ik, cfg=prefixed_cfg.robot_cfgs[robot_name])
        env = RobotWrapper(env, robot, prefixed_cfg.control_mode, home_on_reset=prefixed_cfg.wrapper_cfg.home_on_reset)
        return RobotSimWrapper(env)

    def add_gripper_mujoco(
        self, cfg: SimEnvCreatorConfig, composer: ModelComposer, robot_name: str, gripper_xml: str, attachment_site: str
    ):
        # mujoco scene composition
        assert cfg.gripper_cfgs is not None, "Gripper configs must be provided to add grippers."
        gripper_offset = (
            cfg.gripper_offsets[robot_name]
            if cfg.gripper_offsets is not None and robot_name in cfg.gripper_offsets
            else rcs.common.Pose()
        )
        composer.add_gripper(
            xml_path=gripper_xml,
            gripper_prefix=self.gripper_prefix_template.format(robot_name=robot_name),
            robot_prefix=self.robot_prefix_template.format(robot_name=robot_name),
            attachment_site_name=attachment_site,
            pose=gripper_offset,
        )

    def add_gripper_env(self, prefixed_cfg: SimEnvCreatorConfig, robot_name: str, simulation: Sim, env: gym.Env):
        # rcs wrapper composition
        assert prefixed_cfg.gripper_cfgs is not None, "Gripper configs must be provided to add grippers."
        gripper = rcs.sim.SimGripper(simulation, prefixed_cfg.gripper_cfgs[robot_name])
        env = GripperWrapper(env, gripper, binary=prefixed_cfg.wrapper_cfg.binary_gripper)
        return GripperWrapperSim(env)
