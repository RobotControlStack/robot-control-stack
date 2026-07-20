from __future__ import annotations

import copy
from typing import Any

import gymnasium as gym
import numpy as np
from rcs._core.common import GripperType, Pose
from rcs._core.sim import SimCameraConfig, SimGripperConfig
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.configs import EmptyWorldFR3
from rcs_taxim.taxim_wrapper import TaximSimWrapper, _robotiq2f85_digit_model_path

import rcs

TAXIM_GRIPPER_TYPE = GripperType("Robotiq2F85Digit")
rcs.GRIPPER_PATHS[TAXIM_GRIPPER_TYPE] = str(_robotiq2f85_digit_model_path())

def _prefixed(name: str) -> str:
    return f"gripper{name}"


def _make_camera_cfgs(
    base_cfgs: dict[str, SimCameraConfig],
    cam_list: tuple[str, ...],
    resolution: tuple[int, int] | None,
    frame_rate: int,
) -> dict[str, SimCameraConfig]:
    camera_cfgs: dict[str, SimCameraConfig] = {}
    for cam in cam_list:
        if cam not in base_cfgs:
            available = ", ".join(sorted(base_cfgs))
            msg = f"Unknown camera {cam!r}. Available cameras: {available}"
            raise ValueError(msg)
        cfg = copy.deepcopy(base_cfgs[cam])
        if resolution is not None:
            cfg.resolution_width = resolution[0]
            cfg.resolution_height = resolution[1]
        if frame_rate > 0:
            cfg.frame_rate = frame_rate
        camera_cfgs[cam] = cfg
    return camera_cfgs


def taxim_gripper_cfg() -> SimGripperConfig:
    return SimGripperConfig(
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
        gripper_type=TAXIM_GRIPPER_TYPE,
    )


class FR3TaximSimplePickUpSimEnvCreator:
    def __call__(
        self,
        render_mode: str = "human",
        control_mode: ControlMode = ControlMode.CARTESIAN_TRPY,
        resolution: tuple[int, int] | None = None,
        frame_rate: int = 0,
        delta_actions: bool = True,
        cam_list: tuple[str, ...] | None = None,
        taxim_kwargs: dict[str, Any] | None = None,
        **kwargs: Any,
    ) -> gym.Env:
        binary_gripper = kwargs.pop("binary_gripper", True)
        home_on_reset = kwargs.pop("home_on_reset", True)
        max_relative_movement = kwargs.pop("max_relative_movement", None)
        if kwargs:
            unexpected = ", ".join(sorted(kwargs))
            msg = f"Unexpected keyword arguments: {unexpected}"
            raise TypeError(msg)

        scene = EmptyWorldFR3()
        cfg = scene.config()
        cfg.robot_cfgs["right"].tcp_offset = rcs.GRIPPER_TCP_OFFSETS[rcs.common.GripperType("Robotiq2F85")]
        cfg.control_mode = control_mode
        cfg.headless = render_mode != "human"
        cfg.sim_cfg.realtime = render_mode == "human"
        cfg.sim_cfg.async_control = render_mode == "human"
        cfg.sim_cfg.frequency = 30
        cfg.wrapper_cfg.binary_gripper = binary_gripper
        cfg.wrapper_cfg.home_on_reset = home_on_reset
        cfg.max_relative_movement = (
            max_relative_movement if max_relative_movement is not None else (0.2, np.deg2rad(45))
        )
        cfg.relative_to = RelativeTo.LAST_STEP if delta_actions else RelativeTo.NONE
        if not delta_actions:
            cfg.max_relative_movement = None
        cfg.gripper_cfgs = {"robot": taxim_gripper_cfg()}
        cfg.gripper_offsets = None
        cfg.root_frame_objects = {
            "": (
                rcs.OBJECT_PATHS["green_cube"],
                Pose(translation=np.array([0.5, 0.0, 0.05]), quaternion=np.array([0.0, 0.0, 0.0, 1.0])),
            )
        }

        cam_list = tuple(cam_list or ())
        if cam_list:
            cfg.camera_cfgs = _make_camera_cfgs(cfg.camera_cfgs or {}, cam_list, resolution, frame_rate)
            cfg.camera_adds = (
                {name: cfg.camera_adds[name] for name in cam_list} if cfg.camera_adds is not None else None
            )
        else:
            cfg.camera_cfgs = None
            cfg.camera_adds = None

        env = scene.create_env(cfg)
        merged_taxim_kwargs: dict[str, Any] = {
            "taxim_sites": [_prefixed("left_digit_pad"), _prefixed("right_digit_pad")],
            "taxim_pad_geoms": [_prefixed("left_digit_pad"), _prefixed("right_digit_pad")],
            "target_geom_mesh_dict": {"_box_geom": "_box_geom"},
            "taxim_sensor_type": "digit",
            "taxim_fps": 60,
            "enable_depth": True,
            "visualize": True,
        }
        if taxim_kwargs is not None:
            merged_taxim_kwargs.update(taxim_kwargs)
        env = TaximSimWrapper(env, **merged_taxim_kwargs)
        if render_mode == "human":
            env.get_wrapper_attr("sim").open_gui()
        return env
