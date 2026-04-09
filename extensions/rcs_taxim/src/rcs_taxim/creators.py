import logging
import typing

import gymnasium as gym
import numpy as np
from gymnasium.envs.registration import EnvCreator
from rcs._core.common import Pose
from rcs._core.sim import CameraType
from rcs.camera.sim import SimCameraConfig
from rcs.envs.base import ControlMode
from rcs.envs.creators import SimTaskEnvCreator
from rcs.envs.utils import default_sim_robot_cfg
from rcs.sim import SimGripperConfig
from rcs_taxim.taxim_wrapper import TaximSimWrapper

import rcs

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class FR3TaximSimplePickUpSimEnvCreator(EnvCreator):
    def __call__(  # type: ignore
        self,
        render_mode: str = "human",
        control_mode: ControlMode = ControlMode.CARTESIAN_TRPY,
        resolution: tuple[int, int] | None = None,
        frame_rate: int = 0,
        delta_actions: bool = True,
        cam_list: tuple[str, ...] = (
            "wrist_0",
            "bird_eye_cam",
            "openvla_view",
            "right_side",
            "front",
            "left_side",
            "side_view",
        ),
        taxim_kwargs: dict[str, typing.Any] | None = None,
        **kwargs,
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
        robot_cfg = default_sim_robot_cfg(scene="fr3_digit_simple_pick_up")  # id = 0 by default
        # TODO: Figure out why feeding it the default doesn't work.
        #       Probably because Pinocchio freaks out over all the weird tags?
        robot_cfg.kinematic_model_path = rcs.scenes["fr3_empty_world"].mjcf_robot
        robot_cfg.tcp_offset = Pose(
            translation=np.array([0.0, 0.0, 0.15]),  # type: ignore
            rotation=np.array([[0.707, 0.707, 0], [-0.707, 0.707, 0], [0, 0, 1]]),  # type: ignore
        )
        gripper_cfg = SimGripperConfig()

        # the digit gripper has some custom finger collisions
        # not seen in the defaults. These need to be configured properly.
        gripper_cfg.collision_geoms = [
            "hand_c",
            "d435i_collision",
            "finger_a_left",
            "finger_b_left",
            "finger_c_left",
            "finger_a_right",
            "finger_b_right",
            "finger_c_right",
        ]
        gripper_cfg.collision_geoms_fingers = [
            "finger_a_left",
            "finger_b_left",
            "finger_c_left",
            "finger_a_right",
            "finger_b_right",
            "finger_c_right",
        ]

        # Append the id to keep it consistent with the model
        gripper_cfg.add_id("0")
        random_pos_args = {"joint_name": "box_joint"}

        env = SimTaskEnvCreator()(
            robot_cfg,
            render_mode,
            control_mode,
            delta_actions,
            cameras,
            gripper_cfg=gripper_cfg,
            random_pos_args=random_pos_args,
            **kwargs,
        )

        '''
        env: gym.Env,
        taxim_sites: list[str],
        taxim_pad_geoms: list[str],
        target_geom_mesh_dict: dict[str, str],
        taxim_sensor_type: str = "digit",
        taxim_bg_idx: int = 0,
        taxim_bg_randomize: bool = False,
        enable_depth: bool = False,
        taxim_fps: int = 60,
        visualize: bool = False,
        '''
        # Here, we feed some default values for the taxim wrapper
        # that aligns with what we have in the fr3_digit_simple_pick_up
        if taxim_kwargs is None:
            taxim_kwargs = {}
            taxim_kwargs["taxim_sites"] = ["left_taxim_pad_0", "right_taxim_pad_0"]
            taxim_kwargs["taxim_pad_geoms"] = ["finger_1_left_0", "finger_1_right_0"]
            taxim_kwargs["target_geom_mesh_dict"] = {"box_geom": "box_geom"}
            taxim_kwargs["taxim_sensor_type"] = "digit"
            taxim_kwargs["taxim_fps"] = 60
            taxim_kwargs["enable_depth"] = True
            taxim_kwargs["visualize"] = True

        return TaximSimWrapper(env, **taxim_kwargs)
