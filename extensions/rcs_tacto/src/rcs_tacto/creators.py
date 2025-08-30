import logging
import typing

import numpy as np
import gymnasium as gym
from gymnasium.envs.registration import EnvCreator
from rcs_tacto.tacto_wrapper import TactoSimWrapper
from rcs._core.sim import CameraType
from rcs.camera.sim import SimCameraConfig
from rcs.envs.base import ControlMode
from rcs.envs.creators import SimTaskEnvCreator
from rcs.envs.utils import default_sim_robot_cfg
from rcs._core.common import Pose
from rcs.sim import SimGripperConfig
import rcs

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class FR3TactoSimplePickUpSimEnvCreator(EnvCreator):
    def __call__(  # type: ignore
        self,
        render_mode: str = "human",
        control_mode: ControlMode = ControlMode.CARTESIAN_TRPY,
        resolution: tuple[int, int] | None = None,
        frame_rate: int = 0,
        delta_actions: bool = True,
        cam_list: list[str] = [
            "wrist_0",
            "bird_eye_cam",
            "openvla_view",
            "right_side",
            "front",
            "left_side",
            "side_view",
        ],
        tacto_kwargs: dict[str, typing.Any] | None = None,
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
                    translation=np.array([0.0, 0.0, 0.15]), 
                    rotation=np.array([[0.707, 0.707, 0], [-0.707, 0.707, 0], [0, 0, 1]])
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
        random_pos_args = {"joint_name":"yellow-box-joint"}

        env = SimTaskEnvCreator()(robot_cfg,
                                  render_mode,
                                  control_mode,
                                  delta_actions,
                                  cameras,
                                  gripper_cfg=gripper_cfg,
                                  random_pos_args=random_pos_args,
                                  **kwargs)

        # Here, we feed some default values for the tacto wrapper
        # that aligns with what we have in the fr3_digit_simple_pick_up
        if tacto_kwargs is None:
            tacto_kwargs = {}
            tacto_kwargs["tacto_sites"] = ["left_tacto_pad_0", "right_tacto_pad_0"]
            tacto_kwargs["tacto_geoms"] = ["yellow_box_geom"]
            tacto_kwargs["tacto_fps"] = 60
            tacto_kwargs["enable_depth"] = True
            tacto_kwargs["visualize"] = True

        return TactoSimWrapper(env, **tacto_kwargs)
