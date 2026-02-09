from typing import Any
import gymnasium as gym
import rcs
import numpy as np

from rcs._core.sim import SimConfig
from rcs.envs.base import ControlMode, RelativeActionSpace
from rcs.envs.space_utils import ActObsInfoWrapper
from rcs.sim import SimGripperConfig
from rcs_tacto.tacto_wrapper import TactoSimWrapper
from gymnasium.envs.registration import EnvCreator


from rcs._core.sim import CameraType, SimCameraConfig
from rcs.envs.creators import SimTaskEnvCreator

SCENE_FILE = "beast_refiner/rcs_env/rcs_icra_scene_digit/scene.xml"
CAMERAS = [
    "wrist",
    "bird_eye",
    "side",
    "side_right",
    "side_wide",
]

class AlignSpaceWrapper(ActObsInfoWrapper):

    def __init__(self, env):
        super().__init__(env)
        self.observation_space = gym.spaces.Dict({
            "state": gym.spaces.Box(low=-np.inf, high=np.inf, shape=(15,), dtype=np.float32),
            "images.image": env.observation_space["frames"]["side"]["rgb"]["data"],
            "images.image2": env.observation_space["frames"]["wrist"]["rgb"]["data"],
            "images.tactile_left": gym.spaces.Box(low=0, high=255, shape=(320, 240, 3), dtype=np.uint8),
            "images.tactile_right": gym.spaces.Box(low=0, high=255, shape=(320, 240, 3), dtype=np.uint8),
        })
        # self.action_space = env.action_space["joints"]
        gym.spaces.Box(low=-np.inf, high=np.inf, shape=(8,), dtype=np.float32)

    def observation(self, observation: dict[str, Any], info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        observation = {
            "state": np.concat([observation["joints"], observation["gripper"], info["tau_ext"]]),
            "images.image": observation["frames"]["side"]["rgb"]["data"],
            "images.image2": observation["frames"]["wrist"]["rgb"]["data"],
            "images.tactile_left": observation["frames"]["tactile_left_tacto_pad_0"]["rgb"]["data"],
            "images.tactile_right": observation["frames"]["tactile_right_tacto_pad_0"]["rgb"]["data"],
        }
        # tactile and tau_ext are still unsolved

        return observation, info
        

    def action(self, action: np.ndarray) -> dict[str, Any]:
        action = {"joints": action[:7], "gripper": action[7]}
        return action


def get_robot_cfg():
    robot_cfg = rcs.sim.SimRobotConfig()
    robot_cfg.tcp_offset = rcs.common.Pose(
        translation=np.array([0.0, 0.0, 0.15]),
        rotation=np.array([[0.707, 0.707, 0], [-0.707, 0.707, 0], [0, 0, 1]]),
    )
    robot_cfg.robot_type = rcs.common.RobotType.FR3
    robot_cfg.add_id("0")
    robot_cfg.mjcf_scene_path = str(SCENE_FILE)
    robot_cfg.kinematic_model_path = rcs.scenes["fr3_empty_world"].mjcf_robot
    return robot_cfg


def make_single_env():
    robot_cfg = get_robot_cfg()
    random_pos_args = {
        "joint_name": "box_joint",
        "init_object_pose": rcs.common.Pose(
            translation=np.array([0.048, 0, 0.855]),
            quaternion=np.array([0.0, 0.0, 1, 0]),
        ),
        "include_rotation": True,
        "x_scale": 0.3,
        "y_scale": 0.4,
        "x_offset": -0.1,
        "y_offset": -0.2,
    }
    gripper_cfg = SimGripperConfig()
    gripper_cfg.collision_geoms = []
    gripper_cfg.collision_geoms_fingers = []
    gripper_cfg.add_id("0")

    sim_cfg = SimConfig()
    sim_cfg.realtime = False
    sim_cfg.async_control = True
    sim_cfg.frequency = 30  # in Hz

    resolution = (256, 256)
    cameras = {
        cam: SimCameraConfig(
            identifier=cam,
            type=CameraType.fixed,
            resolution_height=resolution[1],
            resolution_width=resolution[0],
            frame_rate=0,  # this means render on demand
        )
        for cam in CAMERAS
    }

    env_factory = SimTaskEnvCreator()
    env = env_factory(
        robot_cfg=robot_cfg,
        control_mode=ControlMode.JOINTS,
        render_mode="",  # put this to 'human' to open a gui to watch the simulation
        delta_actions=False,
        cameras=cameras,
        random_pos_args=random_pos_args,
        gripper_cfg=gripper_cfg,
        sim_cfg=sim_cfg,
    )
    env = RelativeActionSpace(env, max_mov=np.deg2rad(90))
    env = TactoSimWrapper(
        env,
        tacto_sites=["left_tacto_pad_0", "right_tacto_pad_0"],
        tacto_geoms=["box_geom"],
        tacto_fps=30,
        enable_depth=False,
        visualize=False,
    )
    env = AlignSpaceWrapper(env)
    return env


def make_env(n_envs: int = 1, use_async_envs: bool = False):
    """
    Create vectorized environments for your custom task.

    Args:
        n_envs: Number of parallel environments
        use_async_envs: Whether to use AsyncVectorEnv or SyncVectorEnv

    Returns:
        gym.vector.VectorEnv or dict mapping suite names to vectorized envs
    """

    # Choose vector environment type
    env_cls = gym.vector.AsyncVectorEnv if use_async_envs else gym.vector.SyncVectorEnv

    # Create vectorized environment
    vec_env = env_cls([make_single_env for _ in range(n_envs)])

    return vec_env


class Pick(EnvCreator):
    def __call__(self) -> gym.Env:
        return make_single_env()