import datetime
import logging
from dataclasses import dataclass
from typing import Any

import gymnasium as gym
import numpy as np
from rcs._core.common import BaseCameraConfig, RobotPlatform
from rcs._core.sim import SimConfig
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.configs import EmptyWorldFR3Duo
from rcs.envs.tasks import PickTaskConfig

import rcs
from vlagents.client import RemoteAgent
from vlagents.policies import Act, Obs

logger = logging.getLogger(__name__)


ROBOT2IP = {
    "right": "192.168.102.1",
    "left": "192.168.101.1",
}
ROBOT2ID = {
    "left": "1",
    "right": "0",
}


ROBOT_INSTANCE = RobotPlatform.SIMULATION
# ROBOT_INSTANCE = RobotPlatform.HARDWARE

# set camera dict to none disable cameras
# CAMERA_DICT = {
#     "left_wrist": "230422272017",
#     "right_wrist": "230422271040",
#     "side": "243522070385",
#     "bird_eye": "243522070364",
# }
CAMERA_DICT = None
ZED_CAMERA_DICT = {
    "zed": "19928076",
}
INCLUDE_DEPTH = False


# DIGIT_DICT = {
#     "digit_right_left": "D21182",
#     "digit_right_right": "D21193"
# }
DIGIT_DICT = None


DATASET_PATH = "test_iris"
INSTRUCTION = "pick up cube"
RECORD_FPS = 30
CONTROL_MODE = ControlMode.JOINTS
RELATIVETO = RelativeTo.NONE
DEBUG = True
VIDEO_PATH = "videos"
MODEL = "pi05"
IP = ""
PORT = 20997


robot2world = {
    "right": rcs.common.Pose(
        translation=np.array([0, 0, 0]), rpy_vector=np.array([0.89360858, -0.17453293, 0.46425758])
    ),
    "left": rcs.common.Pose(
        translation=np.array([0, 0, 0]), rpy_vector=np.array([-0.89360858, -0.17453293, -0.46425758])
    ),
}


@dataclass
class InferenceConfig:
    vlagents_host: str
    vlagents_port: int
    vlagents_model: str
    instruction: str
    robot_keys: list[str]
    jpeg_encoding: bool = True
    on_same_machine: bool = False


class ModelInference:
    def __init__(self, env: gym.Env, cfg: InferenceConfig):
        self.env = env
        self.gripper_state = 1
        self._cfg = cfg
        self.remote_agent = RemoteAgent(
            cfg.vlagents_host, cfg.vlagents_port, cfg.vlagents_model, cfg.on_same_machine, cfg.jpeg_encoding
        )

    def obs_rcs2agents(self, obs: dict, info: dict | None = None) -> Obs:
        cameras = {}
        for frame in obs["frames"]:
            cameras[frame] = obs["frames"][frame]["rgb"]["data"]
        state = []
        for robot in self._cfg.robot_keys:
            # TODO: currently hardcoded for joints
            state.append(obs[robot]["joints"])
            state.append(obs[robot]["gripper"])

        return Obs(cameras=None, gripper=None, info=info, state=np.concatenate(state))

    def action_agents2rcs(self, action: Act) -> dict[str, Any]:
        act = {}
        for idx, robot in enumerate(self._cfg.robot_keys):
            # TODO: this is currently hard coded for franka joints
            act[robot] = {}
            act[robot]["joints"] = action.action[idx * 8 : idx * 8 + 7]
            act[robot]["gripper"] = action.action[idx * 8 + 7]
        return act

    def loop(self):
        obs, _ = self.env.reset()

        obs_dict = self.obs_rcs2agents(obs)

        self.remote_agent.reset(obs_dict, instruction=self._cfg.instruction)

        while True:
            action = self.remote_agent.act(obs_dict)
            if action.done:
                logger.info("done issued by agent, shutting down")
                break
            obs, _, _, _, info = self.env.step(self.action_agents2rcs(action))

            obs_dict = self.obs_rcs2agents(obs)


def get_env():
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        from rcs_fr3.configs import FrankaDuoEnv
        from rcs_fr3.creators import HardwareCameraCreatorConfig

        env_creator = FrankaDuoEnv()
        env_creator.left_ip = ROBOT2IP["left"]
        env_creator.right_ip = ROBOT2IP["right"]
        hw_cfg = env_creator.config()
        camera_cfgs: dict[str, HardwareCameraCreatorConfig] = {}
        if CAMERA_DICT is not None:
            camera_cfgs["realsense"] = HardwareCameraCreatorConfig(
                camera_type_id="realsense",
                camera_cfgs={
                    name: BaseCameraConfig(
                        identifier=identifier,
                        resolution_width=1280,
                        resolution_height=720,
                        frame_rate=30,
                    )
                    for name, identifier in CAMERA_DICT.items()
                },
            )
        if ZED_CAMERA_DICT is not None:
            camera_cfgs["zed"] = HardwareCameraCreatorConfig(
                camera_type_id="zed",
                camera_cfgs={
                    name: BaseCameraConfig(
                        identifier=identifier,
                        resolution_width=1280,
                        resolution_height=720,
                        frame_rate=30,
                    )
                    for name, identifier in ZED_CAMERA_DICT.items()
                },
                kwargs={
                    "enable_depth": False,
                    "enable_imu": False,
                },
            )
        if DIGIT_DICT is not None:
            camera_cfgs["digit"] = HardwareCameraCreatorConfig(
                camera_type_id="digit",
                camera_cfgs={
                    name: BaseCameraConfig(
                        identifier=identifier,
                        resolution_width=320,
                        resolution_height=240,
                        frame_rate=30,
                    )
                    for name, identifier in DIGIT_DICT.items()
                },
            )
        hw_cfg.camera_cfgs = camera_cfgs or None
        hw_cfg.control_mode = CONTROL_MODE
        hw_cfg.wrapper_cfg.include_depth = INCLUDE_DEPTH
        hw_cfg.max_relative_movement = 0.5 if CONTROL_MODE == ControlMode.JOINTS else (0.5, np.deg2rad(90))
        hw_cfg.relative_to = RELATIVETO
        hw_cfg.robot_to_shared_base_frame = robot2world
        env_rel = env_creator.create_env(hw_cfg)
    else:
        # FR3

        scene = EmptyWorldFR3Duo()
        sim_cfg_data = scene.config()
        sim_cfg_data.sim_cfg = SimConfig(
            async_control=True, realtime=True, frequency=RECORD_FPS, max_convergence_steps=500
        )
        sim_cfg_data.relative_to = RelativeTo.CONFIGURED_ORIGIN
        sim_cfg_data.wrapper_cfg.include_depth = INCLUDE_DEPTH
        if sim_cfg_data.root_frame_objects is None:
            sim_cfg_data.root_frame_objects = {}
        sim_cfg_data.task_cfg = PickTaskConfig(robot_name="right")

        env_rel = scene.create_env(sim_cfg_data)

    return env_rel


def main():
    env_rel = get_env()
    env_rel.reset()

    VIDEO_PATH.mkdir(parents=True, exist_ok=True)
    timestamp = str(datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

    camera_set = env_rel.get_wrapper_attr("camera_set")
    camera_set.record_video(VIDEO_PATH, timestamp)

    # env = RHCWrapper(env, exec_horizon=1)

    cfg = InferenceConfig(
        IP, PORT, MODEL, INSTRUCTION, jpeg_encoding=True, on_same_machine=True, robot_keys=["left", "right"]
    )
    controller = ModelInference(env_rel, cfg)
    input("robot is about to be controlled by AI, press enter to start")
    with env_rel:
        controller.loop()


if __name__ == "__main__":
    main()
