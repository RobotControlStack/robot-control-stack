import logging
from dataclasses import dataclass
from time import sleep

import cv2
import gymnasium as gym
import json_numpy
import matplotlib.pyplot as plt
import numpy as np
import requests
from agents.client import RemoteAgent
from agents.policies import Act, Obs
from omegaconf import OmegaConf
from PIL import Image
from rcsss._core.sim import FR3, FR3State
from rcsss.camera.realsense import RealSenseCameraSet
from rcsss.config import read_config_yaml

# from rcsss.desk import FCI, Desk
from rcsss.control.fr3_desk import FCI, Desk, DummyResourceManager
from rcsss.control.utils import load_creds_fr3_desk
from rcsss.envs.base import ControlMode, RelativeTo, RobotInstance
from rcsss.envs.factories import (
    default_fr3_hw_gripper_cfg,
    default_fr3_hw_robot_cfg,
    default_fr3_sim_gripper_cfg,
    default_fr3_sim_robot_cfg,
    default_mujoco_cameraset_cfg,
    default_realsense,
    fr3_hw_env,
    fr3_sim_env,
)
from rcsss.envs.wrappers import RHCWrapper, StorageWrapper

json_numpy.patch()


logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


ROBOT_INSTANCE = RobotInstance.HARDWARE
# CAMERA_GUI = "rgb_side"
# INSTRUCTION = "pick up the blue cube"
INSTRUCTION = "pick up the can"

default_cfg = OmegaConf.create(
    {
        "host": "dep-eng-air-p-1.hosts.utn.de",
        "port": 9000,
        "model": "openvla",
        "robot_ip": "192.168.101.1",
        "debug": True,
    }
    # {"host": "dep-eng-air-p-1.hosts.utn.de", "port": 7000, "model": "octo", "robot_ip": "192.168.101.1", "debug": False}
    # {"host": "localhost", "port": 8080, "model": "chatgpt", "robot_ip": "192.168.101.1", "debug": False}
)
cli_conf = OmegaConf.from_cli()
cfg = OmegaConf.merge(cli_conf, default_cfg)
DEBUG = cfg.debug


class RobotControl:
    def __init__(self, env: gym.Env, instruction: str, model_host: str, model_port: int, model_name: str):
        self.env = env
        self.gripper_state = 1

        self.instruction = instruction
        self.remote_agent = RemoteAgent(model_host, model_port, model_name)

        if not DEBUG:
            self.env.get_wrapper_attr("log_files")(self.remote_agent.git_status())

    def get_obs(self, obs: dict) -> Obs:
        # bird_eye=obs["frames"]["bird_eye"]["rgb"]
        # wrist=obs["frames"]["wrist"]["rgb"]

        side = obs["frames"]["side"]["rgb"]
        # side = obs["frames"]["default_free"]["rgb"]
        # side = obs["frames"]["openvla_view"]["rgb"]

        side = np.array(Image.fromarray(side).resize((256, 256), Image.Resampling.LANCZOS))

        # center crop square
        # h = side.shape[0]
        # start_w = (side.shape[1] - h) // 2
        # end_w = start_w + h
        # side = side[:, start_w:end_w]
        # bird_eye = bird_eye[:, start_w:end_w]
        # wrist = wrist[:, start_w:end_w]

        # rotate side by 90 degrees
        # side = cv2.rotate(side, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # rescale side to 256x256
        # side = cv2.resize(side, dsize=(256, 256), interpolation=cv2.INTER_CUBIC)
        # bird_eye = cv2.resize(bird_eye, dsize=(256, 256), interpolation=cv2.INTER_CUBIC)
        # wrist = cv2.resize(wrist, dsize=(256, 256), interpolation=cv2.INTER_CUBIC)

        # TODO: add some noise, use other cameras, record what is happening

        # return {"bird_eye": bird_eye, "wrist": wrist, "side": side, "gripper": obs["gripper"]}
        # return Obs(rgb_bird_eye=bird_eye, rgb_wrist=wrist, rgb_side=side, gripper=obs["gripper"], rgb=obs["frames"])
        return Obs(rgb_side=side, gripper=obs["gripper"])  # , info={"rgb": obs["frames"], "xyzrpy": obs["xyzrpy"]})

    def step(self, action) -> tuple[bool, list[str], dict]:
        # TODO Check if the model indicates when an action is finished.
        logger.info("Executing command")
        # only translation
        # action[3:6] = [0, 0, 0]
        # obs, _, _, truncated, info = self.env.step({"xyzrpy": action[:6], "gripper": action[6]})
        print("action", action)
        obs, _, _, truncated, info = self.env.step(action)
        del info["observations"]
        print("info", info)
        # TODO: ik success from info
        return False, obs, info

    def loop(self):
        # Initialize the environment and obtain the initial observation
        obs, _ = self.env.reset()
        info = {}

        logger.info("Initializing")
        obs_dict = self.get_obs(obs)

        # img = plt.imshow(obs_dict[CAMERA_GUI])
        # plt.show(block=False)
        # plt.pause(0.1)
        self.remote_agent.reset(obs_dict, instruction=self.instruction)

        logger.info("Starting control loop")
        while True:
            logger.info("Getting action")
            obs_dict.info = info
            action = self.remote_agent.act(obs_dict)
            if action.done:
                logger.info("done issues by agent, shutting down")
                break
            logger.info(f"action issued by agent: {action.action}")

            done, obs, info = self.step(action.action)
            obs_dict = self.get_obs(obs)
            # img.set_data(obs_dict.rgb_side)
            # plt.draw()
            # plt.pause(0.5)

            if done:
                break


def main():
    if ROBOT_INSTANCE == RobotInstance.HARDWARE:
        user, pw = load_creds_fr3_desk()
        resource_manger = FCI(
            Desk(cfg.robot_ip, user, pw), unlock=False, lock_when_done=False, guiding_mode_when_done=True
        )
    else:
        resource_manger = DummyResourceManager()
    with resource_manger:

        if ROBOT_INSTANCE == RobotInstance.HARDWARE:
            camera_dict = {
                # "wrist": "244222071045",
                # "bird_eye": "243522070364",
                # "side": "243522070385",
                "side": "244222071045",
            }

            camera_set = default_realsense(camera_dict)
            # env = utils.fr3_hw_env(
            #     ip=cfg.robot_ip,
            #     camera_set = camera_set,
            #     control_mode=ControlMode.CARTESIAN_TRPY,
            #     collision_guard=None,
            #     gripper=True,
            #     max_relative_movement=(0.1, np.deg2rad(5)),
            #     asynchronous=True,
            # relative_to=RelativeTo.LAST_STEP,
            # )
            env = fr3_hw_env(
                ip=cfg.robot_ip,
                camera_set=camera_set,
                robot_cfg=default_fr3_hw_robot_cfg(),
                control_mode=ControlMode.CARTESIAN_TRPY,
                collision_guard=None,
                gripper_cfg=default_fr3_hw_gripper_cfg(),
                max_relative_movement=(0.1, np.deg2rad(5)),
                relative_to=RelativeTo.LAST_STEP,
            )
        else:
            env = fr3_sim_env(
                control_mode=ControlMode.CARTESIAN_TRPY,
                robot_cfg=default_fr3_sim_robot_cfg(),
                gripper_cfg=default_fr3_sim_gripper_cfg(),
                camera_set_cfg=default_mujoco_cameraset_cfg(),
                max_relative_movement=(0.5, np.deg2rad(0)),
                # mjcf="/home/juelg/code/frankcsy/robot-control-stack/build/_deps/scenes-src/scenes/fr3_empty_world/scene.xml",
                mjcf="/home/juelg/code/frankcsy/models/scenes/lab/scene.xml",
                relative_to=RelativeTo.LAST_STEP,
            )
            env.get_wrapper_attr("sim").open_gui()
        if not DEBUG:
            env = StorageWrapper(env, path="octo_realv1_async", instruction=INSTRUCTION)

            # record videos
            video_path = env.path / "videos"
            video_path.mkdir(parents=True, exist_ok=True)
            camera_set.record_video(video_path, 0)

        env = RHCWrapper(env, exec_horizon=1)
        controller = RobotControl(env, INSTRUCTION, cfg.host, cfg.port, cfg.model)
        # controller = RobotControl(env, "Pick up the Yellow Brick from the Top", cfg.host, cfg.port, cfg.model)
        # controller = RobotControl(env, "Move Yellow Box to the Right", cfg.host, cfg.port, cfg.model)
        input("press enter to start")
        with env:
            controller.loop()


if __name__ == "__main__":
    main()
