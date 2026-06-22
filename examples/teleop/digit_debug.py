import logging
import os

import numpy as np
import gymnasium as gym
from rcs._core.common import BaseCameraConfig
from rcs.envs.base import CameraSetWrapper, ControlMode, HardwareEnv, RelativeTo
from rcs.envs.storage_wrapper import StorageWrapper
from rcs.utils import SimpleFrameRate

from rcs_fr3.creators import HardwareCameraCreatorConfig, _create_hardware_camera_set
from rcs.camera.hw import HardwareCameraSet

DIGIT_DICT = {
    "digit_right_left": "D20747",
    "digit_right_right": "D21319"
}
CAMERA_DICT = {
    "wrist": "243222074728",
    "side": "327122079439",
}

ROBOT2IP = {
    # "right": "192.168.102.1",
    "right": "192.168.100.1",
}
ROBOT2ID = {
    # "left": "0",
    "right": "0",
}

def _storage_base_dir(output_path: str) -> str:
    if output_path.lower().endswith(".parquet"):
        return os.path.dirname(output_path) or "."
    return output_path


def _select_camera_subset(items: dict[str, str], use_left: bool, use_right: bool):
    selected: dict[str, str] = {}
    if not items:
        return selected
    if use_left:
        selected[next(iter(items.keys()))] = next(iter(items.values()))
    if use_right:
        right_key, right_val = list(items.items())[-1]
        if not (use_left and right_key in selected):
            selected[right_key] = right_val
    return selected


def _build_camera_cfgs(add_rs1: bool, add_rs2: bool, add_digit1: bool, add_digit2: bool) -> dict[str, HardwareCameraCreatorConfig]:
    camera_cfgs: dict[str, HardwareCameraCreatorConfig] = {}

    rs_cfg = _select_camera_subset(CAMERA_DICT, add_rs1, add_rs2)
    if rs_cfg:
        camera_cfgs["realsense"] = HardwareCameraCreatorConfig(
            camera_type_id="realsense",
            camera_cfgs={
                name: BaseCameraConfig(identifier=identifier, resolution_width=640, resolution_height=480, frame_rate=30)
                for name, identifier in rs_cfg.items()
            },
        )

    digit_cfg = _select_camera_subset(DIGIT_DICT, add_digit1, add_digit2)
    if digit_cfg:
        camera_cfgs["digit"] = HardwareCameraCreatorConfig(
            camera_type_id="digit",
            camera_cfgs={
                name: BaseCameraConfig(identifier=identifier, resolution_width=320, resolution_height=240, frame_rate=30)
                for name, identifier in digit_cfg.items()
            },
        )

    return camera_cfgs


def _build_camera_set(add_rs1: bool, add_rs2: bool, add_digit1: bool, add_digit2: bool) -> HardwareCameraSet | None:
    camera_cfgs = _build_camera_cfgs(add_rs1, add_rs2, add_digit1, add_digit2)
    return _create_hardware_camera_set(camera_cfgs) if camera_cfgs else None

def get_env(
    add_robot: bool = False,
    add_rs2: bool = False,
    add_rs1: bool = False,
    add_digit2: bool = False,
    add_digit1: bool = True,
    output_path: str = "debug",
):
    home_joint_pose = None
    if add_robot:
        from rcs_fr3.configs import SingleArmFR3MultiHardwareEnv
        env_creator = SingleArmFR3MultiHardwareEnv()
        # env_creator.left_ip = ROBOT2IP["left"]
        env_creator.ip = ROBOT2IP["right"]
        hw_cfg = env_creator.config()
        hw_cfg.control_mode = ControlMode.JOINTS
        # RelativeActionSpace expects a single float in radians for joints.
        hw_cfg.max_relative_movement = float(np.deg2rad(5))
        # Use absolute joint commands directly so we can resend a stable home target each step.
        hw_cfg.relative_to = RelativeTo.NONE
        home_joint_pose = hw_cfg.robot_cfgs["right"].q_home
        # FR3 env creators expect camera configuration dicts, not a built HardwareCameraSet.
        hw_cfg.camera_cfgs = _build_camera_cfgs(
            add_rs1=add_rs1, add_rs2=add_rs2, add_digit1=add_digit1, add_digit2=add_digit2
        )
        print(hw_cfg.camera_cfgs)
        env = env_creator.create_env(hw_cfg)
    else:
        # Controlled hardware test stack (no robot): start from bare HardwareEnv and add layers conditionally.
        env = HardwareEnv()
        # Bare HardwareEnv has no spaces; define minimal neutral spaces for loop-driving.
        env.action_space = gym.spaces.Dict({})
        env.observation_space = gym.spaces.Dict({})

        camera_set = _build_camera_set(
            add_rs1=add_rs1, add_rs2=add_rs2, add_digit1=add_digit1, add_digit2=add_digit2
        )
        if camera_set is not None:
            camera_set.start()
            camera_set.wait_for_frames()
            env = CameraSetWrapper(env, camera_set)

    env = StorageWrapper(
        env,
        _storage_base_dir(output_path),
        instruction="digit_debug_run",
        batch_size=64,
        max_rows_per_group=100,
        max_rows_per_file=1000,
    )

    return env, home_joint_pose


def _build_joint_home_action(action_space, home_joint_pose, last_obs=None):
    obs_joint = None
    obs_gripper = None

    def _scalar_from_obs(value):
        if isinstance(value, (int, float, np.floating)):
            return float(value)
        if isinstance(value, np.ndarray):
            if value.size == 0:
                return None
            return float(value.flat[0])
        if isinstance(value, (list, tuple)):
            if not value:
                return None
            return _scalar_from_obs(value[0])
        return None

    if isinstance(last_obs, dict) and home_joint_pose is None:
        # Prefer last observation when we are not in fixed-home robot mode.
        if "joints" in last_obs:
            obs_joint = np.asarray(last_obs["joints"])
        if "gripper" in last_obs:
            obs_gripper = _scalar_from_obs(last_obs["gripper"])

    if home_joint_pose is None and obs_joint is None:
        action = action_space.sample()
        return action if action else {"_dummy": 0.0}

    if isinstance(action_space, gym.spaces.Dict):
        robot_keys = list(action_space.spaces.keys())
        if len(robot_keys) > 0 and isinstance(action_space.spaces[robot_keys[0]], gym.spaces.Dict):
            nested_obs = None
            if isinstance(last_obs, dict) and robot_keys[0] in last_obs:
                nested_obs = last_obs[robot_keys[0]]
            return {
                robot_keys[0]: _build_joint_home_action(
                    action_space.spaces[robot_keys[0]],
                    home_joint_pose,
                    nested_obs,
                )
            }

        action = {}
        if "joints" in action_space.spaces:
            if home_joint_pose is not None:
                action["joints"] = np.asarray(home_joint_pose, dtype=np.float64)
            elif obs_joint is not None:
                action["joints"] = np.asarray(obs_joint, dtype=np.float64)
        if "gripper" in action_space.spaces:
            if home_joint_pose is not None:
                action["gripper"] = 1.0
            elif obs_gripper is not None:
                action["gripper"] = obs_gripper
            else:
                action["gripper"] = 1.0
        if "xyzrpy" in action_space.spaces:
            if "xyzrpy" in (last_obs if isinstance(last_obs, dict) else {}):
                action["xyzrpy"] = np.asarray(last_obs["xyzrpy"], dtype=np.float64)
            else:
                action["xyzrpy"] = np.zeros(6, dtype=np.float64)
        if "tquat" in action_space.spaces:
            if "tquat" in (last_obs if isinstance(last_obs, dict) else {}):
                action["tquat"] = np.asarray(last_obs["tquat"], dtype=np.float64)
            else:
                action["tquat"] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        if not action:
            action = action_space.sample()
            if isinstance(action, dict):
                return action
            return {"_dummy": 0.0}
        return action

    if isinstance(last_obs, np.ndarray):
        # Fallback for non-dict action spaces
        action = last_obs
    else:
        action = action_space.sample()
    return action if action else {"_dummy": 0.0}


from tqdm import tqdm
def main():

    output_path = "digit_2_rs_2_robot"
    add_robot=True
    add_rs2=True
    add_rs1=True
    add_digit2=True
    add_digit1=True
    
    env, home_joint_pose = get_env(
        add_robot=add_robot,
        add_rs2=add_rs2,
        add_rs1=add_rs1,
        add_digit2=add_digit2,
        add_digit1=add_digit1,
        output_path=output_path,
    )
    framerate = 30
    rate_limiter = SimpleFrameRate(
            framerate, "env loop"
        )
    target_seconds = 240
    num_steps = target_seconds*framerate
    try:
        obs, info = env.reset()
        last_obs = obs if isinstance(obs, dict) else None
        env.start_record()
        print("reset", len(obs) if isinstance(obs, dict) else type(obs), info)
        for i in tqdm(range(num_steps)):
            action = _build_joint_home_action(
                env.action_space,
                home_joint_pose if add_robot else None,
                last_obs if isinstance(last_obs, dict) else None,
            )
            obs, reward, terminated, truncated, info = env.step(action)
            if isinstance(obs, dict):
                last_obs = obs
            rate_limiter()
            # print(f"step={i}, reward={reward}, terminated={terminated}, truncated={truncated}")
            if terminated or truncated:
                obs, info = env.reset()
                last_obs = obs if isinstance(obs, dict) else None
                print("reset", len(obs) if isinstance(obs, dict) else type(obs), info)
    finally:
        env.close()


if __name__ == "__main__":
    main()
