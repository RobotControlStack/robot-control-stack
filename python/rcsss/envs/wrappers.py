from datetime import datetime
import os
from pathlib import Path
from typing import Any, SupportsFloat

import gymnasium as gym

import os

import gymnasium as gym
import numpy as np
from PIL import Image
from rcsss.camera.hw import BaseHardwareCameraSet

import subprocess
import h5py


class StorageWrapperNumpy(gym.Wrapper):
    # TODO: this should also record the instruction
    FILE = "episode_{}.npz"
    GIF = "{}_episode_{}_{}.gif"
    FOLDER = "experiment_{}"
    GIF_DURATION_S = 0.5

    def __init__(
        self,
        env: gym.Env,
        path: str,
        instruction: str | None = None,
        description: str | None = None,
        gif: bool = True,
        camera_set: BaseHardwareCameraSet | None = None,
    ):
        super().__init__(env)
        self.episode_count = 0
        self.step_count = 0
        self.data = {}
        self.timestamp = str(datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
        self.gif = gif
        self.camera_set = camera_set
        self.prev_obs: dict | None = None

        # make folders
        self.path = Path(path) / self.FOLDER.format(self.timestamp)
        Path(self.path).mkdir(parents=True, exist_ok=False)
        if description is None:
            # write a small description from input into file
            description = input("Please enter a description for this experiment: ")
        with open(self.path / "description.txt", "w") as f:
            f.write(description)
        if instruction is None:
            # write instruction from input into file
            instruction = input("Instruction: ")
        self.language_instruction = str(instruction)
        self.data["language_instruction"] = self.language_instruction
        # get git commit id
        os.system(f'git log --format="%H" -n 1 > {os.path.join(str(self.path), "git_id.txt")}')
        # submodule git ids
        os.system(f'git submodule status > {os.path.join(str(self.path), "git_id_submodules.txt")}')
        # get git diff
        os.system(f'git diff --submodule=diff > {os.path.join(str(self.path), "git_diff.txt")}')

        self.gif_path = Path(path) / "gifs"
        if self.gif:
            self.gif_path.mkdir(parents=True, exist_ok=True)

    def flush(self):
        """writes data to disk"""
        if len(self.data) == 0 or self.step_count == 0:
            return
        print(self.data.keys())
        np.savez(self.path / self.FILE.format(self.episode_count), **self.data)
        if self.camera_set is not None and self.camera_set.recording_ongoing():
            self.camera_set.stop_video()

        if self.gif:
            # for key in ["side", "wrist", "bird_eye", "openvla_view"]:
            for key in ["side", "right_side", "bird_eye", "left_side", "front"]:
                if f"observation.frames.{key}.rgb" in self.data:
                    imgs = []
                    previous_timestamp = 0
                    for idx in range(min(len(self.data[f"observation.frames.{key}.rgb"]), len(self.data["timestamp"]))):
                        # skip images that have timestamps closer together than 0.5s
                        img = self.data[f"observation.frames.{key}.rgb"][idx]
                        if self.data["timestamp"][idx] - previous_timestamp < self.GIF_DURATION_S:
                            continue
                        previous_timestamp = self.data["timestamp"][idx]
                        imgs.append(Image.fromarray(img))
                    imgs[0].save(
                        self.gif_path / self.GIF.format(self.timestamp, self.episode_count, key),
                        save_all=True,
                        append_images=imgs[1:],
                        duration=self.GIF_DURATION_S * 1000,
                        loop=0,
                    )

        self.episode_count += 1
        self.data = {}

    # TODO: fix recorder order
    def step(self, action: dict) -> tuple[Any, SupportsFloat, bool, bool, dict[str, Any]]:
        obs, reward, terminated, truncated, info = super().step(action)
        # write obs and action into data
        act_obs = {"action": action, "observation": self.prev_obs, "timestamp": datetime.now().timestamp()}
        # delay observation by one time step to ensure that the observation leads to the action (and not like in gym env)
        self.prev_obs = obs
        act_obs["timestamp"] = datetime.now().timestamp()
        self.data["language_instruction"] = self.language_instruction
        for key, value in act_obs.items():
            if key not in self.data:
                self.data[key] = np.expand_dims(value, axis=0)
            else:
                self.data[key] = np.concatenate([self.data[key], np.expand_dims(value, axis=0)], axis=0)
        self.step_count += 1
        return obs, reward, terminated, truncated, info

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[Any, dict[str, Any]]:
        self.flush()
        self.step_count = 0
        self.prev_obs = None
        re = super().reset(seed=seed, options=options)
        self.prev_obs = re[0]
        return re

    def close(self):
        self.flush()
        return super().close()

    @property
    def logger_dir(self):
        return self.path

    def log_files(self, file2content: dict[str, str]):
        for fn, content in file2content.items():
            with open(self.path / fn, "w") as f:
                f.write(content)


# TODO: gifs should not be created after each episode, but there should rather be tool
# to create them from a dataset, how about video?
class StorageWrapperHDF5(gym.Wrapper):
    FILE = "data.h5"
    GIF = "{}_{}.gif"
    FOLDER = "experiment_{}"
    GIF_DURATION_S = 0.5

    def __init__(
        self,
        env: gym.Env,
        path: str,
        instruction: str | None = None,
        description: str | None = None,
        gif: bool = True,
        camera_set: BaseHardwareCameraSet | None = None,
    ):
        super().__init__(env)
        self.episode_count = 0
        self.step_count = 0
        self.timestamp = str(datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
        self.gif = gif
        self.prev_obs: dict | None = None
        self.datasets = {}
        self.camera_set = camera_set

        # Make folders
        self.path = Path(path)  # / self.FOLDER.format(self.timestamp)
        Path(self.path).mkdir(parents=True, exist_ok=True)
        if description is None:
            # Write a small description from input into file
            description = input("Please enter a description for this experiment: ")
        self.description = description

        # with open(self.path / "description.txt", "w") as f:
        #     f.write(self.description)

        if instruction is None:
            # Write instruction from input into file
            instruction = input("Instruction: ")
        self.language_instruction = str(instruction)
        # Open HDF5 file in append mode
        self.h5file = h5py.File(self.path / self.FILE, "a")
        # Check if instruction group exists
        if self.language_instruction in self.h5file:
            self.instruction_group = self.h5file[self.language_instruction]
        else:
            self.instruction_group = self.h5file.create_group(self.language_instruction)

        self.gif_path = self.path / "gifs"
        if self.gif:
            self.gif_path.mkdir(parents=True, exist_ok=True)

    def append_to_hdf5(self, group, data_dict, index):
        for key, value in data_dict.items():
            if isinstance(value, dict):
                # Handle subgroup
                if key not in group:
                    subgroup = group.create_group(key)
                else:
                    subgroup = group[key]
                self.append_to_hdf5(subgroup, value, index)
            else:
                # Handle dataset
                dataset_name = key
                full_dataset_path = group.name + "/" + dataset_name
                if full_dataset_path not in self.datasets:
                    # First time seeing this dataset
                    # Determine dtype
                    if isinstance(value, str):
                        # Variable-length string
                        dtype = h5py.string_dtype(encoding="utf-8")
                        shape = ()
                    elif np.isscalar(value):
                        # Numeric scalar
                        dtype = type(value)
                        shape = ()
                    elif isinstance(value, np.ndarray):
                        # Numpy array
                        dtype = value.dtype
                        shape = value.shape
                    else:
                        # Other types, try to convert to numpy array
                        try:
                            value = np.array(value)
                            dtype = value.dtype
                            shape = value.shape
                        except Exception as e:
                            raise ValueError(f"Unsupported data type for key '{key}': {type(value)}") from e
                    # Create dataset
                    initial_shape = (index + 1,) + shape
                    maxshape = (None,) + shape
                    dataset = group.create_dataset(
                        dataset_name, shape=initial_shape, maxshape=maxshape, chunks=True, dtype=dtype
                    )
                    self.datasets[full_dataset_path] = dataset
                else:
                    dataset = self.datasets[full_dataset_path]
                    if dataset.shape[0] <= index:
                        new_size = index + 1
                        dataset.resize(new_size, axis=0)
                # Store value
                if isinstance(value, str):
                    dataset[index] = value
                elif np.isscalar(value):
                    dataset[index] = value
                else:
                    dataset[index, ...] = value

    def flush(self):
        """Writes data to disk and generates GIFs if enabled."""
        if self.step_count == 0:
            return
        # Flush HDF5 file
        self.h5file.flush()
        # Stop camera recording if applicable
        if self.camera_set is not None and self.camera_set.recording_ongoing():
            self.camera_set.stop_video()
        # Generate GIFs if enabled
        if self.gif:
            for key in ["side", "right_side", "bird_eye", "left_side", "front"]:
                img_dataset_path = f"observation/frames/{key}/rgb"
                if img_dataset_path in self.episode_group:
                    dataset = self.episode_group[img_dataset_path]
                    imgs = []
                    previous_timestamp = 0
                    timestamp_dataset = self.episode_group["timestamp"]
                    for idx in range(min(len(dataset), len(timestamp_dataset))):
                        # Skip images that have timestamps closer together than self.GIF_DURATION_S
                        img = dataset[idx]
                        timestamp = timestamp_dataset[idx]
                        if timestamp - previous_timestamp < self.GIF_DURATION_S:
                            continue
                        previous_timestamp = timestamp
                        imgs.append(Image.fromarray(img))
                    if imgs:
                        imgs[0].save(
                            self.gif_path / self.GIF.format(self.timestamp, key),
                            save_all=True,
                            append_images=imgs[1:],
                            duration=self.GIF_DURATION_S * 1000,
                            loop=0,
                        )
        # Reset datasets for the next episode
        self.datasets = {}
        self.episode_count += 1

    def step(self, action: dict) -> tuple[Any, SupportsFloat, bool, bool, dict[str, Any]]:
        obs, reward, terminated, truncated, info = super().step(action)
        # Delay observation by one time step
        act_obs = {"action": action, "observation": self.prev_obs, "timestamp": datetime.now().timestamp()}
        self.prev_obs = obs  # Update prev_obs for next step
        # Append data to HDF5
        self.append_to_hdf5(self.episode_group, act_obs, self.step_count)
        self.step_count += 1
        return obs, reward, terminated, truncated, info

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[Any, dict[str, Any]]:
        self.flush()
        self.step_count = 0
        self.prev_obs = None
        # Create a new episode group
        episode_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f")
        self.episode_group = self.instruction_group.create_group(episode_name)
        self.datasets = {}
        # Get git metadata
        try:
            git_diff = subprocess.check_output(["git", "diff", "--submodule=diff"]).decode("utf-8")
            git_commit_id = subprocess.check_output(["git", "log", "--format=%H", "-n", "1"]).decode("utf-8").strip()
            git_submodule_status = subprocess.check_output(["git", "submodule", "status"]).decode("utf-8")
        except Exception as e:
            git_diff = ""
            git_commit_id = ""
            git_submodule_status = ""
        # Store git info as attributes
        self.episode_group.attrs["git_diff"] = git_diff
        self.episode_group.attrs["git_commit_id"] = git_commit_id
        self.episode_group.attrs["git_submodule_status"] = git_submodule_status
        # Also store description and language instruction
        self.episode_group.attrs["description"] = self.description
        self.episode_group.attrs["language_instruction"] = self.language_instruction
        result = super().reset(seed=seed, options=options)
        self.prev_obs = result[0]  # Initialize prev_obs
        return result

    def close(self):
        self.flush()
        self.h5file.close()
        return super().close()

    @property
    def logger_dir(self):
        return self.path

    def log_files(self, file2content: dict[str, str]):
        for fn, content in file2content.items():
            with open(self.path / fn, "w") as f:
                f.write(content)


def listdict2dictlist(LD):
    return {k: [dic[k] for dic in LD] for k in LD[0]}


class RHCWrapper(gym.Wrapper):
    """
    Performs receding horizon control. The policy returns `pred_horizon` actions and
    we execute `exec_horizon` of them.
    """

    def __init__(self, env: gym.Env, exec_horizon: int):
        super().__init__(env)
        self.exec_horizon = exec_horizon

    def step(self, actions):
        if self.exec_horizon == 1 and len(actions.shape) == 1:
            actions = actions[None]
        assert len(actions) >= self.exec_horizon
        rewards = []
        observations = []
        infos = []

        for i in range(self.exec_horizon):
            # obs, reward, done, trunc, info = self.env.step(actions[i])
            obs, reward, done, trunc, info = self.env.step({"xyzrpy": actions[i, :6], "gripper": actions[i, 6]})
            observations.append(obs)
            rewards.append(reward)
            infos.append(info)

            if done or trunc:
                break

        infos = listdict2dictlist(infos)
        infos["rewards"] = rewards
        infos["observations"] = observations

        return obs, np.sum(rewards), done, trunc, infos
