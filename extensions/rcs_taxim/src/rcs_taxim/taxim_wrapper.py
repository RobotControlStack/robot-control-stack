import copy
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
import TaximSensor


def _robotiq2f85_digit_model_path() -> str:
    return str(Path(TaximSensor.__file__).parent.parent.parent / "assets/robotiq_2f85/robotiq_2f85.xml")


class TaximSimWrapper(gym.Wrapper):
    """Render TAXIM observations, including static initial RGB reference images by default."""

    def __init__(
        self,
        env: gym.Env,
        taxim_sites: list[str],
        taxim_pad_geoms: list[str],
        target_geom_mesh_dict: dict[str, str],
        target_geom_normal_map_dict: dict[str, str] | None = None,
        taxim_sensor_type: str = "digit",
        bg_file: str | Path | None = None,
        taxim_bg_idx: int = 0,
        taxim_bg_randomize: bool = False,
        enable_depth: bool = False,
        taxim_fps: int = 60,
        visualize: bool = False,
        include_blank_images: bool = True,
    ):
        super().__init__(env)
        self.taxim_sensors: list[Any] = []
        self.model = self.env.get_wrapper_attr("sim").model
        self.data = self.env.get_wrapper_attr("sim").data
        self.last_tactile_frames: dict[str, dict[str, dict[str, Any]]] = {}
        self.blank_tactile_frames: dict[str, dict[str, dict[str, Any]]] = {}

        self.taxim_sites = taxim_sites
        self.taxim_pad_geoms = taxim_pad_geoms
        self.target_geom_mesh_dict = target_geom_mesh_dict
        self.target_geom_normal_map_dict = target_geom_normal_map_dict
        self.taxim_sensor_type = taxim_sensor_type
        self.bg_file = bg_file
        self.taxim_bg_idx = taxim_bg_idx
        self.taxim_bg_randomize = taxim_bg_randomize
        self.taxim_fps = taxim_fps
        self.taxim_last_render = -1.0
        self.enable_depth = enable_depth
        self.initialized = False
        self.visualize = visualize
        self.include_blank_images = include_blank_images

        self.observation_space = copy.deepcopy(env.observation_space)
        if not isinstance(self.observation_space, gym.spaces.Dict):
            msg = "Expected wrapped observation space to be a gym.spaces.Dict."
            raise TypeError(msg)

        frame_spaces = self.observation_space.spaces.get("frames")
        if frame_spaces is None:
            frame_spaces = gym.spaces.Dict({})
            self.observation_space.spaces["frames"] = frame_spaces
        if not isinstance(frame_spaces, gym.spaces.Dict):
            msg = "Expected frames observation space to be a gym.spaces.Dict."
            raise TypeError(msg)

        tactile_space: dict[str, gym.Space[Any]] = {
            "rgb": gym.spaces.Dict(
                {
                    "data": gym.spaces.Box(low=0, high=255, shape=(320, 240, 3), dtype=np.uint8),
                }
            )
        }
        if self.enable_depth:
            tactile_space["depth"] = gym.spaces.Dict(
                {
                    "data": gym.spaces.Box(low=-np.inf, high=np.inf, shape=(320, 240), dtype=np.float64),
                }
            )
        frame_spaces.spaces.update(
            {f"tactile_{site}": gym.spaces.Dict(copy.deepcopy(tactile_space)) for site in self.taxim_sites}
        )
        if self.include_blank_images:
            blank_rgb_space = {"rgb": copy.deepcopy(tactile_space["rgb"])}
            frame_spaces.spaces.update(
                {f"tactile_{site}_blank": gym.spaces.Dict(copy.deepcopy(blank_rgb_space)) for site in self.taxim_sites}
            )

    def _ensure_initialized(self) -> None:
        if self.initialized:
            return

        for sensor_index, (site, pad_geom) in enumerate(zip(self.taxim_sites, self.taxim_pad_geoms, strict=True)):
            sensor = TaximSensor.TaximSensor(
                resize=(240, 320),
                sensor_type=self.taxim_sensor_type,
                bg_file=self.bg_file,
                preprocess_bg=False,
            )
            sensor.add_camera_mujoco(site, self.model, self.data)
            if len(self.taxim_sites) > sensor.bg_len:
                msg = (
                    f"{len(self.taxim_sites)} TAXIM sites require distinct backgrounds, "
                    f"but only {sensor.bg_len} are available."
                )
                raise ValueError(msg)
            sensor.change_bg((self.taxim_bg_idx + sensor_index) % sensor.bg_len)
            for geom, mesh in self.target_geom_mesh_dict.items():
                normal_map_path = None
                if self.target_geom_normal_map_dict is not None and geom in self.target_geom_normal_map_dict:
                    normal_map_path = self.target_geom_normal_map_dict[geom]
                sensor.add_geom_mujoco(geom, self.model, self.data, mesh, normal_map_path=normal_map_path)
            sensor.set_sensor_pad_geom(pad_geom)
            self.taxim_sensors.append(sensor)
        self.initialized = True

    def _render_tactile_frames(self, visualize: bool) -> dict[str, dict[str, dict[str, Any]]]:
        frames: dict[str, dict[str, dict[str, Any]]] = {}
        for site, sensor in zip(self.taxim_sites, self.taxim_sensors, strict=True):
            rgb, depth, _ = sensor.render_taxim(
                self.model, self.data, visualize=self.visualize, cycle_bg=self.taxim_bg_randomize
            )
            tactile_obs: dict[str, dict[str, Any]] = {"rgb": {"data": rgb}}
            if self.enable_depth:
                tactile_obs["depth"] = {"data": depth}
            frames[f"tactile_{site}"] = tactile_obs
        return frames

    def _render_blank_tactile_frames(self) -> dict[str, dict[str, dict[str, Any]]]:
        return {
            f"tactile_{site}_blank": {"rgb": {"data": sensor.render_blank_taxim()}}
            for site, sensor in zip(self.taxim_sites, self.taxim_sensors, strict=True)
        }

    def _render_current_tactile_frames(self, visualize: bool) -> None:
        if self.include_blank_images and (self.taxim_bg_randomize or not self.blank_tactile_frames):
            print("bllank render")
            self.blank_tactile_frames = self._render_blank_tactile_frames()
        self.last_tactile_frames = self._render_tactile_frames(visualize=visualize)

    def _update_obs(self, obs: dict[str, Any]) -> None:
        frames = obs.setdefault("frames", {})
        for site, tactile_obs in self.last_tactile_frames.items():
            frames[site] = copy.deepcopy(tactile_obs)
        for site, blank_tactile_obs in self.blank_tactile_frames.items():
            frames[site] = copy.deepcopy(blank_tactile_obs)

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        obs, info = super().reset(seed=seed, options=options)
        self._ensure_initialized()
        self.taxim_last_render = -1.0
        self._render_current_tactile_frames(visualize=False)
        self._update_obs(obs)
        return obs, info

    def step(self, action: dict[str, Any]):
        obs, reward, done, truncated, info = super().step(action)
        self._ensure_initialized()
        if self.taxim_last_render + (1 / self.taxim_fps) <= self.data.time:
            self._render_current_tactile_frames(visualize=self.visualize)
            self.taxim_last_render = self.data.time

        self._update_obs(obs)
        return obs, reward, done, truncated, info
