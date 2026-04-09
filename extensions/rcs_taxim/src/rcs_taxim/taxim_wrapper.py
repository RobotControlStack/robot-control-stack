import logging
import os
from importlib.resources import files
from typing import Any

import cv2
import gymnasium as gym
from TaximSensor import TaximSensor
from omegaconf import OmegaConf

logger = logging.getLogger(__name__)


class TaximSimWrapper(gym.Wrapper):
    """Wrapper to use Taxim with RCS Sim."""

    def __init__(
        self,
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
    ):
        """
        Initialize Taxim sensor with the given configuration.
        Args:
            env (gym.Env): The environment to wrap.
            simulation (sim.Sim): The simulation instance.
            taxim_sites (list[str]): List of sites to mount Taxim cameras.
            taxim_pad_geoms (list[str]): List of tactile sensor pad geoms which should act as the contact surfaces.
            target_geom_mesh_dict (dict[str, str]): Dictionary mapping mjGeom names to mjMesh names.
            taxim_sensor_type (str)="digit": The type of Taxim sensor to use. either 'digit' or 'gelsight_r1.5'.
            taxim_bg_idx (int)=0: The index of the background image to use.
            taxim_bg_randomize (bool)=False: Whether to randomize the background image for every contact.
            enable_depth (bool)=False: Whether to enable depth rendering.
            taxim_fps (int)=60: Frames per second for Taxim rendering.
            visualize (bool)=False: Whether to visualize Taxim rendering in a separate window.
        """
        super().__init__(env)
        self.env = env
        self.taxim_sensors = []

        self.model = self.env.get_wrapper_attr("sim").model
        self.data = self.env.get_wrapper_attr("sim").data

        self.taxim_sites = taxim_sites
        self.taxim_pad_geoms = taxim_pad_geoms
        self.target_geom_mesh_dict = target_geom_mesh_dict
        self.taxim_sensor_type = taxim_sensor_type
        self.taxim_bg_idx = taxim_bg_idx
        self.taxim_bg_randomize = taxim_bg_randomize

        self.colors = []
        self.depths = []

        self.taxim_fps = taxim_fps
        self.taxim_last_render = -1
        self.enable_depth = enable_depth
        
        self.initialized = False
        self.visualize = visualize

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        obs, info = super().reset(seed=seed, options=options)
        if not self.initialized:
            # Create taxim sensors for each specified site
            print(self.taxim_sites)
            for i, site in enumerate(self.taxim_sites):
                sensor = TaximSensor(resize=(240,320), sensor_type=self.taxim_sensor_type, preprocess_bg=False)
                sensor.add_camera_mujoco(site, self.model, self.data)
                sensor.change_bg(self.taxim_bg_idx)
                # Add the target geoms to the sensor
                for geom, mesh in self.target_geom_mesh_dict.items():
                    sensor.add_geom_mujoco(geom, self.model, self.data, mesh)

                sensor.set_sensor_pad_geom(self.taxim_pad_geoms[i])
                self.taxim_sensors.append(sensor)

            self.initialized = True
            
        self.taxim_last_render = -1  # Reset last render time

        for i, sensor in enumerate(self.taxim_sensors):
            rgb, depth, _ = sensor.render_taxim(self.model, self.data, visualize=False)
            obs.setdefault("frames", {}).setdefault(f"tactile_{self.taxim_sites[i]}", {}).setdefault("rgb", {})["data"] = rgb
            if self.enable_depth:
                obs.setdefault("frames", {}).setdefault(f"tactile_{self.taxim_sites[i]}", {}).setdefault("depth", {})["data"] = depth
        return obs, info

    def step(self, action: dict[str, Any]):
        obs, reward, done, truncated, info = super().step(action)
        if self.taxim_last_render + (1 / self.taxim_fps) > self.data.time:
            return obs, reward, done, truncated, info

        for i, sensor in enumerate(self.taxim_sensors):
            rgb, depth, _ = sensor.render_taxim(self.model, self.data, visualize=self.visualize)
            self.taxim_last_render = self.data.time
            obs.setdefault("frames", {}).setdefault(f"tactile_{self.taxim_sites[i]}", {}).setdefault("rgb", {})["data"] = rgb
            if self.enable_depth:
                obs.setdefault("frames", {}).setdefault(f"tactile_{self.taxim_sites[i]}", {}).setdefault("depth", {})["data"] = depth

        return obs, reward, done, truncated, info
