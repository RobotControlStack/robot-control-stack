import logging
import os
from typing import Any

import gymnasium as gym
import numpy as np
import rcs
from rcs import sim
from rcs.envs.base import GripperWrapper, RobotEnv
from rcs.envs.sim import SimWrapper
from rcs.envs.space_utils import VecType
from rcs.envs.utils import default_sim_robot_cfg
from importlib.resources import files

import hydra
import cv2
import tacto

logger = logging.getLogger(__name__)
class TactoSimWrapper(gym.Wrapper):
    """Wrapper to use Tacto with RCS Sim."""

    def __init__(self, 
                 env: gym.Env, 
                 tacto_sites: list[str],
                 tacto_geoms: list[str],
                 tacto_meshes: dict[str, str] | None = None,
                 tacto_config: str | None = None,
                 tacto_bg: str | None = None,
                 enable_depth: bool = False,
                 tacto_fps: int = 60,
                 visualize: bool = False):
        '''
        Initialize Tacto sensor with the given configuration.
        Args:
            env (gym.Env): The environment to wrap.
            simulation (sim.Sim): The simulation instance.
            tacto_sites (list[str]): List of sites to mount Tacto cameras.
            tacto_geoms (list[str]): List of mjOBJ_GEOM names to add.
            tacto_meshes (dict[str, str] | None): Dictionary mapping geom names to mesh names. 
                                                  Needed when geom names are not the same as the mesh name in the XML.
            tacto_config (str)=None: Absolute path to the Tacto configuration folder containing "digit.yaml". 
                                     If None, package default is used.
            tacto_bg (str)=None: Absolute path to the background image for Tacto, ending with ".jpg". 
                                 If None, package default is used.
            enable_depth (bool)=False: Whether to enable depth rendering.
            tacto_fps (int)=60: Frames per second for Tacto rendering.
            visualize (bool)=False: Whether to visualize Tacto rendering in a separate window.
        '''
        super().__init__(env)
        self.env = env
        if tacto_config is None:
            tacto_config = os.path.dirname(files("tacto") / "cfg" / "digit.yaml")
            logger.warning(f"No tacto_config provided, using default from package: {tacto_config}/digit.yaml")
        if tacto_bg is None:
            tacto_bg = files("tacto") / "assets" / "bg_digit_240_320.jpg"
            logger.warning(f"No tacto_bg provided, using default from package: {tacto_bg}")
        hydra.initialize_config_dir(tacto_config)
        t_config = hydra.compose("digit.yaml")
        self.tacto_sensor = tacto.Sensor(**t_config.tacto, background=cv2.imread(tacto_bg))
        self.tacto_fps = tacto_fps
        self.tacto_last_render = -1
        self.tacto_sites = tacto_sites
        self.tacto_geoms = tacto_geoms
        self.tacto_meshes = tacto_meshes if tacto_meshes is not None else {}
        self.enable_depth = enable_depth
        self.model = self.env.get_wrapper_attr("sim").model
        self.data = self.env.get_wrapper_attr("sim").data
        self.initialized = False
        self.visualize = visualize

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
        obs, info = super().reset(seed=seed, options=options)
        if not self.initialized:
            # Set up Tacto sensor with the simulation 
            for site in self.tacto_sites:
                self.tacto_sensor.add_camera_mujoco(site, self.model, self.data)
            for geom in self.tacto_geoms:
                if(geom in self.tacto_meshes.keys()):
                    self.tacto_sensor.add_geom_mujoco(geom, self.model, self.data, self.tacto_meshes[geom])
                else:
                    self.tacto_sensor.add_geom_mujoco(geom, self.model, self.data)
            self.initialized = True
        self.tacto_last_render = -1  # Reset last render time
        colors, depths = self.tacto_sensor.render(self.model, self.data)
        for site, color, depth in zip(self.tacto_sites, colors, depths):
            obs.setdefault("tacto", {}).setdefault(site, {}).setdefault("rgb", {})["data"] = color
            if self.enable_depth:
                obs.setdefault("tacto", {}).setdefault(site, {}).setdefault("depth", {})["data"] = depth
        return obs, info

    def step(self, action: dict[str, Any]):
        obs, reward, done, truncated, info = super().step(action)
        if self.tacto_last_render + (1 / self.tacto_fps) < self.data.time:
            colors, depths = self.tacto_sensor.render(
                self.model,
                self.data
            )
            self.tacto_sensor.updateGUI(colors, depths) if self.visualize else None
            self.tacto_last_render = self.data.time
            for site, color, depth in zip(self.tacto_sites, colors, depths):
                obs.setdefault("tacto", {}).setdefault(site, {}).setdefault("rgb", {})["data"] = color
                if self.enable_depth:
                    obs.setdefault("tacto", {}).setdefault(site, {}).setdefault("depth", {})["data"] = depth
        return obs, reward, done, truncated, info