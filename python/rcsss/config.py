from typing import Optional

from pydantic import BaseModel
from pydantic_yaml import parse_yaml_raw_as, to_yaml_str


class HWConfig(BaseModel):
    # Franka Desk credentials
    username: str
    password: str
    # path to the urdf model
    urdf_model_path: Optional[str]


class SimConfig(BaseModel):
    pass


class Config(BaseModel):
    hw: HWConfig
    sim: SimConfig


def create_sample_config_yaml(path: str):
    hw = HWConfig(username="...", password="...", urdf_model_path="path/to/urdf")
    sim = SimConfig()
    cfg = Config(hw=hw, sim=sim)
    yml = to_yaml_str(cfg)
    with open(path, "w") as f:
        f.write(yml)


def read_config_yaml(path: str) -> Config:
    return parse_yaml_raw_as(Config, path)
