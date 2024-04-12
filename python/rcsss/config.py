from typing import Optional

from pydantic import BaseModel
from pydantic_yaml import parse_yaml_raw_as


class HWConfig(BaseModel):
    username: str
    password: str
    urdf_model_path: Optional[str]


class SimConfig(BaseModel):
    pass


class Config(BaseModel):
    hw: HWConfig
    sim: SimConfig


def read_config_yaml(path: str) -> Config:
    return parse_yaml_raw_as(Config, path)
