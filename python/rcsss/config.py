from typing import Optional

from pydantic import BaseModel
from pydantic_yaml import parse_yaml_raw_as


class Config(BaseModel):
    username: str
    password: str
    urdf_model_path: Optional[str]


def read_config_yaml(path: str) -> Config:
    return parse_yaml_raw_as(Config, path)
