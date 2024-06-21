from pydantic import BaseModel
from pydantic_yaml import parse_yaml_raw_as, to_yaml_str
from rcsss.camera.realsense import RealSenseConfig


# TODO: this design might need to be adapted in order to
# configure each camera separately
# For this one could add a dict of camera configs
class CameraConfig(BaseModel):
    # kinect_config: KinectConfig | None = None
    realsense_config: RealSenseConfig | None = None
    # TODO: we can also add sim camera configs here


class HWConfig(BaseModel):
    # Franka Desk credentials
    username: str
    password: str
    # path to the urdf model
    urdf_model_path: str | str
    camera_type: str | None = None
    camera_config: CameraConfig | None = None


class SimConfig(BaseModel):
    pass


class Config(BaseModel):
    hw: HWConfig
    sim: SimConfig


def create_sample_config_yaml(path: str):
    real_sense_cfg = RealSenseConfig(devices_to_enable={"human_readable_name": "serial_number"})
    camera_cfg = CameraConfig(realsense_config=real_sense_cfg)
    hw = HWConfig(username="...", password="...", urdf_model_path="path/to/urdf", camera_config=camera_cfg)
    sim = SimConfig()
    cfg = Config(hw=hw, sim=sim)
    yml = to_yaml_str(cfg)
    with open(path, "w") as f:
        f.write(yml)


def read_config_yaml(path: str) -> Config:
    with open(path, "r") as f:
        return parse_yaml_raw_as(Config, f)
