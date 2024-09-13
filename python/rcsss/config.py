from pydantic import BaseModel
from pydantic_yaml import parse_yaml_raw_as, to_yaml_str
from rcsss.camera.interface import BaseCameraConfig
from rcsss.camera.realsense import RealSenseSetConfig
from rcsss.camera.sim import SimCameraConfig, SimCameraSetConfig


# TODO: this design might need to be adapted in order to
# configure each camera separately
# For this one could add a dict of camera configs
class CameraConfig(BaseModel):
    # kinect_config: KinectConfig | None = None
    realsense_config: RealSenseSetConfig | None = None
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
    camera: SimCameraSetConfig


class Config(BaseModel):
    hw: HWConfig
    sim: SimConfig


def create_sample_config_yaml(path: str):

    cameras = {"human_readable_name": BaseCameraConfig(identifier="serial_number")}
    real_sense_cfg = RealSenseSetConfig(cameras=cameras)
    camera_cfg = CameraConfig(realsense_config=real_sense_cfg)
    hw = HWConfig(
        username="...",
        password="...",
        urdf_model_path="path/to/urdf",
        camera_config=camera_cfg,
        camera_type="realsense",
    )
    cameras_sim = {"human_readable_name": SimCameraConfig(identifier="mjcf_name", type=0, on_screen_render=False)}
    sim = SimConfig(camera=SimCameraSetConfig(cameras=cameras_sim))
    cfg = Config(hw=hw, sim=sim)
    yml = to_yaml_str(cfg)
    with open(path, "w") as f:
        f.write(yml)


def read_config_yaml(path: str) -> Config:
    with open(path, "r") as f:
        return parse_yaml_raw_as(Config, f)
