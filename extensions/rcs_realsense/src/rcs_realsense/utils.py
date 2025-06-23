from rcs_realsense.camera import RealSenseCameraSet

from rcs import common


def default_realsense(name2id: dict[str, str] | None) -> RealSenseCameraSet | None:
    if name2id is None:
        return None
    cameras = {
        name: common.BaseCameraConfig(identifier=id, resolution_width=1280, resolution_height=720, frame_rate=30)
        for name, id in name2id.items()
    }
    return RealSenseCameraSet(cameras=cameras)
