from rcs_realsense.calibration import FR3BaseArucoCalibration
from rcs_realsense.camera import RealSenseCameraSet

from rcs import common


def default_realsense(name2id: dict[str, str] | None) -> RealSenseCameraSet | None:
    if name2id is None:
        return None
    cameras = {
        name: common.BaseCameraConfig(identifier=id, resolution_width=1280, resolution_height=720, frame_rate=30)
        for name, id in name2id.items()
    }
    calibration_strategy = {name: FR3BaseArucoCalibration(name) for name in name2id}
    return RealSenseCameraSet(cameras=cameras, calibration_strategy=calibration_strategy)
