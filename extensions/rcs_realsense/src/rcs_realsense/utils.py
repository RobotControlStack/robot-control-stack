import typing

from rcs.camera.hw import CalibrationStrategy
from rcs_realsense.calibration import FR3BaseArucoCalibration
from rcs_realsense.camera import RealSenseCameraSet

from rcs import common


def default_realsense(
    name2id: dict[str, str] | None,
    resolution_width: int = 1280,
    resolution_height: int = 720,
    frame_rate: int = 15,
    enable_depth: bool = True,
) -> RealSenseCameraSet | None:
    if name2id is None:
        return None
    cameras = {
        name: common.BaseCameraConfig(
            identifier=id,
            resolution_width=resolution_width,
            resolution_height=resolution_height,
            frame_rate=frame_rate,
        )
        for name, id in name2id.items()
    }
    calibration_strategy = {name: typing.cast(CalibrationStrategy, FR3BaseArucoCalibration(name)) for name in name2id}
    return RealSenseCameraSet(cameras=cameras, calibration_strategy=calibration_strategy, enable_depth=enable_depth)


def default_realsense_dummy_calibration(name2id: dict[str, str] | None) -> RealSenseCameraSet | None:
    if name2id is None:
        return None
    cameras = {
        name: common.BaseCameraConfig(identifier=id, resolution_width=640, resolution_height=480, frame_rate=30)
        for name, id in name2id.items()
    }
    return RealSenseCameraSet(cameras=cameras)
