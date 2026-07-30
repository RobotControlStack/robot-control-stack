from rcs.camera.hw import CalibrationStrategy
from rcs_zed.camera import ZEDCameraSet

from rcs import common


def default_zed(
    name2id: dict[str, str] | None,
    calibration_strategy: dict[str, CalibrationStrategy] | None = None,
) -> ZEDCameraSet | None:
    """Create the default ZED camera set.

    Args:
        name2id: Mapping from logical camera names to ZED serial numbers.
        calibration_strategy: Optional calibration strategy for each logical
            camera. When omitted, ``ZEDCameraSet`` uses
            ``DummyCalibrationStrategy``.
    """
    if name2id is None:
        return None
    cameras = {
        name: common.BaseCameraConfig(identifier=id, resolution_width=1280, resolution_height=720, frame_rate=30)
        for name, id in name2id.items()
    }
    return ZEDCameraSet(cameras=cameras, calibration_strategy=calibration_strategy)


def default_zed_dummy_calibration(name2id: dict[str, str] | None) -> ZEDCameraSet | None:
    """Create the default ZED camera set with dummy calibration."""
    return default_zed(name2id)
