from collections.abc import Collection

import numpy as np
from rcs.camera.interface import BaseCameraSet


def capture_blank_camera_images(
    camera_set: BaseCameraSet,
    camera_names: Collection[str],
) -> dict[str, np.ndarray]:
    """Copy initialization-time RGB images for the requested tactile cameras.

    The camera set must already be initialized and have at least one buffered
    frame. All images are taken from the same latest frame set.
    """
    if not camera_names:
        return {}

    frameset = camera_set.get_latest_frames()
    if frameset is None:
        msg = "Cameras were initialized but no frames are available for blank-image capture."
        raise RuntimeError(msg)

    missing_camera_frames = set(camera_names) - frameset.frames.keys()
    if missing_camera_frames:
        missing = ", ".join(sorted(missing_camera_frames))
        msg = f"No initialized frame is available for camera(s): {missing}"
        raise RuntimeError(msg)

    blank_camera_dict: dict[str, np.ndarray] = {}
    for name in camera_names:
        image = frameset.frames[name].camera.color.data
        if not isinstance(image, np.ndarray):
            msg = f"RGB frame for camera {name!r} must be a numpy array."
            raise TypeError(msg)
        blank_camera_dict[name] = image.copy()
    return blank_camera_dict
