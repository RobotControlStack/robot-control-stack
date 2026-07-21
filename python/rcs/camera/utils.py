from collections.abc import Collection
from typing import Any

import numpy as np
from rcs.camera.interface import BaseCameraSet


def add_blank_camera_streams(
    observation: dict[str, Any],
    blank_camera_dict: dict[str, np.ndarray],
) -> None:
    """Add static ``<camera_name>_blank`` RGB streams to an observation in place."""
    if not blank_camera_dict:
        return

    frames = observation.setdefault("frames", {})
    blank_names = {f"{camera_name}_blank" for camera_name in blank_camera_dict}
    collisions = blank_names.intersection(frames)
    if collisions:
        names = ", ".join(sorted(collisions))
        msg = f"Blank camera stream name(s) already exist in the observation: {names}"
        raise ValueError(msg)

    frames.update(
        {
            f"{camera_name}_blank": {
                "rgb": {
                    "data": blank_image.copy(),
                    "intrinsics": None,
                    "extrinsics": None,
                }
            }
            for camera_name, blank_image in blank_camera_dict.items()
        }
    )


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
