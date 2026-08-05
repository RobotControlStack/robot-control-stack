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
                    # StorageWrapper cannot infer a stable Arrow schema from fields
                    # that are None in every row.  These blank images are only a
                    # reference stream, so use calibration-shaped placeholders.
                    "intrinsics": np.eye(3, 4, dtype=np.float64),
                    "extrinsics": np.eye(4, dtype=np.float64),
                }
            }
            for camera_name, blank_image in blank_camera_dict.items()
        }
    )


def capture_blank_camera_images(
    camera_set: BaseCameraSet,
    camera_names: Collection[str],
    num_frames: int = 50,
) -> dict[str, np.ndarray]:
    """Average initialization-time RGB images for the requested tactile cameras.

    The camera set must already be initialized and running. Its buffer is
    cleared before every sample so each image comes from a fresh frame set.
    The averaged images retain the shape and dtype of the device images.
    """
    if not camera_names:
        return {}
    if num_frames < 1:
        msg = f"num_frames must be at least 1, got {num_frames}."
        raise ValueError(msg)

    names = tuple(camera_names)
    captured_images: dict[str, list[np.ndarray]] = {name: [] for name in names}
    image_metadata: dict[str, tuple[tuple[int, ...], np.dtype]] = {}

    for _ in range(num_frames):
        camera_set.clear_buffer()
        frameset = camera_set.get_latest_frames()
        if frameset is None:
            msg = "Cameras were initialized but no frames are available for blank-image capture."
            raise RuntimeError(msg)

        missing_camera_frames = set(names) - frameset.frames.keys()
        if missing_camera_frames:
            missing = ", ".join(sorted(missing_camera_frames))
            msg = f"No initialized frame is available for camera(s): {missing}"
            raise RuntimeError(msg)

        for name in names:
            image = frameset.frames[name].camera.color.data
            if not isinstance(image, np.ndarray):
                msg = f"RGB frame for camera {name!r} must be a numpy array."
                raise TypeError(msg)
            if not np.issubdtype(image.dtype, np.number):
                msg = f"RGB frame for camera {name!r} must have a numeric dtype, got {image.dtype}."
                raise TypeError(msg)

            if name not in image_metadata:
                image_metadata[name] = (image.shape, image.dtype)
            else:
                expected_shape, expected_dtype = image_metadata[name]
                if image.shape != expected_shape or image.dtype != expected_dtype:
                    msg = (
                        f"RGB frames for camera {name!r} changed shape or dtype during blank-image capture: "
                        f"expected {expected_shape}/{expected_dtype}, got {image.shape}/{image.dtype}."
                    )
                    raise ValueError(msg)
            captured_images[name].append(image.copy())

    blank_camera_dict: dict[str, np.ndarray] = {}
    for name in names:
        _, dtype = image_metadata[name]
        image_stack = np.stack(captured_images[name], axis=0)
        average = np.mean(image_stack, axis=0, dtype=np.float64)
        if np.issubdtype(dtype, np.integer):
            dtype_limits = np.iinfo(dtype)
            average = np.clip(np.rint(average), dtype_limits.min, dtype_limits.max)
        blank_camera_dict[name] = average.astype(dtype)
    return blank_camera_dict
