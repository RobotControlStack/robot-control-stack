import numpy as np

from rcs.camera.interface import CameraFrame, DataFrame, Frame, FrameSet
from rcs.camera.utils import capture_blank_camera_images


class _FakeCameraSet:
    def __init__(self, images: list[np.ndarray]):
        self.images = images
        self.sample_index = -1
        self.returned_images: list[np.ndarray] = []

    def clear_buffer(self):
        self.sample_index += 1

    def get_latest_frames(self) -> FrameSet:
        image = self.images[self.sample_index]
        self.returned_images.append(image)
        return FrameSet(
            frames={
                "digit": Frame(
                    camera=CameraFrame(color=DataFrame(data=image)),
                )
            },
            avg_timestamp=float(self.sample_index),
        )


def test_capture_blank_camera_images_averages_fresh_frames():
    images = [np.full((2, 3, 3), value, dtype=np.uint8) for value in range(50)]
    camera_set = _FakeCameraSet(images)

    blank_images = capture_blank_camera_images(camera_set, ["digit"])

    assert camera_set.sample_index == 49
    assert len(camera_set.returned_images) == 50
    assert blank_images["digit"].dtype == np.uint8
    np.testing.assert_array_equal(blank_images["digit"], np.full((2, 3, 3), 24, dtype=np.uint8))


def test_capture_blank_camera_images_supports_configurable_sample_count():
    images = [
        np.full((1, 1, 3), 1.0, dtype=np.float32),
        np.full((1, 1, 3), 3.0, dtype=np.float32),
    ]
    camera_set = _FakeCameraSet(images)

    blank_images = capture_blank_camera_images(camera_set, ["digit"], num_frames=2)

    assert blank_images["digit"].dtype == np.float32
    np.testing.assert_array_equal(blank_images["digit"], np.full((1, 1, 3), 2.0, dtype=np.float32))
