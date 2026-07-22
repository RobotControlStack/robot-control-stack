import time
import numpy as np

from digit_interface.digit import Digit
from rcs._core.common import BaseCameraConfig
from rcs.camera.hw import HardwareCamera
from rcs.camera.interface import CameraFrame, DataFrame, Frame


class DigitCam(HardwareCamera):
    """
    This module provides an interface to interact with the DIGIT device.
    It allows for connecting to the device, changing settings, and retrieving information.
    """

    def __init__(
        self,
        cameras: dict[str, BaseCameraConfig],
        harsh_gradient_threshold: float = 50.0,
    ):
        self.cameras = cameras
        self._camera_names = list(self.cameras.keys())
        self._cameras: dict[str, Digit] = {}
        self._harsh_gradient_threshold = harsh_gradient_threshold

    def open(self):
        """
        Initialize the digit interface with the given configuration.
        :param cfg: Configuration for the DIGIT device.
        """
        for name, camera in self.cameras.items():
            digit = Digit(camera.identifier, name)
            digit.connect()

            qvga_res = Digit.STREAMS["QVGA"]
            digit.set_resolution(qvga_res)

            fps_30 = Digit.STREAMS["QVGA"]["fps"]["30fps"]
            digit.set_fps(fps_30)

            self._cameras[name] = digit

    @property
    def camera_names(self) -> list[str]:
        """Returns the names of the cameras in this set."""
        return self._camera_names

    def poll_frame(self, camera_name: str) -> Frame:
        """Polls the frame from the camera with the given name."""
        digit = self._cameras[camera_name]
        # t0 = time.perf_counter()
        frame = digit.get_frame()
        # t1 = time.perf_counter()
        # if self._contains_harsh_gradient(frame):
        #     print(
        #         f"[{camera_name}] get_frame_dt={t1 - t0:.8f}s "
        #         f"shape={frame.shape} dtype={frame.dtype}"
        #     )
        # rgb to bgr as expected by opencv
        # DIGIT cameras are not calibrated, but their observations still need
        # concrete calibration-shaped values so Arrow can infer a schema when
        # recording.  These placeholders are not used for tactile images.
        color = DataFrame(
            data=frame[:, :, ::-1].copy(),
            intrinsics=np.eye(3, 4, dtype=np.float64),
            extrinsics=np.eye(4, dtype=np.float64),
        )
        cf = CameraFrame(color=color)

        return Frame(camera=cf)

    def _contains_harsh_gradient(self, frame: np.ndarray) -> bool:
        """Return True when frame has a strong horizontal or vertical gradient."""
        gray = frame.astype(np.float32).mean(axis=2)
        if gray.shape[0] < 2 or gray.shape[1] < 2:
            return False

        h_grad = np.abs(np.diff(gray, axis=1))
        v_grad = np.abs(np.diff(gray, axis=0))
        max_grad = max(h_grad.max(), v_grad.max())
        return max_grad >= self._harsh_gradient_threshold

    def close(self):
        """
        Closes the connection to the DIGIT device.
        """
        for digit in self._cameras.values():
            digit.disconnect()
        self._cameras = {}

    def config(self, camera_name) -> BaseCameraConfig:
        return self.cameras[camera_name]

    def calibrate(self) -> bool:
        """No calibration needed for DIGIT cameras."""
        return True
