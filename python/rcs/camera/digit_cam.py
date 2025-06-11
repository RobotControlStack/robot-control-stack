from digit_interface.digit import Digit
from rcs._core.common import BaseCameraConfig
from rcs.camera.hw import HardwareCamera
from rcs.camera.interface import CameraFrame, DataFrame, Frame


class DigitCam(HardwareCamera):
    """
    This module provides an interface to interact with the DIGIT device.
    It allows for connecting to the device, changing settings, and retrieving information.
    """

    def __init__(self, cameras: dict[str, BaseCameraConfig]):
        self.cameras = cameras
        self._camera_names = list(self.cameras.keys())
        self._cameras: dict[str, Digit] = {}

    def open(self):
        """
        Initialize the digit interface with the given configuration.
        :param cfg: Configuration for the DIGIT device.
        """
        for name, camera in self.cameras.items():
            digit = Digit(camera.identifier, name)
            digit.connect()
            self._cameras[name] = digit

    @property
    def camera_names(self) -> list[str]:
        """Returns the names of the cameras in this set."""
        return self._camera_names

    def poll_frame(self, camera_name: str) -> Frame:
        """Polls the frame from the camera with the given name."""
        digit = self._cameras[camera_name]
        frame = digit.get_frame()
        color = DataFrame(data=frame)
        # rgb to bgr as expected by opencv
        # color = DataFrame(data=frame[:, :, ::-1])
        cf = CameraFrame(color=color)

        return Frame(camera=cf)

    def close(self):
        """
        Closes the connection to the DIGIT device.
        """
        for digit in self._cameras.values():
            digit.disconnect()
        self._cameras = {}

    def config(self, camera_name) -> BaseCameraConfig:
        return self.cameras[camera_name]
