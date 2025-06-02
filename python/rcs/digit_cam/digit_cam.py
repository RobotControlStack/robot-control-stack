from digit_interface.digit import Digit
from pydantic import Field
from rcs.camera.hw import BaseHardwareCameraSet, HWCameraSetConfig
from rcs.camera.interface import BaseCameraConfig, CameraFrame, DataFrame, Frame


class DigitConfig(HWCameraSetConfig):
    """
    Configuration for the DIGIT device.
    This class is used to define the settings for the DIGIT device.
    """

    cameras: dict[str, BaseCameraConfig] = Field(default={})
    stream_name: str = "QVGA"  # options: "QVGA" (60 and 30 fps), "VGA" (30 and 15 fps)

    @property
    def resolution_width(self) -> int:
        return Digit.STREAMS[self.stream_name]["resolution"]["width"]

    @property
    def resolution_height(self) -> int:
        return Digit.STREAMS[self.stream_name]["resolution"]["height"]

    


class DigitCam(BaseHardwareCameraSet):
    """
    This module provides an interface to interact with the DIGIT device.
    It allows for connecting to the device, changing settings, and retrieving information.
    """

    def __init__(self, cfg: DigitConfig):
        self._cfg = cfg
        super().__init__()
        self._cameras: dict[str, Digit] = {}
        self.initalize(self.config)

    def initalize(self, cfg: HWCameraSetConfig):
        """
        Initialize the digit interface with the given configuration.
        :param cfg: Configuration for the DIGIT device.
        """
        for name, serial in cfg.name_to_identifier.items():
            digit = Digit(serial, name)
            digit.connect()
            self._cameras[name] = digit

    def _poll_frame(self, camera_name: str) -> Frame:
        """Polls the frame from the camera with the given name."""
        digit = self._cameras[camera_name]
        frame = digit.get_frame()
        color = DataFrame(data=frame)
        # rgb to bgr as expected by opencv
        # color = DataFrame(data=frame[:, :, ::-1])
        cf = CameraFrame(color=color)

        return Frame(camera=cf)

    @property
    def config(self) -> DigitConfig:
        return self._cfg
