from digit_interface.digit import Digit
from pydantic import Field
from rcs.camera.hw import BaseHardwareCameraSet, HWCameraSetConfig
from rcs.camera.interface import BaseCameraConfig, CameraFrame, DataFrame, Frame


class DigitConfig(HWCameraSetConfig):
    """
    Configuration for the DIGIT device.
    This class is used to define the settings for the DIGIT device.
    """

    cameras: dict[str, BaseCameraConfig] = Field(
        default_factory=lambda: {"camera1": BaseCameraConfig(identifier="D21182")}
    )
    stream: dict = Digit.STREAMS["QVGA"]  # QVGA resolution or VGA resolution
    resolution_width: int = stream["resolution"]["width"]
    resolution_height: int = stream["resolution"]["height"]
    # for QVGA resolution, 60 fps is the default or 30 fps
    # for VGA resolution, 30 fps is the default or 15 fps
    frame_rate: int = stream["fps"]["30fps"]  # 30 fps

    @property
    def name_to_identifier(self):
        return dict(self.cameras)


class DigitCam(BaseHardwareCameraSet):
    """
    This module provides an interface to interact with the DIGIT device.
    It allows for connecting to the device, changing settings, and retrieving information.
    """

    def __init__(self, cfg: DigitConfig):
        self._cfg = cfg
        super().__init__()
        self._cameras: dict[str, Digit] = {}
        # Initialize the digit interface
        self.initalize(self.config)

    def initalize(self, cfg: HWCameraSetConfig):
        """
        Initialize the digit interface with the given configuration.
        :param cfg: Configuration for the DIGIT device.
        """
        for name, serial in cfg.cameras.items():
            digit = Digit(serial.identifier, name)
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

    @config.setter
    def config(self, cfg: DigitConfig) -> None:
        self._cfg = cfg
