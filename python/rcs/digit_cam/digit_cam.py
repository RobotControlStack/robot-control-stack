from rcs.camera.hw import BaseHardwareCameraSet, BaseCameraSetConfig, HWCameraSetConfig
from rcs.camera.interface import Frame, DataFrame, CameraFrame
from digit_interface.digit import Digit


class DigitConfig(HWCameraSetConfig):
    """
    Configuration for the DIGIT device.
    This class is used to define the settings for the DIGIT device.
    """
    cameras: dict[str, str] ={
            "camera1": "D21182",
            }
    stream: dict = Digit.STREAMS["QVGA"] # QVGA resolution or VGA resolution
    resolution_width: int = stream["resolution"]["width"]
    resolution_height: int = stream["resolution"]["height"]
    # for QVGA resolution, 60 fps is the default or 30 fps
    # for VGA resolution, 30 fps is the default or 15 fps
    frame_rate: int = stream["fps"]["30fps"]  # 30 fps

    @property
    def name_to_identifier(self):
        return {key: serial  for key, serial in self.cameras.items()}

class DigitCam(BaseHardwareCameraSet):
    """
    This module provides an interface to interact with the DIGIT device.
    It allows for connecting to the device, changing settings, and retrieving information.
    """

    def __init__(self, cfg: DigitConfig):
        self.config = cfg
        super().__init__()
        self._cameras: dict[str, Digit] = {}
        # Initialize the digit interface
        self.initalize(self.config)

    def initalize(self, cfg: BaseCameraSetConfig):
        """
        Initialize the digit interface with the given configuration.
        :param cfg: Configuration for the DIGIT device.
        """
        for name, serial in cfg.cameras.items():
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

    def config(self) -> HWCameraSetConfig:
        return self.config