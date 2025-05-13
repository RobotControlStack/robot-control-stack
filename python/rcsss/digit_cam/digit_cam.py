from rcsss.camera.hw import BaseHardwareCameraSet, BaseCameraSetConfig, HWCameraSetConfig
from rcsss.camera.interface import Frame, DataFrame, CameraFrame
from rcsss.digit_cam.interface import DigitConfig
from digit_interface.digit import Digit

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