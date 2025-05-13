from rcsss.camera.hw import HWCameraSetConfig
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