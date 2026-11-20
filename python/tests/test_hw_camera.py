import time
from datetime import datetime

import numpy as np
from rcs._core.common import BaseCameraConfig
from rcs.camera.hw import HardwareCameraSet
from rcs.camera.interface import CameraFrame, DataFrame, Frame


class _FakeCamera:
    @property
    def camera_names(self) -> list[str]:
        return ["cam"]

    def open(self):
        pass

    def close(self):
        pass

    def config(self, camera_name: str) -> BaseCameraConfig:
        return BaseCameraConfig(identifier=camera_name, frame_rate=100, resolution_width=4, resolution_height=3)

    def poll_frame(self, camera_name: str) -> Frame:
        ts = time.time()
        color = DataFrame(data=np.zeros((3, 4, 3), dtype=np.uint8), timestamp=ts)
        return Frame(camera=CameraFrame(color=color), avg_timestamp=ts)

    def calibrate(self) -> bool:
        return True


def test_timestamp_lookup_before_the_buffer_is_full():
    cameras = HardwareCameraSet([_FakeCamera()], warm_up_disposal_frames=0, max_buffer_frames=1000)
    cameras.start(warm_up=False)
    try:
        cameras.wait_for_frames()
    finally:
        cameras.stop()
    assert 0 < cameras.buffer_size() < cameras.max_buffer_frames
    # only the filled slots are searched
    assert cameras.get_timestamp_frames(datetime.fromtimestamp(0)) is None
    assert cameras.get_timestamp_frames(datetime.now()) is cameras.get_latest_frames()
