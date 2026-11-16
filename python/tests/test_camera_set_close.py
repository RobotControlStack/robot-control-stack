"""HardwareCameraSet.close() must stop the polling thread before the cameras are closed (no hardware needed)."""

import threading
import time

import numpy as np
from rcs._core.common import BaseCameraConfig
from rcs.camera.hw import HardwareCameraSet
from rcs.camera.interface import CameraFrame, DataFrame, Frame


class _FakeCamera:
    """Polls slowly and fails like rcs_zed once it is closed (its devices are gone)."""

    def __init__(self):
        self.devices = {}
        self._cfg = BaseCameraConfig("fake", 60, 4, 3)

    @property
    def camera_names(self):
        return ["front"]

    def config(self, camera_name):
        return self._cfg

    def open(self):
        self.devices["front"] = object()

    def close(self):
        self.devices.clear()

    def calibrate(self):
        return True

    def poll_frame(self, camera_name):
        time.sleep(0.03)  # a poll takes most of the frame period, close() lands inside one
        _ = self.devices[camera_name]  # KeyError after close, like rcs_zed
        data = DataFrame(data=np.zeros((3, 4, 3), dtype=np.uint8), timestamp=time.time())
        return Frame(camera=CameraFrame(color=data), avg_timestamp=data.timestamp)


def test_close_stops_polling_before_closing_the_cameras():
    errors = []
    previous = threading.excepthook
    threading.excepthook = lambda args: errors.append(args.exc_value)
    try:
        cameras = HardwareCameraSet([_FakeCamera()], warm_up_disposal_frames=0, max_buffer_frames=10)
        cameras.start(warm_up=False)
        time.sleep(0.2)
        cameras.close()
        assert not cameras.running
        time.sleep(0.05)
    finally:
        threading.excepthook = previous
    assert errors == []
