import atexit
import time
import multiprocessing as mp
import uuid
from copy import deepcopy
from logging import getLogger
from multiprocessing import Event
from os import PathLike
from pathlib import Path
from tempfile import NamedTemporaryFile
from threading import Thread
from typing import Optional

import mujoco as mj
import mujoco.viewer
import rcs.egl_bootstrap
from rcs._core.sim import GuiClient as _GuiClient
from rcs._core.sim import Sim as _Sim
from rcs._core.sim import (
    SimGripper,
    SimGripperConfig,
    SimGripperState,
    SimRobot,
    SimRobotConfig,
    SimRobotState,
)

__all__ = ["Sim", "SimRobot", "SimRobotConfig", "SimRobotState", "SimGripper", "SimGripperConfig", "SimGripperState"]

logger = getLogger(__name__)


# Target frames per second
FPS = 60.0
FRAME_DURATION = 1.0 / FPS


def gui_loop(model_bytes: bytes, close_event: Event, gui_uuid: str):
    with NamedTemporaryFile(mode="wb") as f:
        f.write(model_bytes)
        model = mujoco.MjModel.from_binary_path(f.name)
    data = mujoco.MjData(model)
    gui_client = _GuiClient(gui_uuid)
    gui_client.set_model_and_data(model._address, data._address)
    mujoco.mj_step(model, data)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        next_frame_time = time.perf_counter()
        while not close_event.is_set():
            mujoco.mj_step(model, data)
            viewer.sync()
            gui_client.sync()
            next_frame_time += FRAME_DURATION
            sleep_duration = next_frame_time - time.perf_counter()
            if sleep_duration > 0:
                time.sleep(sleep_duration)
            else:
                next_frame_time = time.perf_counter()


class Sim(_Sim):
    def __init__(self, mjmdl: str | PathLike):
        mjmdl = Path(mjmdl)
        if mjmdl.suffix == ".xml":
            self.model = mj.MjModel.from_xml_path(str(mjmdl))
        elif mjmdl.suffix == ".mjb":
            self.model = mj.MjModel.from_binary_path(str(mjmdl))
        else:
            msg = f"Filetype {mjmdl.suffix} is unknown"
            logger.error(msg)
        self.data = mj.MjData(self.model)
        super().__init__(self.model._address, self.data._address)
        self._gui_uuid: Optional[str] = None
        self._mp_context = mp.get_context("spawn")
        self._gui_client: Optional[GuiClient] = None
        self._gui_thread: Optional[Thread] = None

    def open_gui(self):
        if self._gui_uuid is None:
            self._gui_uuid = "rcs_" + str(uuid.uuid4())
            self._start_gui_server(self._gui_uuid)
            atexit.register(self._stop_gui_server)
        if self._gui_client is None:
            self._gui_client = _GuiClient(self._gui_uuid)
            model_bytes = self._gui_client.get_model_bytes()
            ctx = mp.get_context("spawn")
            self._stop_event = ctx.Event()
            self._gui_process = ctx.Process(
                target=gui_loop,
                args=(model_bytes, self._stop_event, self._gui_uuid),
            )
            self._gui_process.start()

    def __del__(self):
        self._stop_event.set()
        self._gui_process.join()
        self._stop_gui_server()
