import atexit
import multiprocessing as mp
import uuid
from logging import getLogger
from os import PathLike
from pathlib import Path
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
            model_byte = self._gui_client.get_model_bytes()
            # TODO: load model and create data, pass to a process which calls
            # step & sync in a loop

    def __del__(self):
        self._stop_gui_server()
