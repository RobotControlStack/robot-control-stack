import multiprocessing as mp
import uuid
from logging import getLogger
from os import PathLike
from pathlib import Path
from typing import Optional

import mujoco as mj
from rcsss._core.sim import Sim as _Sim
from rcsss._core.sim import (
    SimGripper,
    SimGripperConfig,
    SimGripperState,
    SimRobot,
    SimRobotConfig,
    SimRobotState,
)
from rcsss._core.sim import open_gui_window as _open_gui_window

__all__ = ["Sim", "SimRobot", "SimRobotConfig", "SimRobotState", "SimGripper", "SimGripperConfig", "SimGripperState"]

logger = getLogger(__name__)


def _start_gui(identifier: str):
    _open_gui_window(identifier)


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

    def open_gui(self):
        if self._gui_uuid is None:
            self._gui_uuid = "rcsss_" + str(uuid.uuid4())
            self._start_gui_server(self._gui_uuid)
        self._mp_context.Process(target=_start_gui, args=(self._gui_uuid,), daemon=True).start()

    def __del__(self):
        self._stop_gui_server()
