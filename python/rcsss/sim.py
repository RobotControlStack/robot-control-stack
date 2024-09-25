import multiprocessing as mp
import uuid
from logging import getLogger
from os import PathLike
from pathlib import Path
from typing import Optional

import mujoco as mj
from rcsss._core.sim import FR3, FHConfig, FHState, FR3Config, FR3State, FrankaHand
from rcsss._core.sim import Sim as _Sim
from rcsss._core.sim import open_gui_window as _open_gui_window

__all__ = ["Sim", "FR3", "FR3Config", "FR3State", "FHConfig", "FHState", "FrankaHand"]

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
            self._gui_uuid = str(uuid.uuid4())
            self.start_gui_server(self._gui_uuid)
        self._mp_context.Process(target=_start_gui, args=(self._gui_uuid,)).start()
