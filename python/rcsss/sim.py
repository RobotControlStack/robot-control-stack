import mujoco as mj
from pathlib import Path
from os import PathLike
from rcsss._core.sim import FR3, FHConfig, FHState, FR3Config, FR3State, FrankaHand
from rcsss._core.sim import Sim as _Sim

__all__ = ["Sim", "FR3", "FR3Config", "FR3State", "FHConfig", "FHState", "FrankaHand"]


class Sim(_Sim):
    def __init__(self, mjmdl: str | PathLike):
        mjmdl = Path(mjmdl)
        if mjmdl.suffix == ".xml":
            self.model = mj.MjModel.from_xml_path(str(mjmdl))
        elif mjmdl.suffix == ".mjb":
            self.model = mj.MjModel.from_binary_path(str(mjmdl))
        else:
            print(f"Filetype {mjmdl.suffix} is unknown")
        self.data = mj.MjData(self.model)
        super().__init__(self.model._address, self.data._address)
