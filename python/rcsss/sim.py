import mujoco as mj
from rcsss._core.sim import FR3
from rcsss._core.sim import Sim as _Sim

__all__ = ["Sim"]


class Sim(_Sim):
    def __init__(self, mjmdl: str):
        self.model = mj.MjModel.from_xml_path(mjmdl)
        self.data = mj.MjData(self.model)
        super().__init__(self.model._address, self.data._address)


del mj
