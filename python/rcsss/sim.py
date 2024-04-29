import mujoco as mj
from rcsss._core.sim import FR3Config, FR3State
from rcsss._core.sim import FR3 as _FR3

__all__ = ["FR3", "FR3Config", "FR3State"]


class FR3(_FR3):
    def __init__(self, mjmdl: str, rlmdl: str, render: bool = True):
        self.model = mj.MjModel.from_xml_path(mjmdl)
        self.data = mj.MjData(self.model)
        super().__init__(self.model._address, self.data._address, rlmdl, render)
