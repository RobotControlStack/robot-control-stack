import mujoco as mj
from rcsss._core.sim import _FR3, FR3Config, FR3State

__all__ = ["FR3", "FR3Config", "FR3State"]


class FR3:
    def __init__(self, mjmdl: str, rlmdl: str, render: bool = True):
        self.model = mj.MjModel.from_xml_path("models/mjcf/scene.xml")
        self.data = mj.MjData(self.model)
        self._robot = _FR3(self.model._address, self.data._address, rlmdl, render)

    def __getattr__(self, name):
        try:
            return getattr(self._robot, name)
        except AttributeError:
            msg = f"Neither FR3 nor _FR3 have an attribute named {name}"
            raise AttributeError(msg)
