import atexit
import multiprocessing as mp
import uuid
from logging import getLogger
from multiprocessing.synchronize import Event as EventClass
from os import PathLike
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Optional

import mujoco as mj
import mujoco.viewer
import numpy as np
from rcs._core.sim import DynamicJointSchema as _DynamicJointSchema
from rcs._core.sim import DynamicJointState as _DynamicJointState
from rcs._core.sim import GuiClient as _GuiClient
from rcs._core.sim import Sim as _Sim
from rcs.sim import SimConfig, egl_bootstrap
from rcs.utils import SimpleFrameRate

egl_bootstrap.bootstrap()
logger = getLogger(__name__)


# Target frames per second
FPS = 60


def gui_loop(gui_uuid: str, close_event):
    frame_rate = SimpleFrameRate(FPS, "gui_loop")
    gui_client = _GuiClient(gui_uuid)
    model_bytes = gui_client.get_model_bytes()
    with NamedTemporaryFile(mode="wb") as f:
        f.write(model_bytes)
        model = mujoco.MjModel.from_binary_path(f.name)
    data = mujoco.MjData(model)
    gui_client.set_model_and_data(model._address, data._address)
    mujoco.mj_step(model, data)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while not close_event.is_set():
            mujoco.mj_step(model, data)
            viewer.sync()
            gui_client.sync()
            frame_rate()


class Sim(_Sim):
    def __init__(self, mjmdl: str | PathLike, cfg: SimConfig | None = None):
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
        self._mp_context = mp.get_context("spawn")
        self._gui_uuid: Optional[str] = None
        self._gui_client: Optional[_GuiClient] = None
        self._gui_process: Optional[mp.context.SpawnProcess] = None
        self._stop_event: Optional[EventClass] = None
        if cfg is not None:
            self.set_config(cfg)

    def get_state_spec(self) -> dict[str, list[str] | list[int]]:
        return self.get_dynamic_joint_schema()

    def get_state_size(self, spec: dict[str, list[str] | list[int]] | None = None) -> int:
        state_spec = self.get_state_spec() if spec is None else spec
        qpos_size = sum(int(value) for value in state_spec["qpos_sizes"])
        qvel_size = sum(int(value) for value in state_spec["qvel_sizes"])
        return qpos_size + qvel_size

    def get_state(self, spec: dict[str, list[str] | list[int]] | None = None) -> np.ndarray:
        del spec
        dynamic_joint_state = self.get_dynamic_joint_state()
        return np.concatenate((dynamic_joint_state["qpos"], dynamic_joint_state["qvel"]))

    def set_state(
        self,
        state: np.ndarray,
        spec: dict[str, list[str] | list[int]] | None = None,
    ):
        state_spec = self.get_state_spec() if spec is None else spec
        state_array = np.asarray(state, dtype=np.float64)
        expected_size = self.get_state_size(state_spec)
        if state_array.shape != (expected_size,):
            msg = f"Expected state with shape ({expected_size},), got {state_array.shape}."
            raise ValueError(msg)

        qpos_size = sum(int(value) for value in state_spec["qpos_sizes"])
        dynamic_joint_state = {
            "qpos": state_array[:qpos_size],
            "qvel": state_array[qpos_size:],
        }
        self.set_dynamic_joint_state(state_spec, dynamic_joint_state)

    def get_dynamic_joint_schema(self) -> dict[str, list[str] | list[int]]:
        schema = super().get_dynamic_joint_schema()
        return {
            "joint_names": list(schema.joint_names),
            "joint_types": list(schema.joint_types),
            "qpos_sizes": list(schema.qpos_sizes),
            "qvel_sizes": list(schema.qvel_sizes),
        }

    def get_dynamic_joint_state(self) -> dict[str, np.ndarray]:
        state = super().get_dynamic_joint_state()
        return {
            "qpos": np.asarray(state.qpos, dtype=np.float64),
            "qvel": np.asarray(state.qvel, dtype=np.float64),
        }

    def set_dynamic_joint_state(
        self,
        schema: dict[str, list[str] | list[int]],
        state: dict[str, np.ndarray],
    ):
        dynamic_joint_schema = _DynamicJointSchema()
        dynamic_joint_schema.joint_names = list(schema["joint_names"])
        dynamic_joint_schema.joint_types = [int(value) for value in schema["joint_types"]]
        dynamic_joint_schema.qpos_sizes = [int(value) for value in schema["qpos_sizes"]]
        dynamic_joint_schema.qvel_sizes = [int(value) for value in schema["qvel_sizes"]]

        dynamic_joint_state = _DynamicJointState()
        dynamic_joint_state.qpos = np.asarray(state["qpos"], dtype=np.float64)
        dynamic_joint_state.qvel = np.asarray(state["qvel"], dtype=np.float64)
        super().set_dynamic_joint_state(dynamic_joint_schema, dynamic_joint_state)

    def close_gui(self):
        if self._stop_event is not None:
            self._stop_event.set()
        if self._gui_process is not None:
            self._gui_process.join()
        self._stop_gui_server()

    def open_gui(self):
        if self._gui_uuid is None:
            self._gui_uuid = "rcs_" + str(uuid.uuid4())
            self._start_gui_server(self._gui_uuid)
        if self._gui_client is None:
            ctx = mp.get_context("spawn")
            self._stop_event = ctx.Event()
            self._gui_process = ctx.Process(
                target=gui_loop,
                args=(self._gui_uuid, self._stop_event),
            )
            self._gui_process.start()
        atexit.register(self.close_gui)
