import atexit
import contextlib
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
from rcs._core.sim import GuiClient as _GuiClient
from rcs._core.sim import Sim as _Sim
from rcs.sim import SimConfig, egl_bootstrap
from rcs.sim.composer import ModelComposer
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
    STATE_SPEC = mj.mjtState.mjSTATE_INTEGRATION

    def __init__(self, mjmdl: str | PathLike | ModelComposer, cfg: SimConfig | None = None):
        self.model, self.data = self.get_model_data(mjmdl)
        super().__init__(self.model._address, self.data._address)
        self._mp_context = mp.get_context("spawn")
        self._gui_uuid: Optional[str] = None
        self._gui_client: Optional[_GuiClient] = None
        self._gui_process: Optional[mp.context.SpawnProcess] = None
        self._stop_event: Optional[EventClass] = None
        self._gui_atexit_registered = False
        if cfg is not None:
            self.set_config(cfg)

    def get_model_data(self, mjmdl: str | PathLike | ModelComposer) -> tuple[mj.MjModel, mj.MjData]:
        if isinstance(mjmdl, ModelComposer):
            model = mjmdl.get_model()
        else:
            mjmdl = Path(mjmdl)
            if mjmdl.suffix == ".xml":
                model = mj.MjModel.from_xml_path(str(mjmdl))
            elif mjmdl.suffix == ".mjb":
                model = mj.MjModel.from_binary_path(str(mjmdl))
            else:
                msg = f"Filetype {mjmdl.suffix} is unknown"
                logger.error(msg)
        data = mj.MjData(model)
        return model, data

    def set_model_data(self, model: mj.MjModel, data: mj.MjData):
        super().set_model_data(model._address, data._address)

    def instantiate_new_sim(self, mjmdl: str | PathLike | ModelComposer):
        self.model, self.data = self.get_model_data(mjmdl)
        self.set_model_data(self.model, self.data)

    def get_state_spec(self) -> int:
        return int(self.STATE_SPEC)

    def get_state_size(self, spec: int | None = None) -> int:
        state_spec = self.STATE_SPEC if spec is None else mj.mjtState(spec)
        return mj.mj_stateSize(self.model, state_spec)

    def get_state(self, spec: int | None = None) -> np.ndarray:
        state_spec = self.STATE_SPEC if spec is None else mj.mjtState(spec)
        state = np.empty(self.get_state_size(int(state_spec)), dtype=np.float64)
        mj.mj_getState(self.model, self.data, state, state_spec)
        return state

    def set_state(self, state: np.ndarray, spec: int | None = None):
        state_spec = self.STATE_SPEC if spec is None else mj.mjtState(spec)
        state_array = np.asarray(state, dtype=np.float64)
        expected_size = self.get_state_size(int(state_spec))
        if state_array.shape != (expected_size,):
            msg = (
                f"Expected MuJoCo state with shape ({expected_size},), "
                f"got {state_array.shape} for spec {int(state_spec)}."
            )
            raise ValueError(msg)
        mj.mj_setState(self.model, self.data, state_array, state_spec)
        mj.mj_forward(self.model, self.data)

    def close_gui(self):
        if self._stop_event is not None:
            self._stop_event.set()
        if self._gui_process is not None:
            self._gui_process.join()
        self._stop_gui_server()
        self._gui_uuid = None
        self._gui_client = None
        self._gui_process = None
        self._stop_event = None
        if self._gui_atexit_registered:
            with contextlib.suppress(ValueError):
                atexit.unregister(self.close_gui)
            self._gui_atexit_registered = False

    def open_gui(self):
        if self._gui_uuid is None:
            self._gui_uuid = "rcs_" + str(uuid.uuid4())
            self._start_gui_server(self._gui_uuid)
        if self._gui_process is None or not self._gui_process.is_alive():
            self._stop_event = self._mp_context.Event()
            self._gui_process = self._mp_context.Process(
                target=gui_loop,
                args=(self._gui_uuid, self._stop_event),
            )
            self._gui_process.start()
        if not self._gui_atexit_registered:
            atexit.register(self.close_gui)
            self._gui_atexit_registered = True
