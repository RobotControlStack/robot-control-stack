import mujoco  # noqa: F401  -- loads libmujoco before _core (TAM ideal-model gravity)
from rcs_fr3._core import __version__, hw

from . import configs, desk, envs

__all__ = [
    "configs",
    "desk",
    "hw",
    "envs",
    "__version__",
]
