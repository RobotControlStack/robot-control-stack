"""Robot control stack python bindings."""

import pathlib
import site

from rcsss import camera, control, envs, sim
from rcsss._core import __version__, common, hw

scenes = {
    path.stem: path / "scene.mjb" for path in (pathlib.Path(site.getsitepackages()[0]) / "rcsss" / "scenes").glob("*")
}

__all__ = ["__doc__", "__version__", "common", "hw", "sim", "camera", "scenes", "control", "envs"]
