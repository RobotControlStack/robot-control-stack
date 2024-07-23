"""Robot control stack python bindings."""

import pathlib

from rcsss import camera, desk, sim
from rcsss._core import __version__, common, hw

scenes = {path.stem: path / "scene.mjb" for path in (pathlib.Path(__file__).parent.resolve() / "scenes").glob("*")}

__all__ = ["__doc__", "__version__", "common", "hw", "sim", "desk", "camera", "scenes"]
