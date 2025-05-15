"""Robot control stack python bindings."""

import pathlib
import site

from rcs import camera, control, envs, sim
from rcs._core import __version__, common, hw

# available mujoco scenes
scenes: dict[str, dict[str, pathlib.Path]] = {
    path.stem: {
        "mjb": path / "scene.mjb",
        "urdf": path / "fr3.urdf",
    }
    for path in (pathlib.Path(site.getsitepackages()[0]) / "rcs" / "scenes").glob("*")
}

# make submodules available
__all__ = ["__doc__", "__version__", "common", "hw", "sim", "camera", "scenes", "control", "envs"]
