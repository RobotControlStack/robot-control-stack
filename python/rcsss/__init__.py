"""Robot control stack python bindings."""

import pathlib
import site

from rcsss import camera, control, envs, sim
from rcsss._core import __version__, common, hw

scenes = {
    path.stem: {
        "xml": path / "scene.xml" if (path / "scene.xml").exists() else None,
        "mjb": path / "scene.mjb" if (path / "scene.mjb").exists() else None,
        "urdfs": {
            urdf.stem: urdf for urdf in path.glob("*.urdf")
        } if tuple(path.glob("*.urdf")) else None
    }
    for path in (pathlib.Path(site.getsitepackages()[0]) / "rcsss" / "scenes").glob("*")
}

__all__ = ["__doc__", "__version__", "common", "hw", "sim", "camera", "scenes", "control", "envs"]
