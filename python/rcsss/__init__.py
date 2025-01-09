"""Robot control stack python bindings."""

import pathlib
import site
from typing import TypedDict
from warnings import warn

from gymnasium import register
from rcsss import camera, control, envs, sim
from rcsss._core import __version__, common, hw
from rcsss.envs.factories import FR3Real, FR3SimplePickUpSim


class SceneData(TypedDict):
    xml: pathlib.Path
    mjb: pathlib.Path
    urdfs: dict[str, pathlib.Path]


scenes: dict[str, SceneData] = {
    path.stem: {
        "xml": path / "scene.xml",
        "mjb": path / "scene.mjb",
        "urdfs": {urdf.stem: urdf for urdf in path.glob("*.urdf")},
    }
    for path in (pathlib.Path(site.getsitepackages()[0]) / "rcsss" / "scenes").glob("*")
}

for _ in scenes.values():
    if not _["xml"].exists():
        warn(f"Missing XML scene file {_['xml']}", stacklevel=2)
    if not _["mjb"].exists():
        warn(f"Missing mjb scene file {_['mjb']}", stacklevel=2)
    for __ in _["urdfs"].values():
        if not __.exists():
            warn(f"Missing urdf file {__}", stacklevel=2)

# make submodules available
__all__ = ["__doc__", "__version__", "common", "hw", "sim", "camera", "scenes", "control", "envs"]

# register gymnasium environments
register(
    id="rcs/SimplePickUpSim-v0",
    entry_point=FR3SimplePickUpSim(),
)
register(
    id="rcs/FR3Real-v0",
    entry_point=FR3Real(),
)
