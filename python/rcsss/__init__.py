"""Robot control stack python bindings."""

import pathlib
import site
from typing import TypedDict

from rcsss import camera, control, envs, sim
from rcsss._core import __version__, common, hw


class ScenesDict(TypedDict):
    xml: pathlib.Path
    mjb: pathlib.Path
    urdfs: dict[str, pathlib.Path]


scenes: dict[str, ScenesDict] = {
    path.stem: {
        "xml": path / "scene.xml",
        "mjb": path / "scene.mjb",
        "urdfs": {urdf.stem: urdf for urdf in path.glob("*.urdf")},
    }
    for path in (pathlib.Path(site.getsitepackages()[0]) / "rcsss" / "scenes").glob("*")
}
for scene in scenes.values():
    assert scene["xml"].exists()
    assert scene["mjb"].exists()
    for urdf in scene["urdfs"].values():
        assert urdf.exists()
del scene

__all__ = ["__doc__", "__version__", "common", "hw", "sim", "camera", "scenes", "control", "envs"]
