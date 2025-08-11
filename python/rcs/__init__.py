"""Robot control stack python bindings."""

import os
import pathlib
import site

from gymnasium import register
from rcs._core import __version__, common
from rcs.envs.creators import FR3SimplePickUpSimEnvCreator

from rcs import camera, envs, hand, sim

# available mujoco scenes
scenes: dict[str, dict[str, pathlib.Path]] = {
    path.stem: {
        "mjb": path / "scene.mjb",
        "urdf": os.path.join(os.path.dirname(__file__), "..", "..", "assets", "scenes", path.stem, "robot.urdf"),
        "mjcf_scene": os.path.join(os.path.dirname(__file__), "..", "..", "assets", "scenes", path.stem, "scene.xml"),
        "mjcf_robot": os.path.join(os.path.dirname(__file__), "..", "..", "assets", "scenes", path.stem, "robot.xml"),
    }
    for path in (pathlib.Path(site.getsitepackages()[0]) / "rcs" / "scenes").glob("*")
}

# make submodules available
__all__ = ["__doc__", "__version__", "common", "sim", "camera", "scenes", "envs", "hand"]

# register gymnasium environments
register(
    id="rcs/FR3SimplePickUpSim-v0",
    entry_point=FR3SimplePickUpSimEnvCreator(),
)
# TODO: gym.make("rcs/FR3SimEnv-v0") results in a pickling error:
# TypeError: cannot pickle 'rcs._core.sim.SimRobotConfig' object
# cf. https://pybind11.readthedocs.io/en/stable/advanced/classes.html#deepcopy-support
# register(
#    id="rcs/FR3SimEnv-v0",
#    entry_point=SimEnvCreator(),
# )
