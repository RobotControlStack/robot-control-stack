"""Robot control stack python bindings."""

import pathlib
import site

from gymnasium import register
from rcs import camera, control, envs, sim
from rcs._core import __version__, common, hw
from rcs.envs.creators import (
    FR3LabPickUpSimDigitHandEnvCreator,
    FR3SimplePickUpSimDigitHandEnvCreator,
    FR3SimplePickUpSimEnvCreator,
    RCSFR3DefaultEnvCreator,
    RCSFR3EnvCreator,
)

# available mujoco scenes
scenes = {
    path.stem: path / "scene.mjb" for path in (pathlib.Path(site.getsitepackages()[0]) / "rcs" / "scenes").glob("*")
}

# make submodules available
__all__ = ["__doc__", "__version__", "common", "hw", "sim", "camera", "scenes", "control", "envs"]

# register gymnasium environments
register(
    id="rcs/SimplePickUpSim-v0",
    entry_point=FR3SimplePickUpSimEnvCreator(),
)

register(
    id="rcs/SimplePickUpSimDigitHand-v0",
    entry_point=FR3SimplePickUpSimDigitHandEnvCreator(),
)
register(
    id="rcs/LabPickUpSimDigitHand-v0",
    entry_point=FR3LabPickUpSimDigitHandEnvCreator(),
)

register(
    id="rcs/FR3Env-v0",
    entry_point=RCSFR3EnvCreator(),
)

register(
    id="rcs/FR3Default-v0",
    entry_point=RCSFR3DefaultEnvCreator(),
)
