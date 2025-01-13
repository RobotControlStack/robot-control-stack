"""Robot control stack python bindings."""

import pathlib
import site

from gymnasium import register
from rcsss import camera, control, envs, sim
from rcsss._core import __version__, common, hw
from rcsss.envs.factories import FR3Real, FR3SimplePickUpSim, FR3SimplePickUpSimDigitHand, FR3LabPickUpSimDigitHand

# available mujoco scenes
scenes = {
    path.stem: path / "scene.mjb" for path in (pathlib.Path(site.getsitepackages()[0]) / "rcsss" / "scenes").glob("*")
}

# make submodules available
__all__ = ["__doc__", "__version__", "common", "hw", "sim", "camera", "scenes", "control", "envs"]

# register gymnasium environments
register(
    id="rcs/SimplePickUpSim-v0",
    entry_point=FR3SimplePickUpSim(),
)

register(
    id="rcs/SimplePickUpSimDigitHand-v0",
    entry_point=FR3SimplePickUpSimDigitHand(),
)
register(
    id="rcs/FR3LabPickUpSimDigitHand-v0",
    entry_point=FR3LabPickUpSimDigitHand(),
)

register(
    id="rcs/FR3Real-v0",
    entry_point=FR3Real(),
)
