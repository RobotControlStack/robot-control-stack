"""Robot control stack python bindings."""

import os
import site
from dataclasses import dataclass

from gymnasium import register
from rcs._core import __version__, common
from rcs.envs.creators import FR3SimplePickUpSimEnvCreator

from rcs import camera, envs, hand, sim


@dataclass(kw_only=True)
class Scene:
    """Scene configuration."""

    mjb: str
    """Path to the Mujoco binary scene file."""
    mjcf_scene: str
    """Path to the Mujoco scene XML file."""
    mjcf_robot: str
    """Path to the Mujoco robot XML file for IK."""
    urdf: str | None = None
    """Path to the URDF robot file for IK, if available."""
    robot_type: common.RobotType
    """Type of the robot in the scene."""


# available mujoco scenes
scenes: dict[str, Scene] = {
    "fr3_empty_world": Scene(
        mjb=os.path.join(site.getsitepackages()[0], "rcs", "scenes", "fr3_empty_world", "scene.mjb"),
        mjcf_scene=os.path.join(
            os.path.dirname(__file__), "..", "..", "assets", "scenes", "fr3_empty_world", "scene.xml"
        ),
        mjcf_robot=os.path.join(
            os.path.dirname(__file__), "..", "..", "assets", "scenes", "fr3_empty_world", "robot.xml"
        ),
        urdf=os.path.join(os.path.dirname(__file__), "..", "..", "assets", "scenes", "fr3_empty_world", "robot.urdf"),
        robot_type=common.RobotType.FR3,
    ),
    "fr3_simple_pick_up": Scene(
        mjb=os.path.join(site.getsitepackages()[0], "rcs", "scenes", "fr3_simple_pick_up", "scene.mjb"),
        mjcf_scene=os.path.join(
            os.path.dirname(__file__), "..", "..", "assets", "scenes", "fr3_simple_pick_up", "scene.xml"
        ),
        mjcf_robot=os.path.join(
            os.path.dirname(__file__), "..", "..", "assets", "scenes", "fr3_simple_pick_up", "robot.xml"
        ),
        urdf=os.path.join(
            os.path.dirname(__file__), "..", "..", "assets", "scenes", "fr3_simple_pick_up", "robot.urdf"
        ),
        robot_type=common.RobotType.FR3,
    ),
    "xarm7_empty_world": Scene(
        mjb=os.path.join(site.getsitepackages()[0], "rcs", "scenes", "xarm7_empty_world", "scene.mjb"),
        mjcf_scene=os.path.join(
            os.path.dirname(__file__), "..", "..", "assets", "scenes", "xarm7_empty_world", "scene.xml"
        ),
        mjcf_robot=os.path.join(
            os.path.dirname(__file__), "..", "..", "assets", "scenes", "xarm7_empty_world", "robot.xml"
        ),
        urdf=os.path.join(os.path.dirname(__file__), "..", "..", "assets", "scenes", "xarm7_empty_world", "robot.urdf"),
        robot_type=common.RobotType.FR3,
    ),
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
