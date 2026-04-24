import copy
import logging
import threading
from dataclasses import dataclass, field
from time import sleep

import numpy as np
from rcs._core.common import Pose
from rcs.envs.base import ArmWithGripper, ControlMode, RelativeTo
from rcs.operator.interface import BaseOperator, BaseOperatorConfig, TeleopCommands
from rcs.sim.sim import Sim
from rcs.utils import SimpleFrameRate

try:
    from simpub.core.simpub_server import SimPublisher
    from simpub.parser.simdata import SimObject, SimScene
    from simpub.xr_device.meta_quest3 import MetaQuest3

    HAS_SIMPUB = True
except ImportError:
    HAS_SIMPUB = False

logger = logging.getLogger(__name__)

# download the iris apk from the following repo release: https://github.com/intuitive-robots/IRIS-Meta-Quest3
# in order to use usb connection install adb install adb
# sudo apt install android-tools-adb
# install it on your quest with
# adb install IRIS-Meta-Quest3.apk

if HAS_SIMPUB:

    class FakeSimPublisher(SimPublisher):
        def get_update(self):
            return {}

    class FakeSimScene(SimScene):
        def __init__(self):
            super().__init__()
            self.root = SimObject(name="root")


class QuestOperator(BaseOperator):

    control_mode = (ControlMode.CARTESIAN_TQuat, RelativeTo.CONFIGURED_ORIGIN)
    controller_names: list[str] = ["left", "right"]  # noqa: RUF012

    def __init__(self, config: "QuestConfig", sim: Sim | None = None):
        super().__init__(config, sim)
        if not HAS_SIMPUB:
            msg = "simpub is not installed. Please install it to use QuestOperator."
            raise ImportError(msg)

        self.config: QuestConfig

        self._resource_lock = threading.Lock()
        self._cmd_lock = threading.Lock()

        self._trg_btn = {"left": "index_trigger", "right": "index_trigger"}
        self._grp_btn = {"left": "hand_trigger", "right": "hand_trigger"}
        self._start_btn = "A"
        self._stop_btn = "B"
        self._unsuccessful_btn = "Y"

        self._prev_data = None
        self._exit_requested = False
        self._grp_pos = {key: 1.0 for key in self.controller_names}  # start with opened gripper
        self._last_controller_pose = {key: Pose() for key in self.controller_names}
        self._offset_pose = {key: Pose() for key in self.controller_names}

        self._commands = TeleopCommands()
        self._reset_origin_to_current()

        self._step_env = False
        self._set_frame = {key: Pose() for key in self.controller_names}
        # if self.config.simulation:
        #     self._publisher = MujocoPublisher(self.sim.model, self.sim.data, self.config.mq3_addr, visible_geoms_groups=list(range(1, 3)))
        if not self.config.simulation:
            self._publisher = FakeSimPublisher(FakeSimScene(), self.config.mq3_addr)
            # robot_cfg = default_sim_robot_cfg("fr3_empty_world")
            # sim_cfg = SimConfig()
            # sim_cfg.async_control = True
            # twin_env = SimMultiEnvCreator()(
            #     name2id=ROBOT2IP,
            #     robot_cfg=robot_cfg,
            #     control_mode=ControlMode.JOINTS,
            #     gripper_cfg=default_sim_gripper_cfg(),
            #     sim_cfg=sim_cfg,
            # )
            # sim = env_rel.unwrapped.envs[ROBOT2IP.keys().__iter__().__next__()].sim
            # sim.open_gui()
            # MujocoPublisher(sim.model, sim.data, MQ3_ADDR, visible_geoms_groups=list(range(1, 3)))
            # env_rel = DigitalTwin(env_rel, twin_env)
        self._reader = MetaQuest3("RCSNode")

    def _reset_origin_to_current(self, controller: str | None = None):
        with self._cmd_lock:
            if controller is None:
                self._commands.reset_origin_to_current = {key: True for key in self.controller_names}
            else:
                self._commands.reset_origin_to_current[controller] = True

    def _reset_state(self):
        with self._resource_lock:
            for controller in self.controller_names:
                self._offset_pose[controller] = Pose()
                self._last_controller_pose[controller] = Pose()
                self._grp_pos[controller] = 1

    def consume_commands(self) -> TeleopCommands:
        # must be threadsafe
        with self._cmd_lock:
            cmds = copy.copy(self._commands)
            self._commands = TeleopCommands()
            if self.config.switched_left_right:
                swapped_reset_origin_to_current = {}
                if "left" in cmds.reset_origin_to_current:
                    swapped_reset_origin_to_current["right"] = cmds.reset_origin_to_current["left"]
                if "right" in cmds.reset_origin_to_current:
                    swapped_reset_origin_to_current["left"] = cmds.reset_origin_to_current["right"]
                cmds.reset_origin_to_current = swapped_reset_origin_to_current
            return cmds

    def reset_operator_state(self):
        """Resets the hardware offsets when the environment resets."""
        self._reset_state()
        self._reset_origin_to_current()

    def consume_action(self) -> dict[str, ArmWithGripper]:
        transforms = {}
        with self._resource_lock:
            for controller in self.controller_names:
                transform = Pose(
                    translation=(
                        self._last_controller_pose[controller].translation()  # type: ignore
                        - self._offset_pose[controller].translation()
                    ),
                    quaternion=(
                        self._last_controller_pose[controller] * self._offset_pose[controller].inverse()
                    ).rotation_q(),
                )

                set_axes = Pose(quaternion=self._set_frame[controller].rotation_q())

                transform = set_axes.inverse() * transform * set_axes
                if not self.config.include_rotation:
                    transform = Pose(translation=transform.translation())  # identity rotation
                transforms[controller] = ArmWithGripper(
                    tquat=np.concatenate([transform.translation(), transform.rotation_q()]),
                    gripper=np.array([self._grp_pos[controller]]),
                )
        return (
            {"left": transforms["right"], "right": transforms["left"]}
            if self.config.switched_left_right
            else transforms
        )

    def close(self):
        self._reader.disconnect()
        # self._publisher.shutdown()
        self._exit_requested = True
        self.join()

    def run(self):
        rate_limiter = SimpleFrameRate(self.config.read_frequency, "teleop readout")
        warning_raised = False

        while not self._exit_requested:
            input_data = self._reader.get_controller_data()

            if input_data is None:
                if not warning_raised:
                    logger.warning("[Quest Reader] packets empty")
                    warning_raised = True
                sleep(0.5)
                continue

            if warning_raised:
                logger.warning("[Quest Reader] packets arriving again")
                warning_raised = False

            # === Update Semantic Commands ===
            with self._cmd_lock:
                if input_data[self._start_btn] and (self._prev_data is None or not self._prev_data[self._start_btn]):
                    self._commands.record = True

                if input_data[self._stop_btn] and (self._prev_data is None or not self._prev_data[self._stop_btn]):
                    self._commands.success = True

                if input_data[self._unsuccessful_btn] and (
                    self._prev_data is None or not self._prev_data[self._unsuccessful_btn]
                ):
                    self._commands.failure = True

            # === Update Poses & Grippers ===
            for controller in self.controller_names:
                last_controller_pose = Pose(
                    translation=np.array(input_data[controller]["pos"]),
                    quaternion=np.array(input_data[controller]["rot"]),
                )
                # if controller == "left":
                #     last_controller_pose = (
                #         Pose(translation=np.array([0, 0, 0]), rpy=RPY(roll=0, pitch=0, yaw=np.deg2rad(180)))  # type: ignore
                #         * last_controller_pose
                #     )

                if input_data[controller][self._trg_btn[controller]] and (
                    self._prev_data is None or not self._prev_data[controller][self._trg_btn[controller]]
                ):
                    # trigger just pressed (first data sample with button pressed)

                    with self._resource_lock:
                        self._offset_pose[controller] = last_controller_pose
                        self._last_controller_pose[controller] = last_controller_pose

                elif not input_data[controller][self._trg_btn[controller]] and (
                    self._prev_data is None or self._prev_data[controller][self._trg_btn[controller]]
                ):
                    with self._resource_lock:
                        self._last_controller_pose[controller] = Pose()
                        self._offset_pose[controller] = Pose()
                    self._reset_origin_to_current(controller)

                elif input_data[controller][self._trg_btn[controller]]:
                    # button is pressed
                    with self._resource_lock:
                        self._last_controller_pose[controller] = last_controller_pose

                if input_data[controller][self._grp_btn[controller]] and (
                    self._prev_data is None or not self._prev_data[controller][self._grp_btn[controller]]
                ):
                    # just pressed
                    self._grp_pos[controller] = 0
                if not input_data[controller][self._grp_btn[controller]] and (
                    self._prev_data is None or self._prev_data[controller][self._grp_btn[controller]]
                ):
                    # just released
                    self._grp_pos[controller] = 1

            self._prev_data = input_data
            rate_limiter()


@dataclass(kw_only=True)
class QuestConfig(BaseOperatorConfig):
    operator_class: type[BaseOperator] = field(default=QuestOperator)
    include_rotation: bool = True
    mq3_addr: str = "10.42.0.1"
    switched_left_right: bool = False
