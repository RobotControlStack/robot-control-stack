import copy
import logging
import threading
from dataclasses import dataclass, field
from time import sleep
from typing import Any

import numpy as np
from rcs._core.common import Pose
from rcs.envs.base import ArmWithGripper, ControlMode, RelativeTo
from rcs.operator.interface import BaseOperator, BaseOperatorConfig, TeleopCommands
from rcs.sim.sim import Sim
from rcs.utils import SimpleFrameRate

try:
    from simpub.core.simpub_server import RigidObjectUpdateData, SimPublisher
    from simpub.core.video_streamer import VideoStreamerManager
    from simpub.parser.simdata import SimObject, SimScene, SimSceneConfig
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
            return RigidObjectUpdateData(data={})

    class FakeSimScene(SimScene):
        def __init__(self, name: str = "RCS"):
            super().__init__(
                SimSceneConfig(
                    name=name,
                    pos=[0.0, 0.0, 0.0],
                    rot=[0.0, 0.0, 0.0, 1.0],
                    scale=[1.0, 1.0, 1.0],
                )
            )
            self.root.add_data(
                SimObject(
                    name="root",
                    parent="root",
                    trans={
                        "pos": [0.0, 0.0, 0.0],
                        "rot": [0.0, 0.0, 0.0, 1.0],
                        "scale": [1.0, 1.0, 1.0],
                    },
                    visuals=[],
                )
            )


class QuestOperator(BaseOperator):

    control_mode = (ControlMode.CARTESIAN_TQuat, RelativeTo.CONFIGURED_ORIGIN)
    controller_names: list[str] = ["right"]  # noqa: RUF012

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
        self._controller_pose_valid = dict.fromkeys(self.controller_names, False)
        self._offset_pose = {key: Pose() for key in self.controller_names}

        self._commands = TeleopCommands()
        if not self.config.absolute_tracking:
            self._reset_origin_to_current()

        self._step_env = False
        self._set_frame = {key: Pose() for key in self.controller_names}
        self._video_stream_manager = None
        self._video_streamers: dict[str, Any] = {}
        if not self.config.simulation:
            self._publisher = FakeSimPublisher(FakeSimScene(), self.config.mq3_addr)
            # Not working code for digital twin:
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
                self._controller_pose_valid[controller] = False
                self._grp_pos[controller] = 1

    def _swap_controller_input(self, input_data: dict[str, Any]) -> dict[str, Any]:
        if not self.config.switched_left_right:
            return input_data
        swapped = copy.copy(input_data)
        swapped["left"] = input_data["right"]
        swapped["right"] = input_data["left"]
        return swapped

    @staticmethod
    def _normalize_axis(value: bool | float | int) -> float:
        if isinstance(value, bool):
            return float(value)
        return float(np.clip(value, 0.0, 1.0))

    def consume_commands(self) -> TeleopCommands:
        # must be threadsafe
        with self._cmd_lock:
            cmds = copy.copy(self._commands)
            self._commands = TeleopCommands()
            return cmds

    def reset_operator_state(self):
        """Resets the hardware offsets when the environment resets."""
        self._reset_state()
        if not self.config.absolute_tracking:
            self._reset_origin_to_current()

    def consume_action(self) -> dict[str, ArmWithGripper]:
        transforms = {}
        with self._resource_lock:
            for controller in self.controller_names:
                if self.config.absolute_tracking:
                    # Do not command the identity pose while waiting for the first
                    # Quest packet after startup or an environment reset.
                    if not self._controller_pose_valid[controller]:
                        continue
                    transform = self._last_controller_pose[controller]
                else:
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
        return transforms

    def set_camera(self, observation: dict) -> None:
        if not self.config.display_cameras:
            return
        frames = observation.get("frames")
        if not isinstance(frames, dict):
            return

        if self._video_stream_manager is None:
            self._video_stream_manager = VideoStreamerManager(self.config.mq3_addr)

        for camera_name, camera_data in frames.items():
            if not isinstance(camera_data, dict) or "rgb" not in camera_data:
                continue
            rgb = camera_data["rgb"]
            if not isinstance(rgb, dict) or "data" not in rgb:
                continue
            frame = rgb["data"]
            if camera_name not in self._video_streamers:
                height, width = frame.shape[:2]
                stream_topic = self._get_stream_topic_name(camera_name)
                assert self._video_stream_manager is not None
                self._video_streamers[camera_name] = self._video_stream_manager.create_streamer(
                    stream_topic, width, height
                )
            assert self._video_streamers[camera_name] is not None
            self._video_streamers[camera_name].update_cv_image(frame[:, :, ::-1])

    @staticmethod
    def _get_stream_topic_name(camera_name: str) -> str:
        if camera_name.endswith("_left"):
            return f"{camera_name[:-len('_left')]}_camera_left"
        if camera_name.endswith("_right"):
            return f"{camera_name[:-len('_right')]}_camera_right"
        return f"{camera_name}_camera"

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

            input_data = self._swap_controller_input(input_data)

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
                prev_data = self._prev_data
                last_controller_pose = Pose(
                    translation=np.array(input_data[controller]["pos"]),
                    quaternion=np.array(input_data[controller]["rot"]),
                )
                if self.config.umi_mode and controller in self.config.umi_mode_tool_offset:
                    last_controller_pose = last_controller_pose * self.config.umi_mode_tool_offset[controller]
                # if controller == "left":
                #     last_controller_pose = (
                #         Pose(translation=np.array([0, 0, 0]), rpy=RPY(roll=0, pitch=0, yaw=np.deg2rad(180)))  # type: ignore
                #         * last_controller_pose
                #     )

                trigger_pressed = self._normalize_axis(input_data[controller][self._trg_btn[controller]]) > 0.5
                if prev_data is None:
                    prev_trigger_pressed = False
                else:
                    prev_trigger_pressed = self._normalize_axis(prev_data[controller][self._trg_btn[controller]]) > 0.5

                if self.config.absolute_tracking:
                    # UMI-style tracking is unclutched: every controller packet is
                    # an absolute Cartesian target for the robot end effector.
                    with self._resource_lock:
                        self._last_controller_pose[controller] = last_controller_pose
                        self._controller_pose_valid[controller] = True

                elif trigger_pressed and not prev_trigger_pressed:
                    # trigger just pressed (first data sample with button pressed)

                    with self._resource_lock:
                        self._offset_pose[controller] = last_controller_pose
                        self._last_controller_pose[controller] = last_controller_pose

                elif not trigger_pressed and prev_trigger_pressed:
                    with self._resource_lock:
                        self._last_controller_pose[controller] = Pose()
                        self._offset_pose[controller] = Pose()
                    self._reset_origin_to_current(controller)

                elif trigger_pressed:
                    # button is pressed
                    with self._resource_lock:
                        self._last_controller_pose[controller] = last_controller_pose

                gripper_axis = self._normalize_axis(input_data[controller][self._grp_btn[controller]])
                # convert from IRIS to RCS gripper logic
                with self._resource_lock:
                    self._grp_pos[controller] = 1.0 - gripper_axis

            self._prev_data = input_data
            rate_limiter()


@dataclass(kw_only=True)
class QuestConfig(BaseOperatorConfig):
    operator_class: type[BaseOperator] = field(default=QuestOperator)
    include_rotation: bool = True
    mq3_addr: str = "10.42.0.1"
    switched_left_right: bool = False
    display_cameras: bool = True
    umi_mode: bool = False
    umi_mode_tool_offset: dict[str, Pose] = field(default_factory=dict)
    # Emit transformed controller poses directly and update them without a clutch.
    absolute_tracking: bool = False
