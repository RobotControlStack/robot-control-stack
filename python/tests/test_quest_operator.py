import threading
from typing import Any, ClassVar, cast

import numpy as np
from rcs._core.common import Pose
from rcs.operator.interface import TeleopCommands
from rcs.operator.quest import QuestOperator


class _Config:
    switched_left_right = True
    include_rotation = True


def test_swap_controller_input_swaps_left_and_right_packets():
    operator = QuestOperator.__new__(QuestOperator)
    operator.config = cast(Any, _Config())

    input_data = {
        "left": {"hand_trigger": 0.1, "index_trigger": 0.2},
        "right": {"hand_trigger": 0.9, "index_trigger": 0.8},
        "A": True,
    }

    swapped = QuestOperator._swap_controller_input(operator, input_data)

    assert swapped["left"] == input_data["right"]
    assert swapped["right"] == input_data["left"]
    assert swapped["A"] is True


def test_consume_commands_swaps_both_reset_origin_keys():
    operator = QuestOperator.__new__(QuestOperator)
    operator.config = cast(Any, _Config())
    operator._cmd_lock = threading.Lock()
    operator._commands = TeleopCommands(reset_origin_to_current={"right": True, "left": False})

    cmds = QuestOperator.consume_commands(operator)

    assert cmds.reset_origin_to_current == {"right": True, "left": False}
    assert operator._commands.reset_origin_to_current == {}


def test_consume_action_swaps_gripper_with_controller():
    operator = QuestOperator.__new__(QuestOperator)
    operator.config = cast(Any, _Config())
    operator._resource_lock = threading.Lock()
    operator.controller_names = ["left", "right"]
    operator._last_controller_pose = {key: Pose() for key in operator.controller_names}
    operator._offset_pose = {key: Pose() for key in operator.controller_names}
    operator._set_frame = {key: Pose() for key in operator.controller_names}
    operator._grp_pos = {"left": 0.75, "right": 0.25}

    actions = QuestOperator.consume_action(operator)

    assert np.allclose(actions["left"]["gripper"], np.array([0.75]))
    assert np.allclose(actions["right"]["gripper"], np.array([0.25]))


def test_is_engaged_follows_index_trigger():
    operator = QuestOperator.__new__(QuestOperator)
    operator._engaged = {"left": False, "right": False}
    assert not QuestOperator.is_engaged(operator)
    operator._engaged["right"] = True
    assert QuestOperator.is_engaged(operator)


def test_stop_publisher_removes_its_scene_and_ignores_reconnects(monkeypatch):
    """A stopped publisher deletes its scene on the connected Quest apps (the next publisher only deletes a scene
    with its own name), stops streaming and does not send the scene to apps that connect later."""
    import asyncio
    import concurrent.futures

    import pyzlc
    from rcs.operator import quest

    calls: list[tuple[str, Any]] = []

    async def fake_call(service, request, **_kwargs):
        calls.append((service, request))

    def fake_submit(coro, *_args, **_kwargs):
        future: concurrent.futures.Future = concurrent.futures.Future()
        future.set_result(asyncio.run(coro))
        return future

    monkeypatch.setattr(pyzlc, "async_call", fake_call)
    monkeypatch.setattr(pyzlc, "submit_loop_task", fake_submit)
    monkeypatch.setattr(
        pyzlc,
        "get_nodes_info",
        lambda *_args, **_kwargs: [
            {"name": "IRIS/Device/Meta Quest 3/RCSNode", "nodeID": "quest"},
            {"name": "IRIS/Device/other", "nodeID": "not-this-publishers-device"},
        ],
    )

    class _Streamer:
        running = True

    class _WebServer:
        stopped = False

        def shutdown(self):
            self.stopped = True

    class _Scene:
        name = "blocksuite_sim2real_scene"

    class _Publisher:
        xr_device_set: ClassVar[set[str]] = {"quest"}
        sim_scene = _Scene()
        scene_update_streamer = _Streamer()
        web_server = _WebServer()

        async def on_new_device_found(self, xr_info):
            calls.append(("sent scene", xr_info))

    publisher = _Publisher()
    quest.stop_publisher(cast(Any, publisher))

    assert calls == [("IRIS/Device/Meta Quest 3/RCSNode/DeleteSimScene", "blocksuite_sim2real_scene")]
    assert publisher.scene_update_streamer.running is False
    assert publisher.web_server.stopped
    asyncio.run(publisher.on_new_device_found({"name": "IRIS/Device/Meta Quest 3/RCSNode"}))
    assert len(calls) == 1  # a reconnecting Quest app does not get the old scene again
