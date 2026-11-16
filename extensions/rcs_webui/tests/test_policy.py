import time

import numpy as np
import pytest
from rcs_webui.policy import (
    ChunkExecutor,
    MockPolicyClient,
    VLAgentsPolicyClient,
    load_policy_client_class,
)

OBS = {"right": {"joints": np.zeros(7), "gripper": [1.0]}}


def run(executor: ChunkExecutor, steps: int, step_time: float = 0.0):
    sources = []
    for step in range(steps):
        _, source, _ = executor.act(OBS, step)
        sources.append(source)
        time.sleep(step_time)
    return sources


def test_load_clients():
    assert load_policy_client_class("mock") is MockPolicyClient
    assert load_policy_client_class("vlagents") is VLAgentsPolicyClient
    assert load_policy_client_class("rcs_webui.policy:MockPolicyClient") is MockPolicyClient
    with pytest.raises(ValueError, match="unknown policy client"):
        load_policy_client_class("does_not_exist")
    with pytest.raises(TypeError):
        load_policy_client_class("rcs_webui.policy:ChunkExecutor")


def test_vlagents_has_no_rtc():
    with pytest.raises(ValueError, match="does not support real-time chunking"):
        ChunkExecutor(VLAgentsPolicyClient("localhost", 1), rtc_delay=2)


def test_synchronous_chunks():
    client = MockPolicyClient(horizon=8)
    executor = ChunkExecutor(client, execute_steps=4)
    executor.reset("pick")
    sources = run(executor, 10)
    assert [s.chunk_id for s in sources] == [0] * 4 + [1] * 4 + [2] * 2
    assert [s.chunk_index for s in sources] == [0, 1, 2, 3] * 2 + [0, 1]
    assert [s.switch_step for s in sources] == [0] * 4 + [4] * 4 + [8] * 2
    assert all(r.prefix is None for r in client.requests)
    assert client.requests[1].episode_step == 4
    assert client.requests[0].instruction == "pick"


def test_rtc_switches_at_switch_over_step_with_prefix():
    client = MockPolicyClient(horizon=8, delay_s=0.0)
    executor = ChunkExecutor(client, rtc_delay=2)
    executor.reset("pick")
    sources = run(executor, 14, step_time=0.01)
    # first chunk synchronous, query for the next one 2 steps before its end (step 6), switch at step 8
    assert [s.chunk_id for s in sources[:8]] == [0] * 8
    assert sources[8].chunk_id == 1
    assert sources[8].switch_step == 8
    # the first actions of the new chunk are the committed prefix that was executed meanwhile
    assert sources[8].chunk_index == 2
    request = client.requests[1]
    assert request.prefix is not None
    assert request.guidance is not None
    assert request.episode_step == 6
    assert request.delay == 2
    assert np.array_equal(request.prefix, [[0, 6], [0, 7]])
    assert request.prefix_chunk_id == 0
    assert request.prefix_start == 6
    assert request.guidance.shape == (0, 2)
    # next query is 2 steps before the end of chunk 1 (index 6 of it = step 12)
    assert client.requests[2].episode_step == 12
    assert np.array_equal(np.asarray(client.requests[2].prefix), [[1, 6], [1, 7]])
    assert all(s.waited == 0.0 for s in sources[1:])


def test_rtc_guidance_with_partial_execution():
    client = MockPolicyClient(horizon=10)
    executor = ChunkExecutor(client, execute_steps=6, rtc_delay=2)
    executor.reset("pick")
    run(executor, 8, step_time=0.01)
    request = client.requests[1]
    assert request.prefix is not None
    assert request.guidance is not None
    assert request.episode_step == 4
    assert np.array_equal(request.prefix, [[0, 4], [0, 5]])
    # the rest of the current chunk after the switch-over step is sent for inpainting
    assert np.array_equal(request.guidance[:, 1], [6, 7, 8, 9])


def test_rtc_late_answer_keeps_executing_and_switches_when_it_arrives():
    client = MockPolicyClient(horizon=12, delay_s=0.1)
    executor = ChunkExecutor(client, execute_steps=6, rtc_delay=2)
    executor.reset("pick")
    sources = run(executor, 12, step_time=0.02)
    # query at step 4, switch-over step 6, but the answer needs ~5 steps: the old chunk continues
    first_new = next(i for i, s in enumerate(sources) if s.chunk_id == 1)
    assert first_new > 6
    # it is used at the index aligned with the step the query was sent
    assert sources[first_new].chunk_index == first_new - 4
    assert sources[first_new].switch_step == first_new
    assert sources[first_new].response_received - sources[first_new].request_sent >= 0.1


def test_rtc_starved_waits():
    client = MockPolicyClient(horizon=4, delay_s=0.1)
    executor = ChunkExecutor(client, rtc_delay=1)
    executor.reset("pick")
    sources = run(executor, 6)
    # without step time the old chunk runs out before the answer: the loop waits for it
    assert sources[4].chunk_id == 1
    assert sources[4].waited > 0.05


def test_rtc_receding_horizon_matches_replan_steps():
    """execute_steps = r actions per chunk counted from the switch-over index (the SURF replan_steps)."""
    client = MockPolicyClient(horizon=16)
    executor = ChunkExecutor(client, execute_steps=8, rtc_delay=3)
    executor.reset("pick")
    sources = run(executor, 30, step_time=0.005)
    # boot chunk: rows 0..7, query at step 5 with prefix rows 5..7
    assert client.requests[1].episode_step == 5
    assert client.requests[1].prefix_start == 5
    # chunk 1 starts at index 3 (step 8) and executes 8 actions (indices 3..10, steps 8..15)
    chunk1 = [s for s in sources if s.chunk_id == 1]
    assert [s.chunk_index for s in chunk1] == list(range(3, 11))
    # next query 3 steps before its end: index 8 of chunk 1 = step 13, prefix rows 8..10
    assert client.requests[2].episode_step == 13
    assert client.requests[2].prefix_start == 8
    assert np.array_equal(np.asarray(client.requests[2].prefix)[:, 1], [8, 9, 10])


def test_action_at_converts_at_execution_time():
    class RawOnly(MockPolicyClient):
        def infer(self, request):
            chunk = super().infer(request)
            chunk.actions = []
            return chunk

        def action_at(self, chunk, index, observation):
            return {"row": chunk.raw[index].tolist(), "obs": observation["right"]["joints"][0]}

    executor = ChunkExecutor(RawOnly(horizon=4), execute_steps=4)
    executor.reset("pick")
    for step in range(6):
        obs = {"right": {"joints": np.full(7, float(step)), "gripper": [1.0]}}
        action, source, _ = executor.act(obs, step)
        assert action == {"row": [source.request_id, source.chunk_index], "obs": float(step)}


def test_controller_kinds():
    from rcs_webui.policy import Controller, MockPolicyClient, VLAgentsPolicyClient

    assert {c.value for c in Controller} >= {"human", "learned_policy", "scripted_policy", "agent", "none"}
    assert VLAgentsPolicyClient.controller is Controller.LEARNED_POLICY
    assert MockPolicyClient.controller is Controller.SCRIPTED_POLICY
    assert Controller("agent") == "agent"  # str enum: compares and serializes as its value
