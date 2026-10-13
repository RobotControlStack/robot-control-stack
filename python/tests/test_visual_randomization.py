import struct
from pathlib import Path

import mujoco
import numpy as np
import pytest
from PIL import Image
from rcs.sim.visual_randomization import (
    BACKGROUND,
    LIGHTING,
    MATERIAL,
    BackgroundRandomizerConfig,
    BackgroundScene,
    LightingPreset,
    LightingRandomizerConfig,
    LightSpec,
    MaterialOption,
    MaterialRandomizerConfig,
    MaterialTarget,
    VisualRandomizer,
    VisualRandomizerConfig,
    write_box_projected_uv_obj,
)

import rcs

BASE_XML = """
<mujoco>
  <asset>
    <material name="table_mat" rgba="0.6 0.4 0.2 1"/>
  </asset>
  <worldbody>
    <light name="main_light" pos="0 0 2" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="0 0 0.05"/>
    <geom name="table" type="box" size="0.5 0.5 0.02" pos="0 0 0.8" material="table_mat"/>
    <body name="box" pos="0 0 1">
      <freejoint name="box_joint"/>
      <geom name="box_geom" type="box" size="0.02 0.02 0.02" rgba="0 0 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""

BACKGROUND_XML = """
<mujoco>
  <worldbody>
    <light name="room_light" pos="0 0 2.5" dir="0 0 -1"/>
    <geom name="room_floor" type="plane" size="3 3 0.05"/>
    <geom name="ceiling_panel" type="box" size="3 3 0.01" pos="0 0 2.6"/>
    <body name="apple" pos="1 1 0.1">
      <freejoint name="apple_joint"/>
      <geom name="apple_geom" type="sphere" size="0.05"/>
    </body>
  </worldbody>
</mujoco>
"""


def _write_png(path: Path, color: tuple[int, int, int]) -> str:
    Image.new("RGB", (8, 8), color).save(path)
    return str(path)


def _write_binary_stl(path: Path) -> str:
    tris = np.array(
        [
            [[0, 0, 0], [1, 0, 0], [0, 1, 0]],
            [[0, 0, 0], [0, 1, 0], [0, 0, 1]],
            [[0, 0, 0], [0, 0, 1], [1, 0, 0]],
            [[1, 0, 0], [0, 0, 1], [0, 1, 0]],
        ],
        dtype=np.float32,
    )
    with open(path, "wb") as f:
        f.write(b"\0" * 80 + struct.pack("<I", len(tris)))
        for i in range(len(tris)):
            f.write(np.zeros(3, np.float32).tobytes() + tris[i].tobytes() + b"\0\0")
    return str(path)


@pytest.fixture()
def config(tmp_path: Path) -> VisualRandomizerConfig:
    background_xml = tmp_path / "room.xml"
    background_xml.write_text(BACKGROUND_XML)
    return VisualRandomizerConfig(
        background=BackgroundRandomizerConfig(
            scenes=[
                BackgroundScene(
                    xml_path=str(background_xml),
                    scene_to_world=rcs.common.Pose(translation=np.array([1.0, 0.0, 0.0])),
                    name="room",
                )
            ]
        ),
        material=MaterialRandomizerConfig(
            targets=[
                MaterialTarget(
                    name="table",
                    materials=["table_mat"],
                    options=[MaterialOption(texture=_write_png(tmp_path / "marble.png", (230, 230, 230)))],
                ),
                MaterialTarget(
                    name="box",
                    geoms=["box_geom"],
                    options=[MaterialOption(rgba=(0.1, 0.2, 0.9, 1.0), name="blue")],
                ),
            ]
        ),
        lighting=LightingRandomizerConfig(
            presets=[
                LightingPreset(
                    name="warm_spot",
                    lights=[LightSpec(type="spot", pos=(1, 0, 2), target=(0, 0, 0.8), diffuse=(1.0, 0.7, 0.4))],
                    headlight_ambient=(0.1, 0.1, 0.1),
                )
            ]
        ),
    )


def _base_spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_string(BASE_XML)


def test_defaults_are_disabled():
    cfg = VisualRandomizerConfig()
    assert cfg.enabled_randomizers() == set()
    assert cfg.configured_randomizers() == set()
    spec = _base_spec()
    assert VisualRandomizer(cfg).apply(spec, np.random.default_rng(0)) == {}
    model = spec.compile()
    assert model.ngeom == _base_spec().compile().ngeom


def test_enabled_flags_select_randomizers(config: VisualRandomizerConfig):
    config.lighting.enabled = True
    sampled = VisualRandomizer(config).apply(_base_spec(), np.random.default_rng(0))
    assert set(sampled) == {LIGHTING}
    assert config.configured_randomizers() == {BACKGROUND, MATERIAL, LIGHTING}


def test_background_is_static_and_keeps_state_layout(config: VisualRandomizerConfig):
    reference = _base_spec().compile()
    spec = _base_spec()
    sampled = VisualRandomizer(config).apply(spec, np.random.default_rng(0), {BACKGROUND})
    model = spec.compile()

    assert sampled == {BACKGROUND: {"scene": "room"}}
    assert (model.nq, model.nv, model.njnt) == (reference.nq, reference.nv, reference.njnt)
    geom_names = {model.geom(i).name for i in range(model.ngeom)}
    assert "floor" not in geom_names
    assert "background_room_floor" in geom_names
    assert "background_ceiling_panel" not in geom_names
    assert {model.light(i).name for i in range(model.nlight)} == {"main_light"}
    background_geoms = [i for i in range(model.ngeom) if model.geom(i).name.startswith("background_")]
    assert all(model.geom_contype[i] == 0 and model.geom_conaffinity[i] == 0 for i in background_geoms)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert np.allclose(data.geom("background_room_floor").xpos, [1.0, 0.0, 0.0])


def test_material_randomization(config: VisualRandomizerConfig):
    spec = _base_spec()
    sampled = VisualRandomizer(config).apply(spec, np.random.default_rng(0), {MATERIAL})
    model = spec.compile()

    assert sampled == {MATERIAL: {"table": "marble", "box": "blue"}}
    table_mat = model.material("table_mat")
    assert model.mat_texid[table_mat.id, int(mujoco.mjtTextureRole.mjTEXROLE_RGB)] >= 0
    box_geom = model.geom("box_geom")
    assert np.allclose(model.mat_rgba[box_geom.matid[0]], [0.1, 0.2, 0.9, 1.0])


def test_lighting_randomization(config: VisualRandomizerConfig):
    spec = _base_spec()
    sampled = VisualRandomizer(config).apply(spec, np.random.default_rng(0), {LIGHTING})
    model = spec.compile()

    assert sampled == {LIGHTING: {"preset": "warm_spot"}}
    assert model.nlight == 1
    assert model.light_type[0] == mujoco.mjtLightType.mjLIGHT_SPOT
    expected_dir = np.array([-1.0, 0.0, -1.2]) / np.linalg.norm([-1.0, 0.0, -1.2])
    assert np.allclose(model.light_dir[0], expected_dir)
    assert np.allclose(model.vis.headlight.ambient, [0.1, 0.1, 0.1])


def test_sampling_is_deterministic(config: VisualRandomizerConfig):
    config.lighting.presets = [LightingPreset(name=f"p{i}") for i in range(10)]
    config.material.targets[1].options = [MaterialOption(name=f"m{i}") for i in range(10)]
    randomizer = VisualRandomizer(config)
    names = {BACKGROUND, MATERIAL, LIGHTING}
    first = randomizer.apply(_base_spec(), np.random.default_rng(42), names)
    second = randomizer.apply(_base_spec(), np.random.default_rng(42), names)
    assert first == second


def test_unknown_and_unconfigured_randomizers_raise():
    randomizer = VisualRandomizer(VisualRandomizerConfig())
    with pytest.raises(ValueError, match="Unknown randomizers"):
        randomizer.apply(_base_spec(), np.random.default_rng(0), {"camera"})
    with pytest.raises(ValueError, match="at least one"):
        randomizer.apply(_base_spec(), np.random.default_rng(0), {LIGHTING})


def test_box_projected_uv_obj_is_textured(tmp_path: Path):
    obj = write_box_projected_uv_obj(_write_binary_stl(tmp_path / "part.stl"), str(tmp_path / "part_uv.obj"))
    spec = mujoco.MjSpec.from_string(
        f"""
        <mujoco>
          <asset><mesh name="part" file="{obj}"/></asset>
          <worldbody><geom name="part_geom" type="mesh" mesh="part"/></worldbody>
        </mujoco>
        """
    )
    model = spec.compile()
    assert model.mesh_texcoordnum[0] == 12


def _empty_world_cfg(visual_randomization: VisualRandomizerConfig | None):
    from rcs.envs.configs import EmptyWorldFR3Duo

    scene = EmptyWorldFR3Duo()
    cfg = scene.config()
    cfg.headless = True
    cfg.camera_cfgs = {}
    cfg.visual_randomization = visual_randomization
    return scene, cfg


def test_env_creation_applies_enabled_randomizers(config: VisualRandomizerConfig):
    config.lighting.enabled = True
    scene, cfg = _empty_world_cfg(config)
    composer = scene.create_model(cfg)
    assert composer.visual_randomization_params == {LIGHTING: {"preset": "warm_spot"}}
    env = scene.create_env_from_model(cfg, composer)
    try:
        model = env.get_wrapper_attr("sim").model
        assert model.nlight == 1
        assert model.light_type[0] == mujoco.mjtLightType.mjLIGHT_SPOT
    finally:
        env.close()


def test_env_creation_without_enabled_randomizers_is_unchanged(config: VisualRandomizerConfig):
    scene, cfg = _empty_world_cfg(None)
    reference = scene.create_model(cfg).spec.compile()
    scene, cfg = _empty_world_cfg(config)
    composer = scene.create_model(cfg)
    model = composer.spec.compile()
    assert composer.visual_randomization_params == {}
    assert (model.ngeom, model.nlight, model.nmat) == (reference.ngeom, reference.nlight, reference.nmat)
