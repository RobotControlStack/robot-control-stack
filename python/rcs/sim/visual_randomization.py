"""Visual randomization of composed MuJoCo scenes.

The randomizers operate on an ``mujoco.MjSpec`` before it is compiled. They only change the visual appearance of
the scene (background, materials, lights) and never add joints, so the simulator state layout stays compatible with
recorded episodes. This makes them usable both when creating an environment and when re-rendering recorded data
with the replayer.
"""

import os
from dataclasses import dataclass, field
from typing import Any

import mujoco
import numpy as np
from rcs._core.common import Pose

BACKGROUND = "background"
MATERIAL = "material"
LIGHTING = "lighting"
RANDOMIZER_NAMES = (BACKGROUND, MATERIAL, LIGHTING)


# --- background ---


@dataclass(kw_only=True)
class BackgroundScene:
    xml_path: str
    """MJCF of the scene the setup is placed in, e.g. a room."""
    scene_to_world: Pose = field(default_factory=Pose)
    """Pose of the background scene in the mujoco world frame, use it to place the setup on free floor."""
    name: str | None = None
    """Optional name used in the sampled parameters, defaults to the xml file name."""


@dataclass(kw_only=True)
class BackgroundRandomizerConfig:
    enabled: bool = False
    scenes: list[BackgroundScene] = field(default_factory=list)
    remove_base_geoms: list[str] = field(default_factory=lambda: ["floor"])
    """Geoms of the base scene that are removed when a background is added, e.g. the default floor plane."""
    remove_name_patterns: list[str] = field(default_factory=lambda: ["ceiling"])
    """Elements of the background scene whose name contains one of these (case-insensitive) substrings are removed.
    Ceilings would otherwise block overhead lights."""
    remove_lights: bool = True
    """Remove the lights of the background scene so that lighting stays controlled by the main scene."""
    prefix: str = "background_"


# --- material ---


@dataclass(kw_only=True)
class MaterialOption:
    texture: str | None = None
    """Path to a 2D texture image, None for an untextured material."""
    rgba: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)
    texrepeat: tuple[float, float] = (1.0, 1.0)
    texuniform: bool = False
    specular: float = 0.1
    shininess: float = 0.1
    reflectance: float = 0.0
    name: str | None = None
    """Optional name used in the sampled parameters, defaults to the texture file name."""


@dataclass(kw_only=True)
class MaterialTarget:
    """A group of materials and geoms that receive the same sampled material option."""

    options: list[MaterialOption]
    materials: list[str] = field(default_factory=list)
    """Names of existing materials whose properties are overwritten."""
    geoms: list[str] = field(default_factory=list)
    """Names of geoms that get a new material assigned."""
    mesh_files: dict[str, str] = field(default_factory=dict)
    """Mesh name to replacement file, e.g. a copy with texture coordinates of a mesh that has none."""
    name: str | None = None


@dataclass(kw_only=True)
class MaterialRandomizerConfig:
    enabled: bool = False
    targets: list[MaterialTarget] = field(default_factory=list)


# --- lighting ---


@dataclass(kw_only=True)
class LightSpec:
    type: str = "directional"
    """One of 'directional', 'spot' or 'point'."""
    pos: tuple[float, float, float] = (0.0, 0.0, 2.0)
    dir: tuple[float, float, float] = (0.0, 0.0, -1.0)
    target: tuple[float, float, float] | None = None
    """If given, the light points from pos to this point and dir is ignored."""
    diffuse: tuple[float, float, float] = (0.7, 0.7, 0.7)
    specular: tuple[float, float, float] = (0.3, 0.3, 0.3)
    ambient: tuple[float, float, float] = (0.0, 0.0, 0.0)
    cutoff: float = 45.0
    exponent: float = 10.0
    attenuation: tuple[float, float, float] = (1.0, 0.0, 0.0)
    castshadow: bool = True


@dataclass(kw_only=True)
class LightingPreset:
    lights: list[LightSpec] = field(default_factory=list)
    headlight_ambient: tuple[float, float, float] | None = None
    headlight_diffuse: tuple[float, float, float] | None = None
    replace_scene_lights: bool = True
    """Remove all lights of the scene before adding the preset lights."""
    name: str | None = None


@dataclass(kw_only=True)
class LightingRandomizerConfig:
    enabled: bool = False
    presets: list[LightingPreset] = field(default_factory=list)


@dataclass(kw_only=True)
class VisualRandomizerConfig:
    background: BackgroundRandomizerConfig = field(default_factory=BackgroundRandomizerConfig)
    material: MaterialRandomizerConfig = field(default_factory=MaterialRandomizerConfig)
    lighting: LightingRandomizerConfig = field(default_factory=LightingRandomizerConfig)
    shadowsize: int | None = None
    """Optional shadow map size, larger values reduce shadow artifacts of spot lights."""
    seed: int | None = None
    """Seed used when the randomization is applied at environment creation."""

    def enabled_randomizers(self) -> set[str]:
        return {name for name in RANDOMIZER_NAMES if getattr(self, name).enabled}

    def configured_randomizers(self) -> set[str]:
        """Randomizers that have options to sample from, independent of their enabled flag."""
        configured = set()
        if self.background.scenes:
            configured.add(BACKGROUND)
        if self.material.targets:
            configured.add(MATERIAL)
        if self.lighting.presets:
            configured.add(LIGHTING)
        return configured


def _basename(path: str | None) -> str | None:
    return None if path is None else os.path.splitext(os.path.basename(path))[0]


def _strip_elements(spec: mujoco.MjSpec, patterns: list[str]):
    lowered = [p.lower() for p in patterns]
    for collection in (spec.geoms, spec.sites, spec.cameras, spec.lights):
        for element in list(collection):
            if element.name and any(p in element.name.lower() for p in lowered):
                spec.delete(element)


def _make_static_visual(spec: mujoco.MjSpec):
    """Removes everything that would change the simulator state layout or interact with the setup."""
    for collection in (spec.actuators, spec.sensors, spec.tendons, spec.equalities, spec.keys, spec.excludes):
        for element in list(collection):
            spec.delete(element)
    for joint in list(spec.joints):
        spec.delete(joint)
    for geom in spec.geoms:
        geom.contype = 0
        geom.conaffinity = 0


def apply_background(spec: mujoco.MjSpec, cfg: BackgroundRandomizerConfig, rng: np.random.Generator) -> dict[str, Any]:
    if not cfg.scenes:
        msg = "Background randomization requires at least one background scene."
        raise ValueError(msg)
    scene = cfg.scenes[rng.integers(len(cfg.scenes))]
    if not os.path.exists(scene.xml_path):
        msg = f"Background scene not found: {scene.xml_path}"
        raise FileNotFoundError(msg)

    for geom in list(spec.geoms):
        if geom.name in cfg.remove_base_geoms:
            spec.delete(geom)

    background = mujoco.MjSpec.from_file(scene.xml_path)
    xml_dir = os.path.dirname(os.path.abspath(scene.xml_path))
    for mesh in background.meshes:
        if mesh.file and not os.path.isabs(mesh.file):
            mesh.file = os.path.abspath(os.path.join(xml_dir, background.meshdir or "", mesh.file))
    for tex in background.textures:
        if tex.file and not os.path.isabs(tex.file):
            tex.file = os.path.abspath(os.path.join(xml_dir, background.texturedir or "", tex.file))
    _strip_elements(background, cfg.remove_name_patterns)
    if cfg.remove_lights:
        for light in list(background.lights):
            background.delete(light)
    _make_static_visual(background)

    frame = spec.worldbody.add_frame()
    frame.pos = list(scene.scene_to_world.translation())
    frame.quat = list(scene.scene_to_world.rotation_q_wxyz())
    spec.attach(background, prefix=cfg.prefix, suffix="", frame=frame)
    return {"scene": scene.name or _basename(scene.xml_path)}


def _set_material(spec: mujoco.MjSpec, material: mujoco._specs.MjsMaterial, option: MaterialOption):
    textures = list(material.textures)
    rgb_role = int(mujoco.mjtTextureRole.mjTEXROLE_RGB)
    if option.texture is not None:
        texture = spec.add_texture()
        texture.name = f"{material.name}_randomized_texture"
        texture.type = mujoco.mjtTexture.mjTEXTURE_2D
        texture.file = os.path.abspath(option.texture)
        textures[rgb_role] = texture.name
    else:
        textures[rgb_role] = ""
    material.textures = textures
    material.rgba = list(option.rgba)
    material.texrepeat = list(option.texrepeat)
    material.texuniform = option.texuniform
    material.specular = option.specular
    material.shininess = option.shininess
    material.reflectance = option.reflectance


def apply_material(spec: mujoco.MjSpec, cfg: MaterialRandomizerConfig, rng: np.random.Generator) -> dict[str, Any]:
    if not cfg.targets:
        msg = "Material randomization requires at least one material target."
        raise ValueError(msg)
    sampled = {}
    geoms_by_name = {geom.name: geom for geom in spec.geoms if geom.name}
    for target_idx, target in enumerate(cfg.targets):
        target_name = target.name or f"target{target_idx}"
        option = target.options[rng.integers(len(target.options))]
        for mesh_name, mesh_file in target.mesh_files.items():
            mesh = spec.mesh(mesh_name)
            if mesh is None:
                msg = f"Mesh '{mesh_name}' of target '{target_name}' not found in scene."
                raise ValueError(msg)
            mesh.file = os.path.abspath(mesh_file)
        for material_name in target.materials:
            material = spec.material(material_name)
            if material is None:
                msg = f"Material '{material_name}' of target '{target_name}' not found in scene."
                raise ValueError(msg)
            _set_material(spec, material, option)
        if target.geoms:
            material = spec.add_material()
            material.name = f"{target_name}_randomized_material"
            _set_material(spec, material, option)
            for geom_name in target.geoms:
                if geom_name not in geoms_by_name:
                    msg = f"Geom '{geom_name}' of target '{target_name}' not found in scene."
                    raise ValueError(msg)
                geom = geoms_by_name[geom_name]
                geom.material = material.name
                # a non-default geom rgba would override the material color
                geom.rgba = [0.5, 0.5, 0.5, 1.0]
        sampled[target_name] = option.name or _basename(option.texture) or "untextured"
    return sampled


_LIGHT_TYPES = {
    "directional": mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
    "spot": mujoco.mjtLightType.mjLIGHT_SPOT,
    "point": mujoco.mjtLightType.mjLIGHT_POINT,
}


def apply_lighting(spec: mujoco.MjSpec, cfg: LightingRandomizerConfig, rng: np.random.Generator) -> dict[str, Any]:
    if not cfg.presets:
        msg = "Lighting randomization requires at least one lighting preset."
        raise ValueError(msg)
    preset_idx = int(rng.integers(len(cfg.presets)))
    preset = cfg.presets[preset_idx]
    if preset.replace_scene_lights:
        for light in list(spec.lights):
            spec.delete(light)
    for light_spec in preset.lights:
        light = spec.worldbody.add_light()
        light.type = _LIGHT_TYPES[light_spec.type]
        light.pos = list(light_spec.pos)
        direction = np.array(light_spec.dir, dtype=np.float64)
        if light_spec.target is not None:
            direction = np.subtract(light_spec.target, light_spec.pos, dtype=np.float64)
        light.dir = list(direction / np.linalg.norm(direction))
        light.diffuse = list(light_spec.diffuse)
        light.specular = list(light_spec.specular)
        light.ambient = list(light_spec.ambient)
        light.cutoff = light_spec.cutoff
        light.exponent = light_spec.exponent
        light.attenuation = list(light_spec.attenuation)
        light.castshadow = light_spec.castshadow
    if preset.headlight_ambient is not None:
        spec.visual.headlight.ambient = list(preset.headlight_ambient)
    if preset.headlight_diffuse is not None:
        spec.visual.headlight.diffuse = list(preset.headlight_diffuse)
    return {"preset": preset.name or f"preset{preset_idx}"}


class VisualRandomizer:
    """Applies the configured randomizers to an MjSpec.

    Example:
        randomizer = VisualRandomizer(cfg)
        params = randomizer.apply(composer.spec, np.random.default_rng(0))
    """

    def __init__(self, cfg: VisualRandomizerConfig):
        self.cfg = cfg

    def apply(
        self, spec: mujoco.MjSpec, rng: np.random.Generator, randomizers: set[str] | None = None
    ) -> dict[str, Any]:
        """Randomizes the spec in place.

        Args:
            spec: scene to randomize, must not be compiled into a running simulation yet.
            rng: random generator used for sampling.
            randomizers: names of the randomizers to apply, defaults to the enabled ones in the config.

        Returns:
            The sampled parameters per randomizer, e.g. {"background": {"scene": "FloorPlan1"}}.
        """
        if randomizers is None:
            randomizers = self.cfg.enabled_randomizers()
        unknown = set(randomizers) - set(RANDOMIZER_NAMES)
        if unknown:
            msg = f"Unknown randomizers {sorted(unknown)}, available: {list(RANDOMIZER_NAMES)}"
            raise ValueError(msg)
        if randomizers and self.cfg.shadowsize is not None:
            spec.visual.quality.shadowsize = self.cfg.shadowsize

        # fixed order so that the result only depends on the rng seed
        sampled: dict[str, Any] = {}
        if BACKGROUND in randomizers:
            sampled[BACKGROUND] = apply_background(spec, self.cfg.background, rng)
        if MATERIAL in randomizers:
            sampled[MATERIAL] = apply_material(spec, self.cfg.material, rng)
        if LIGHTING in randomizers:
            sampled[LIGHTING] = apply_lighting(spec, self.cfg.lighting, rng)
        return sampled


def write_box_projected_uv_obj(stl_path: str, obj_path: str, scale: float = 0.1) -> str:
    """Writes a copy of a binary STL mesh as OBJ with box-projected texture coordinates.

    MuJoCo can only apply 2D textures to meshes with texture coordinates. This helper makes STL meshes usable as
    material randomization targets, see ``MaterialTarget.mesh_files``.

    Args:
        stl_path: binary STL file.
        obj_path: output OBJ file.
        scale: texture size in mesh units, i.e. the texture repeats every ``scale`` units.
    """
    with open(stl_path, "rb") as f:
        data = f.read()
    n_tris = int(np.frombuffer(data, np.uint32, 1, 80)[0])
    dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
    tris: np.ndarray = np.frombuffer(data, dtype, n_tris, 84)["vertices"].astype(np.float64)
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    axes: list[int] = np.argmax(np.abs(normals), axis=1).tolist()

    lines = [f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in tris.reshape(-1, 3).tolist()]
    for tri, axis in zip(tris.tolist(), axes, strict=True):
        u, v = (i for i in range(3) if i != axis)
        lines.extend(f"vt {p[u] / scale:.5f} {p[v] / scale:.5f}" for p in tri)
    for i in range(n_tris):
        k = 3 * i + 1
        lines.append(f"f {k}/{k} {k + 1}/{k + 1} {k + 2}/{k + 2}")
    os.makedirs(os.path.dirname(os.path.abspath(obj_path)), exist_ok=True)
    with open(obj_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return obj_path
