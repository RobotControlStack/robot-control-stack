#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["mujoco>=3.3", "xacro", "trimesh", "networkx", "scipy"]
# ///
"""Generate the Flexiv Grav (GN-01) MJCF from Flexiv's official URDF description.

Pipeline (every step is deterministic, no hand edits of the output):

1. Fetch ``grav_macro.xacro``, the license and the Grav meshes from
   https://github.com/flexivrobotics/flexiv_description at a pinned commit
   (or take them from a local checkout passed via ``--src``).
2. Copy the meshes into ``<out>/assets``. Collision meshes get a ``_collision``
   suffix because MuJoCo names meshes by file stem and would otherwise merge the
   visual and collision ``base.stl`` into one mesh. ``finger_tip.obj`` holds the
   rubber pad and the aluminum holder as separate OBJ groups, of which MuJoCo only
   loads the first, so it is split into one single-group file per part.
3. Expand the xacro macro into a URDF with ``xacro`` and rewrite it: point the
   mesh paths at ``<out>/assets``, drop the flange parent link so the gripper
   base becomes the root, drop the virtual ``finger_width_joint`` (MuJoCo has no
   mimic joints) and turn the massless TCP links into recorded frames.
4. Import the URDF with ``mujoco.MjSpec`` and post-process it: default classes,
   link masses and inertias derived from the mesh volumes at the density of
   their material (the URDF only specifies the total mass), pad boxes on the
   finger tips, TCP sites, contact excludes, joint equality constraints
   replacing the URDF mimic joints, a tendon splitting the actuation between
   both knuckles and a position actuator whose control input is the finger
   opening width in meters.
5. Write ``<out>/flexiv_grav.xml`` plus ``scene.xml`` and ``LICENSE``.

Usage:
    uv run assets/grippers/flexiv_grav/convert_from_urdf.py --out mujoco_menagerie/flexiv_grav
    uv run mujoco_menagerie/format_xml.py --write mujoco_menagerie/flexiv_grav/*.xml
"""

from __future__ import annotations

import argparse
import pathlib
import shutil
import tempfile
import urllib.request
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh
import xacro

UPSTREAM_REPO = "flexivrobotics/flexiv_description"
UPSTREAM_COMMIT = "8b8452105c5faf088f456db18b0e2581b16fc630"  # branch humble, 2026-09-03
UPSTREAM_RAW = f"https://raw.githubusercontent.com/{UPSTREAM_REPO}/{UPSTREAM_COMMIT}/"
UPSTREAM_FILES = [
    "LICENSE",
    "urdf/common/grav_macro.xacro",
    "meshes/Grav/visual/base.stl",
    "meshes/Grav/visual/outer_bar.stl",
    "meshes/Grav/visual/inner_bar.stl",
    "meshes/Grav/visual/finger_mount.stl",
    "meshes/Grav/visual/finger_tip.obj",
    "meshes/Grav/collision/base.stl",
    "meshes/Grav/collision/outer_bar.stl",
    "meshes/Grav/collision/inner_bar.stl",
    "meshes/Grav/collision/finger_tip.stl",
]

MODEL_NAME = "flexiv_grav"
ROOT_CLASS = "grav"

# The URDF drives the linkage through a virtual prismatic joint (finger width in meters) with
# knuckle_angle = WIDTH_GAIN * width + WIDTH_OFFSET. We keep this mapping for the actuator.
WIDTH_GAIN = 9.404
WIDTH_OFFSET = -0.155
MAX_WIDTH = 0.1
# Position controller gains on the tendon length (= knuckle angle) and the force saturation.
KP = 20.0
KV = 1.0
FORCERANGE = 5.0

# The URDF only specifies the total mass (1.186 kg on the base link, 1e-4 kg on the moving links, which
# is too light for a stable simulation). Flexiv's datasheet lists 0.9 kg for the Grav; the URDF value is
# kept since Flexiv's own software uses it as tool mass. The moving links get the mass of their visual
# meshes at the density of their material, the base the remainder.
TOTAL_MASS = 1.186
ALUMINUM = 2700.0  # kg/m^3
RUBBER = 1100.0  # kg/m^3
# Visual meshes per URDF link (left side, the right side is identical) with their material density.
LINK_MESHES = {
    "outer_bar": [("outer_bar.stl", ALUMINUM)],
    "inner_bar": [("inner_bar.stl", ALUMINUM)],
    "finger_mount": [("finger_mount.stl", ALUMINUM)],
    "finger_tip": [("finger_tip_holder.obj", ALUMINUM), ("finger_tip_pad.obj", RUBBER)],
}
# The base visual mesh is not watertight, its inertia is taken from the convex collision hull instead.
BASE_HULL = "base_collision.stl"
# The URDF rotates every mesh by 90 degrees about z into the link frame (rpy="0 0 pi/2").
MESH_TO_LINK = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])

# ``finger_tip.obj`` groups: mesh1 and mesh2 are the rubber pad and its end cap, mesh3 the aluminum
# holder. MuJoCo only loads the first group of an OBJ file, so each part becomes its own file.
FINGER_TIP_OBJ = "finger_tip.obj"
FINGER_TIP_PARTS = {"finger_tip_pad": ["mesh1", "mesh2"], "finger_tip_holder": ["mesh3"]}
FINGER_TIP_MATERIALS = {"finger_tip_pad": "black", "finger_tip_holder": "aluminum"}

# The upstream ``finger_mount.stl`` is exported rotated by 1.858 degrees about the outer bar pivot
# (the x axis of the finger mount link) relative to the URDF joint frames: its top face is slanted,
# the inner bar pin misses the pivot hole and the finger tip holder cuts into its outer face. The
# URDF kinematics are consistent (the pads meet when closed), so only the visual geom is rotated
# back; the mount has no collision geom and the finger tip frame stays as in the URDF.
FINGER_MOUNT_VISUAL_TILT = np.deg2rad(-1.858)

# Massless URDF links that only define frames. They become sites on their parent body.
TCP_LINKS = ["grav_tcp", "closed_fingers_tcp", "left_finger_tcp", "right_finger_tcp"]

# Thin boxes on the inner face of the finger tips, like the pads of the Menagerie Robotiq 2F-85.
# The finger tip collision hull spans y in [-0.006, 0.0135] and z in [0, 0.036].
PAD_BOXES = [
    ("pad1", (0, 0.0125, 0.009), (0.007, 0.001, 0.009)),
    ("pad2", (0, 0.0125, 0.027), (0.007, 0.001, 0.009)),
]

CONTACT_EXCLUDES = [
    ("base", "left_outer_bar"),
    ("base", "left_inner_bar"),
    ("base", "right_outer_bar"),
    ("base", "right_inner_bar"),
    ("left_outer_bar", "left_inner_bar"),
    ("right_outer_bar", "right_inner_bar"),
    ("left_inner_bar", "left_finger_tip"),
    ("right_inner_bar", "right_finger_tip"),
]

# (joint, reference joint, polynomial coefficient): joint = coef * reference. These replace the
# <mimic> tags of the URDF.
JOINT_COUPLINGS = [
    ("right_outer_knuckle_joint", "left_outer_knuckle_joint", 1.0),
    ("left_inner_knuckle_joint", "left_outer_knuckle_joint", 1.0),
    ("right_inner_knuckle_joint", "right_outer_knuckle_joint", 1.0),
    ("left_inner_finger_joint", "left_outer_knuckle_joint", -1.0),
    ("right_inner_finger_joint", "right_outer_knuckle_joint", -1.0),
]

SOLIMP = (0.95, 0.99, 0.001, 0.5, 2.0)
SOLREF = (0.005, 1.0)

XACRO_WRAPPER = """<?xml version="1.0"?>
<robot name="{model}" xmlns:xacro="http://wiki.ros.org/xacro">
  <xacro:arg name="mesh_prefix_path" default=""/>
  <xacro:include filename="{macro}"/>
  <link name="flange"/>
  <xacro:flexiv_GN01 prefix="" mesh_prefix_path="$(arg mesh_prefix_path)"/>
</robot>
"""

SCENE_XML = """<mujoco model="flexiv_grav scene">
  <include file="flexiv_grav.xml"/>

  <!-- Add some fluid viscosity to prevent the hanging box from jiggling forever -->
  <option viscosity="0.1"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="60" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <worldbody>
    <light pos="0 0 1"/>
    <light pos="0 -0.2 1" dir="0 0.2 -0.8" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
    <site size="0.002" pos="0 0 0.25" name="anchor"/>
    <body name="object" pos="0 0 0.19">
      <freejoint/>
      <geom type="box" size="0.02 0.02 0.02" rgba=".5 .7 .5 1" friction=".5" priority="1"/>
      <site size="0.002" pos="0.02 0.02 0.02" name="hook"/>
    </body>
  </worldbody>

  <tendon>
    <spatial limited="true" range="0 0.03" width="0.001">
      <site site="hook"/>
      <site site="anchor"/>
    </spatial>
  </tendon>
</mujoco>
"""

XML_COMMENTS = {
    "  <equality>": (
        "  <!--\n"
        "    The URDF drives the linkage through a virtual finger width joint and <mimic> tags:\n"
        "    the parallel bars copy the knuckle angle and the distal joint negates it, which keeps\n"
        "    the finger tips parallel. MuJoCo has no mimic joints, so these are equality constraints.\n"
        "  -->\n"
    ),
    "  <tendon>": (
        "  <!--\n"
        "    The tendon splits the actuator force equally between both knuckle joints so that the\n"
        "    equality constraint only has to correct small asymmetries. Its length is the knuckle angle.\n"
        "  -->\n"
    ),
    "  <actuator>": (
        "  <!--\n"
        f"    Position actuator whose control input is the finger opening width in meters (0 closed,\n"
        f"    {MAX_WIDTH} open). The URDF maps width to knuckle angle as theta = {WIDTH_GAIN} * width - {-WIDTH_OFFSET},\n"
        f"    so with kp = {KP:g} on the tendon length L the force is\n"
        f"    F = kp * ({WIDTH_GAIN} * ctrl - {-WIDTH_OFFSET} - L) - kv * L_dot, giving gainprm = kp * {WIDTH_GAIN}\n"
        f"    and biasprm = (-kp * {-WIDTH_OFFSET}, -kp, -kv). Each knuckle receives half of F.\n"
        "  -->\n"
    ),
}


def fetch_upstream(src: pathlib.Path) -> None:
    """Downloads the pinned upstream files that are not present in ``src`` yet."""
    for rel in UPSTREAM_FILES:
        dst = src / rel
        if dst.is_file():
            continue
        dst.parent.mkdir(parents=True, exist_ok=True)
        url = UPSTREAM_RAW + rel
        print(f"fetching {url}")
        with urllib.request.urlopen(url, timeout=60) as response:
            dst.write_bytes(response.read())


def copy_meshes(src: pathlib.Path, assets: pathlib.Path) -> None:
    assets.mkdir(parents=True, exist_ok=True)
    for rel in UPSTREAM_FILES:
        path = pathlib.Path(rel)
        if path.parts[0] != "meshes":
            continue
        if path.name == FINGER_TIP_OBJ:
            split_obj(src / rel, assets, FINGER_TIP_PARTS)
            continue
        name = path.name if path.parts[2] == "visual" else f"{path.stem}_collision{path.suffix}"
        shutil.copyfile(src / rel, assets / name)


def split_obj(obj: pathlib.Path, assets: pathlib.Path, parts: dict[str, list[str]]) -> None:
    """Writes one single-group OBJ per part, each holding the groups listed for it.

    Vertex lines are copied verbatim and re-indexed, normals and texture coordinates are dropped
    (MuJoCo recomputes the normals).
    """
    vertices: list[str] = []
    faces: dict[str, list[list[int]]] = {}
    group = None
    for line in obj.read_text().splitlines():
        if line.startswith("v "):
            vertices.append(line)
        elif line.startswith("g "):
            group = line[2:].strip()
            faces[group] = []
        elif line.startswith("f "):
            assert group is not None, "face before the first group"
            faces[group].append([int(corner.split("/")[0]) for corner in line[2:].split()])
    assert {g for groups in parts.values() for g in groups} == set(faces), f"groups in {obj.name}: {sorted(faces)}"

    for name, groups in parts.items():
        part_faces = [face for g in groups for face in faces[g]]
        used = sorted({index for face in part_faces for index in face})
        remap = {old: new for new, old in enumerate(used, start=1)}
        lines = [f"# {name}: groups {' '.join(groups)} of {obj.name}", f"g {name}"]
        lines += [vertices[index - 1] for index in used]
        lines += ["f " + " ".join(str(remap[index]) for index in face) for face in part_faces]
        (assets / f"{name}.obj").write_text("\n".join(lines) + "\n")


def mesh_mass_properties(path: pathlib.Path, density: float) -> tuple[float, np.ndarray, np.ndarray]:
    """Mass, center of mass and inertia tensor (about the link origin, in the link frame) of a mesh
    filled uniformly with the given density."""
    mesh = trimesh.load(path, force="mesh")  # merges the duplicate STL vertices so parts connect
    mesh.apply_scale(0.001)  # upstream meshes are in millimeters
    transform = np.eye(4)
    transform[:3, :3] = MESH_TO_LINK
    mesh.apply_transform(transform)
    mesh.fix_normals()
    mass = 0.0
    first_moment = np.zeros(3)
    inertia = np.zeros((3, 3))
    # Non-watertight meshes are handled part by part: the volume of a closed part has a consistent
    # sign after fix_normals, so summing the absolute volumes of all parts is robust to parts that
    # are oriented inwards.
    for part in mesh.split(only_watertight=False):
        if abs(part.volume) < 1e-12:
            continue  # open surface patches without volume
        part_mass = density * abs(part.volume)
        offset = part.center_mass
        mass += part_mass
        first_moment += part_mass * offset
        inertia += density * np.sign(part.volume) * part.moment_inertia + part_mass * (
            offset @ offset * np.eye(3) - np.outer(offset, offset)
        )
    return mass, first_moment / mass, inertia


def link_inertials(assets: pathlib.Path) -> dict[str, tuple[float, np.ndarray, np.ndarray]]:
    """Mass, center of mass and inertia tensor per body, derived from the meshes."""
    inertials: dict[str, tuple[float, np.ndarray, np.ndarray]] = {}
    for link, meshes in LINK_MESHES.items():
        mass = 0.0
        first_moment = np.zeros(3)
        inertia = np.zeros((3, 3))
        for file, density in meshes:
            m, com, i = mesh_mass_properties(assets / file, density)
            mass += m
            first_moment += m * com
            inertia += i
        inertials[link] = (mass, first_moment / mass, inertia)

    # The base gets what is left of the total mass, distributed uniformly over its collision hull.
    moving_mass = 2 * sum(mass for mass, _, _ in inertials.values())
    base_mass = TOTAL_MASS - moving_mass
    hull_mass, hull_com, hull_inertia = mesh_mass_properties(assets / BASE_HULL, 1.0)
    inertials["base"] = (base_mass, hull_com, hull_inertia * base_mass / hull_mass)

    for link in list(LINK_MESHES):
        for side in ["left", "right"]:
            inertials[f"{side}_{link}"] = inertials[link]
        del inertials[link]
    # MuJoCo wants the inertia about the center of mass.
    return {
        name: (mass, com, inertia - mass * (com @ com * np.eye(3) - np.outer(com, com)))
        for name, (mass, com, inertia) in inertials.items()
    }


def expand_xacro(src: pathlib.Path, work: pathlib.Path) -> str:
    wrapper = work / f"{MODEL_NAME}.urdf.xacro"
    wrapper.write_text(XACRO_WRAPPER.format(model=MODEL_NAME, macro=src / "urdf/common/grav_macro.xacro"))
    doc = xacro.process_file(str(wrapper), mappings={"mesh_prefix_path": f"{src}/"})
    return doc.toprettyxml(indent="  ")


def rewrite_urdf(urdf_xml: str, assets: pathlib.Path) -> tuple[str, dict[str, tuple[str, np.ndarray]]]:
    """Adapts the URDF for the MuJoCo importer and returns it with the removed TCP frames."""
    root = ET.fromstring(urdf_xml)
    links = {link.get("name"): link for link in root.findall("link")}
    joints = {joint.get("name"): joint for joint in root.findall("joint")}

    for mesh in root.iter("mesh"):
        path = pathlib.Path(mesh.get("filename"))
        collision = path.parent.name == "collision"
        mesh.set("filename", path.name if not collision else f"{path.stem}_collision{path.suffix}")

    # The gripper base becomes the root body.
    root.remove(links.pop("flange"))
    root.remove(joints.pop("gripper_base_joint"))

    # The virtual width joint has no physical counterpart and is replaced by the actuator mapping.
    root.remove(joints.pop("finger_width_joint"))
    root.remove(links.pop("unused_finger_width_link"))

    # The <mimic> tags become the equality constraints of JOINT_COUPLINGS (newer MuJoCo versions
    # would otherwise import them as unnamed equalities with default solver parameters).
    for joint in joints.values():
        for mimic in joint.findall("mimic"):
            joint.remove(mimic)

    tcps: dict[str, tuple[str, np.ndarray]] = {}
    for joint in list(joints.values()):
        child = joint.find("child").get("link")
        if child not in TCP_LINKS:
            continue
        xyz = np.array([float(v) for v in joint.find("origin").get("xyz").split()])
        tcps[child] = (joint.find("parent").get("link"), xyz)
        root.remove(joint)
        root.remove(links.pop(child))

    mj = ET.SubElement(root, "mujoco")
    ET.SubElement(mj, "compiler", meshdir=str(assets), discardvisual="false", fusestatic="false", strippath="false")
    return ET.tostring(root, encoding="unicode"), tcps


def build_spec(urdf: pathlib.Path, tcps: dict[str, tuple[str, np.ndarray]], assets: pathlib.Path) -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(urdf))
    spec.modelname = MODEL_NAME
    spec.compiler.autolimits = True
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10

    spec.add_material(name="black", rgba=(0.149, 0.149, 0.149, 1))
    spec.add_material(name="aluminum", rgba=(0.75, 0.76, 0.78, 1), specular=0.8, shininess=0.6)

    # Default classes. Values set on the elements below must match these for the XML writer to
    # omit them, so the same constants are used in both places.
    mesh_scale = (0.001, 0.001, 0.001)  # upstream meshes are in millimeters
    # The URDF rotates every mesh by 90 degrees about z. Baking the inverse into the mesh reference
    # frame moves the rotation from the geoms into the mesh assets.
    mesh_refquat = (0.7071068, 0, 0, -0.7071068)
    identity = (1, 0, 0, 0)
    main = spec.default
    grav = spec.add_default(ROOT_CLASS, main)
    grav.mesh.scale = mesh_scale  # refquat is not allowed in default classes, it is set per mesh
    grav.actuator.biastype = mujoco.mjtBias.mjBIAS_AFFINE
    grav.joint.axis = (1, 0, 0)
    grav.joint.armature = 0.001
    grav.joint.damping = (0.02, 0, 0)  # per-DOF vector in MjSpec, hinges use the first entry
    grav.joint.solimp_limit = SOLIMP
    grav.joint.solref_limit = SOLREF
    knuckle = spec.add_default("knuckle", grav)
    knuckle.joint.range = (WIDTH_OFFSET, WIDTH_GAIN * MAX_WIDTH + WIDTH_OFFSET)
    finger = spec.add_default("finger", grav)
    finger.joint.range = (-knuckle.joint.range[1], -knuckle.joint.range[0])
    visual = spec.add_default("visual", grav)
    visual.geom.type = mujoco.mjtGeom.mjGEOM_MESH
    visual.geom.contype = 0
    visual.geom.conaffinity = 0
    visual.geom.group = 2
    visual.geom.material = "black"
    visual.geom.density = 0
    collision = spec.add_default("collision", grav)
    collision.geom.type = mujoco.mjtGeom.mjGEOM_MESH
    collision.geom.group = 3
    pad = spec.add_default("pad", collision)
    pad.geom.type = mujoco.mjtGeom.mjGEOM_BOX
    pad.geom.friction = (1, 0.005, 0.0001)
    pad.geom.priority = 1
    pad.geom.solimp = SOLIMP
    pad.geom.solref = (0.004, 1)
    pad.geom.rgba = (0.3, 0.3, 0.3, 1)

    # The URDF references the multi-group finger_tip.obj, which is split into one mesh per part.
    tip_stem = pathlib.Path(FINGER_TIP_OBJ).stem
    pad_name, holder_name = FINGER_TIP_PARTS
    for mesh in spec.meshes:
        mesh.file = pathlib.Path(mesh.file).name
        if mesh.name == tip_stem:
            mesh.name = pad_name
            mesh.file = f"{pad_name}.obj"
        mesh.classname = grav
        mesh.scale = mesh_scale
        mesh.refquat = mesh_refquat
    spec.add_mesh(default=grav, name=holder_name, file=f"{holder_name}.obj", scale=mesh_scale, refquat=mesh_refquat)

    base = spec.body("grav_base_link")
    base.name = "base"
    base.childclass = ROOT_CLASS

    inertials = link_inertials(assets)
    for body in spec.bodies:
        if body.name not in inertials:
            continue
        mass, com, inertia = inertials[body.name]
        principal, axes = np.linalg.eigh(inertia)
        if np.linalg.det(axes) < 0:
            axes[:, 0] *= -1  # eigh may return a reflection, MuJoCo wants a rotation
        iquat = np.zeros(4)
        mujoco.mju_mat2Quat(iquat, axes.flatten())
        body.explicitinertial = True
        body.mass = round(mass, 5)
        body.ipos = np.round(com, 5)
        body.iquat = np.round(iquat, 6)
        body.fullinertia = [float("nan")] * 6  # NaN marks unset, the URDF importer filled it
        body.inertia = [float(f"{value:.4g}") for value in principal]

    for joint in spec.joints:
        joint.classname = knuckle if joint.range[1] > 0.5 else finger
        joint.axis = grav.joint.axis
        joint.armature = grav.joint.armature
        joint.damping = grav.joint.damping
        joint.solimp_limit = SOLIMP
        joint.solref_limit = SOLREF

    mount_half_angle = FINGER_MOUNT_VISUAL_TILT / 2
    mount_quat = (np.cos(mount_half_angle), np.sin(mount_half_angle), 0, 0)
    for geom in spec.geoms:
        is_visual = geom.contype == 0 and geom.conaffinity == 0
        cls = visual if is_visual else collision
        geom.classname = cls
        geom.group = cls.geom.group
        geom.quat = identity  # the URDF rotation is now part of the mesh
        if is_visual:
            geom.density = 0
            geom.material = "black"
            if geom.meshname == "finger_mount":
                geom.quat = mount_quat
            elif geom.meshname == tip_stem:
                geom.meshname = pad_name
                geom.material = FINGER_TIP_MATERIALS[pad_name]
        else:
            geom.density = main.geom.density

    for side in ["left", "right"]:
        tip = spec.body(f"{side}_finger_tip")
        holder = tip.add_geom(default=visual, meshname=holder_name, material=FINGER_TIP_MATERIALS[holder_name])
        holder.type = mujoco.mjtGeom.mjGEOM_MESH
        for suffix, pos, size in PAD_BOXES:
            geom = tip.add_geom(default=pad, name=f"{side}_{suffix}", pos=pos, size=size)
            geom.type = mujoco.mjtGeom.mjGEOM_BOX

    for name, (parent, xyz) in tcps.items():
        body = spec.body("base" if parent == "grav_base_link" else parent)
        body.add_site(name=name, pos=xyz, group=5, size=(0.002, 0.002, 0.002))

    for body1, body2 in CONTACT_EXCLUDES:
        spec.add_exclude(bodyname1=body1, bodyname2=body2)

    for joint, reference, coef in JOINT_COUPLINGS:
        eq = spec.add_equality(name1=joint, name2=reference, objtype=mujoco.mjtObj.mjOBJ_JOINT)
        eq.type = mujoco.mjtEq.mjEQ_JOINT
        eq.data[:5] = (0, coef, 0, 0, 0)
        eq.solimp = SOLIMP
        eq.solref = SOLREF

    tendon = spec.add_tendon(name="split")
    tendon.wrap_joint("left_outer_knuckle_joint", 0.5)
    tendon.wrap_joint("right_outer_knuckle_joint", 0.5)

    actuator = spec.add_actuator(default=grav, name="fingers_actuator")
    actuator.trntype = mujoco.mjtTrn.mjTRN_TENDON
    actuator.target = "split"
    actuator.gaintype = mujoco.mjtGain.mjGAIN_FIXED
    actuator.biastype = mujoco.mjtBias.mjBIAS_AFFINE
    actuator.gainprm[0] = KP * WIDTH_GAIN
    actuator.biasprm[:3] = (KP * WIDTH_OFFSET, -KP, -KV)
    actuator.ctrlrange = (0, MAX_WIDTH)
    actuator.forcerange = (-FORCERANGE, FORCERANGE)
    return spec


def add_comments(xml: str) -> str:
    for anchor, comment in XML_COMMENTS.items():
        assert xml.count(f"\n{anchor}\n") == 1, anchor
        xml = xml.replace(f"\n{anchor}\n", f"\n{comment}{anchor}\n", 1)
    return xml


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=pathlib.Path, required=True, help="output model directory")
    parser.add_argument(
        "--src",
        type=pathlib.Path,
        default=None,
        help="flexiv_description checkout; defaults to a cache directory filled from GitHub at the pinned commit",
    )
    parser.add_argument("--work", type=pathlib.Path, default=None, help="keep intermediate URDF files here")
    args = parser.parse_args()

    src = args.src or pathlib.Path(tempfile.gettempdir()) / f"flexiv_description-{UPSTREAM_COMMIT[:12]}"
    fetch_upstream(src)
    work = args.work or pathlib.Path(tempfile.mkdtemp(prefix="flexiv_grav_"))
    work.mkdir(parents=True, exist_ok=True)
    out = args.out
    assets = out / "assets"

    copy_meshes(src, assets)
    urdf_xml, tcps = rewrite_urdf(expand_xacro(src, work), assets.resolve())
    urdf = work / f"{MODEL_NAME}.urdf"
    urdf.write_text(urdf_xml)

    spec = build_spec(urdf, tcps, assets)
    spec.compile()  # fail early on model errors, mesh paths are still absolute here
    spec.modelfiledir = str(out.resolve())  # to_xml recompiles, so relative mesh paths must resolve
    spec.compiler.meshdir = "assets"
    (out / f"{MODEL_NAME}.xml").write_text(add_comments(spec.to_xml()))
    (out / "scene.xml").write_text(SCENE_XML)
    shutil.copyfile(src / "LICENSE", out / "LICENSE")
    print(f"wrote {out / f'{MODEL_NAME}.xml'} (intermediate files in {work})")


if __name__ == "__main__":
    main()
