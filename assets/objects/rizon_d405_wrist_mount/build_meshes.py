"""Builds the STL meshes of the Rizon D405 wrist mount from the OpenSCAD source.

Requires the `openscad` binary (tested with 2021.01). The visual mesh is the printable part,
the collision mesh is the convex lower bracket without the rail: the rail hugs the wrist link, so
its convex hull would cut into the link.

Also writes print-ready copies to `print/`: the part rotated onto its side web (the recommended
print orientation, see README) as STL and, if `networkx` and `lxml` are available for trimesh, as 3MF for
Bambu Studio / OrcaSlicer.
"""

import subprocess
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).parent
SCAD = HERE / "rizon_d405_wrist_mount.scad"
ASSETS = HERE / "assets"
PRINT = HERE / "print"

MESHES = {
    "rizon_d405_wrist_mount.stl": "visual",
    "rizon_d405_wrist_mount_collision.stl": "collision",
}


def main():
    ASSETS.mkdir(exist_ok=True)
    for filename, part in MESHES.items():
        out = ASSETS / filename
        subprocess.run(
            ["openscad", "-o", str(out), "--export-format", "binstl", "-D", f'part="{part}"', str(SCAD)],
            check=True,
        )
        print(f"wrote {out}")
    export_print_files()


def export_print_files():
    PRINT.mkdir(exist_ok=True)
    mesh = trimesh.load(ASSETS / "rizon_d405_wrist_mount.stl", force="mesh")
    # lie on the side web: flange-frame y becomes the print z axis
    mesh.apply_transform(trimesh.transformations.rotation_matrix(np.deg2rad(90), [1, 0, 0]))
    mesh.apply_translation(-mesh.bounds[0])
    out = PRINT / "rizon_d405_wrist_mount_print_oriented.stl"
    mesh.export(out)
    print(f"wrote {out}")
    try:
        out = PRINT / "rizon_d405_wrist_mount.3mf"
        mesh.export(out)
        print(f"wrote {out}")
    except ImportError as e:
        print(f"skipped 3mf export ({e}); run with `uv run --with networkx --with lxml python build_meshes.py`")


if __name__ == "__main__":
    main()
