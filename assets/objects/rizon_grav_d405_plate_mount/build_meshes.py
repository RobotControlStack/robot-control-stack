"""Builds the STL meshes of the Rizon/Grav D405 wrist mount from the OpenSCAD source.

Requires the `openscad` binary (tested with 2021.01). The visual mesh is the printable part,
the collision mesh is the convex bracket without the spacer plate: the plate is enclosed by
the flange and the gripper, and its convex hull together with the bracket would fill the
volume where the gripper sits.
"""

import subprocess
from pathlib import Path

HERE = Path(__file__).parent
SCAD = HERE / "rizon_grav_d405_plate_mount.scad"
ASSETS = HERE / "assets"

MESHES = {
    "rizon_grav_d405_plate_mount.stl": "visual",
    "rizon_grav_d405_plate_mount_collision.stl": "collision",
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


if __name__ == "__main__":
    main()
