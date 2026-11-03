"""Builds the mesh of the mounting plate of the Franka FR3 Duo pedestal (installation guide R04240, "P2.1 Mounting
Plate"), used by fr3_duo_mount_franka_pedestal.xml.

Requires shapely and mapbox_earcut (for trimesh.creation.extrude_polygon).

Modeled from photos with a ruler (no CAD available): 280 x 280 x 12 mm aluminium plate, rounded corners,
- 11 x 11 hole grid, 22.5 mm pitch, center region open: rows 1-3 / 9-11 full, rows 4-8 only the two outer
  columns on each side; M8 threads (modeled as 6.8 mm holes)
- 8 counterbored through holes in the outer ring (counterbore on the top side, screws into the pedestal)
- 2 x 3 through holes at 40 mm pitch in the center for the item 120x80 profile (120 along x), counterbored
  on the bottom side (countersunk screws from below), with 4 pin holes (blind, from the top) in between

Origin: center of the bottom face, z up.
"""

from pathlib import Path

import shapely
import trimesh
from shapely.geometry import Point, box

OUT = Path(__file__).parent / "assets" / "fr3_duo_mounting_plate.obj"

SIZE = 0.28
THICKNESS = 0.012
CORNER_RADIUS = 0.008
PITCH = 0.0225
GRID = 11
THREAD_D = 0.0068
THROUGH_D = 0.009
COUNTERBORE_D = 0.0145
COUNTERBORE_DEPTH = 0.005
PIN_D = 0.006
PIN_DEPTH = 0.006
# (row, col) of the counterbored outer holes, row 0 = +y side
COUNTERBORED = {(0, 2), (0, 8), (2, 0), (2, 10), (8, 0), (8, 10), (10, 2), (10, 8)}
PROFILE_HOLES = [(x, y) for x in (-0.04, 0.0, 0.04) for y in (-0.02, 0.02)]
PIN_HOLES = [(x, y) for x in (-0.02, 0.02) for y in (-0.02, 0.02)]


def circle(x, y, d):
    return Point(x, y).buffer(d / 2, quad_segs=8)


def grid_holes():
    for row in range(GRID):
        for col in range(GRID):
            if 3 <= row <= 7 and 2 <= col <= 8:
                continue
            x = (col - (GRID - 1) / 2) * PITCH
            y = ((GRID - 1) / 2 - row) * PITCH
            yield x, y, (row, col) in COUNTERBORED


def layer(z0, z1, holes):
    outline = box(-SIZE / 2, -SIZE / 2, SIZE / 2, SIZE / 2).buffer(-CORNER_RADIUS).buffer(CORNER_RADIUS, quad_segs=8)
    poly = outline.difference(shapely.union_all([circle(*h) for h in holes]))
    m = trimesh.creation.extrude_polygon(poly, z1 - z0)
    m.apply_translation([0, 0, z0])
    return m


def main():
    grid = list(grid_holes())
    # layers bottom to top, each with the hole diameters present at that height
    bottom = layer(
        0.0,
        COUNTERBORE_DEPTH,
        [(x, y, THROUGH_D if cb else THREAD_D) for x, y, cb in grid]
        + [(x, y, COUNTERBORE_D) for x, y in PROFILE_HOLES],
    )
    middle = layer(
        COUNTERBORE_DEPTH,
        THICKNESS - max(COUNTERBORE_DEPTH, PIN_DEPTH),
        [(x, y, THROUGH_D if cb else THREAD_D) for x, y, cb in grid] + [(x, y, THROUGH_D) for x, y in PROFILE_HOLES],
    )
    top = layer(
        THICKNESS - max(COUNTERBORE_DEPTH, PIN_DEPTH),
        THICKNESS,
        [(x, y, COUNTERBORE_D if cb else THREAD_D) for x, y, cb in grid]
        + [(x, y, THROUGH_D) for x, y in PROFILE_HOLES]
        + [(x, y, PIN_D) for x, y in PIN_HOLES],
    )
    plate = trimesh.util.concatenate([bottom, middle, top])
    OUT.parent.mkdir(exist_ok=True)
    plate.export(OUT)
    print(f"wrote {OUT}: {len(plate.faces)} triangles")


if __name__ == "__main__":
    main()
