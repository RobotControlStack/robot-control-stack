# Rizon 4s / Grav D405 plate mount

3D-printable mount for an Intel RealSense D405 on a Flexiv Rizon 4s that carries the Flexiv
Grav gripper. The part is an 8 mm spacer plate sandwiched between the Rizon tool flange
(ISO 9409-1-50-4-M6) and the Grav, with a short bracket that holds the camera on the front of
the gripper (the side facing away from the robot base in the rcs home pose), tilted 20° towards
the finger tips. The whole camera stays below the shoulder of the Grav housing, so it does not
add reach along the gripper axis.

Because the plate shifts the gripper and its TCP by 8 mm, this variant is not the one used by
the rcs scene; see `../rizon_d405_wrist_mount`, which clamps to the accessory holes of the Rizon
wrist link instead and leaves the gripper where it is.

Files:

- `rizon_grav_d405_plate_mount.scad` – parametric OpenSCAD source (all dimensions in mm)
- `build_meshes.py` – regenerates the STLs with OpenSCAD (`uv run python build_meshes.py`)
- `assets/rizon_grav_d405_plate_mount.stl` – printable part, also the MuJoCo visual mesh
- `assets/rizon_grav_d405_plate_mount_collision.stl` – convex bracket used as the MuJoCo collision mesh
- `rizon_grav_d405_plate_mount.xml` – MJCF, registered as `OBJECT_PATHS["rizon_grav_d405_plate_mount"]`

## Frames

Everything is defined in the Rizon attachment-site (flange) frame: z points out of the flange
towards the fingers, the Grav fingers open along y, the camera sits on the -x side. In rcs the
mount is placed with `DEFAULT_TRANSFORMS["RIZON_GRAV_D405_PLATE_MOUNT"]` (identity), the Grav has to be
shifted by `RIZON_GRAV_D405_PLATE_MOUNT_GRIPPER_OFFSET` (8 mm along z) and the D405 body sits at
`RIZON_GRAV_D405_PLATE_MOUNT_CAMERA`: front glass centre at (-65, 0, 60) mm, optical axis 20°
off the flange z axis towards the fingers, image up pointing away from the gripper so the
fingers show at the bottom of the image, stereo baseline (M4 threads and USB-C port) along
flange y, so the plug points sideways.

To use this variant, apply `RIZON_GRAV_D405_PLATE_MOUNT_GRIPPER_OFFSET` as gripper offset and
compose it into the TCP offset, in sim and on the hardware.

## Hardware

- 4x M6 screws, 8 mm longer than the ones that hold the Grav on the bare flange (they pass
  through the Grav base and the plate into the flange threads, bolt circle 50 mm)
- 1x 6 mm dowel pin, long enough to engage flange and Grav through the 8 mm plate. The plate
  has pin holes at 0°, 180° and 270° (the same positions the Grav offers), use the one that
  matches the flange pin.
- 2x M4x12 countersunk screws (DIN 7991) for the camera, driven from below the 7.5 mm shelf
  into the two M4 threads on the back of the D405 (20 mm apart along the baseline, 5.5 mm deep;
  4.5 mm engagement). The middle between the two side webs is open for the screwdriver.
- Any USB-C cable; the port faces sideways.

## Printing

Print with the plate on the bed (the shelf underside is a 20° overhang that needs support) or
lying on one of the side webs, which prints support-free. 4+ perimeters and 40 % infill; PETG
or PLA are fine for the 58 g camera. The part is about 53 cm³, i.e. roughly 65 g in PLA.

## Design references

The Grav envelope (86.6 mm skirt, 84 mm body, 63 mm contact face) and the Rizon flange pattern
(4x M6 on a 50 mm circle at 45°, 31.5 mm pilot) were measured from the meshes in
`assets/grippers/flexiv_grav` and `assets/robots/rizon4s`; the D405 dimensions (42 x 42 x 23 mm,
two M4 threads on the back 20 mm apart along the baseline) from the Intel D400 series datasheet and the D405 mesh in
`assets/cameras/d405`.
