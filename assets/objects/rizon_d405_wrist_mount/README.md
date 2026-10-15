# Rizon 4s D405 wrist mount

3D-printable mount for an Intel RealSense D405 on a Flexiv Rizon 4s. A curved rail is screwed to
the accessory pocket on the front of the wrist link (link7), 78 mm above the tool flange, runs
down the link past the flange and the Grav skirt and carries the camera on an inclined shelf (two side webs and a
centre web) on the front of the gripper (the side facing away from the robot base in the rcs home pose), tilted
20° towards the finger tips. The gripper stays bolted directly to the flange, so gripper and TCP
offsets are unchanged. The camera sits below the shoulder of the Grav housing, so no reach is
added along the gripper axis.

Files:

- `rizon_d405_wrist_mount.scad` – parametric OpenSCAD source (all dimensions in mm)
- `build_meshes.py` – regenerates the STLs with OpenSCAD (`uv run --with networkx --with lxml python build_meshes.py`)
- `assets/rizon_d405_wrist_mount.stl` – printable part, also the MuJoCo visual mesh
- `assets/rizon_d405_wrist_mount_collision.stl` – convex lower bracket used as the MuJoCo collision mesh
- `print/` – the part rotated onto its flat side face, as STL and 3MF, for the slicer
- `rizon_d405_wrist_mount.xml` – MJCF, registered as `OBJECT_PATHS["rizon_d405_wrist_mount"]`

A spacer-plate variant that goes between flange and Grav lives in `../rizon_grav_d405_plate_mount`.

## Mounting interface

The wrist link has three identical accessory pockets 78 mm above the flange face: on the front,
on the back and on one side (positions from the Menagerie link7 mesh, dimensions measured on the
robot). Each is a flat milled into the cylinder, 37.5 mm wide (chord), 22 mm tall and 3.5 mm
deep at the centre, with two M5 threads 18 mm apart, 5 mm deep and perpendicular to the flat,
and an 8 mm alignment hole, 4 mm deep, in the middle; two deep curved pockets flank it. The rail
carries a 35 x 21.4 mm block on its inside that fills the pocket and rests on the flat (1.25 mm
clearance to the pocket sides, 0.3 mm top and bottom, 0.2 mm stand-off that the screws close),
and a Ø7.7 mm pin, 3.7 mm long with a chamfered tip and a 45° V underneath for printing, that
drops 3.5 mm into the alignment hole. The rail itself is 46 mm wide with flat side faces (planes
at y = ±23, not radial cuts), its inner surface follows the slight taper of the link (Ø98.0
near the flange, Ø99.2 at the pocket) with a 0.3 mm gap; over the pocket it spans the rims of
the curved side pockets, which are recessed, so nothing touches there.

## Frames

Everything is defined in the Rizon attachment-site (flange) frame: z points out of the flange
towards the fingers, the Grav fingers open along y, the camera sits on the -x side, the wrist
link occupies z < 0. In rcs the mount is placed with `DEFAULT_TRANSFORMS["RIZON_D405_WRIST_MOUNT"]`
(identity) and the D405 body with `RIZON_D405_WRIST_CAMERA`: front glass centre at
(-65, 0, 52) mm, optical axis 20° off the flange z axis towards the fingers, image up pointing
away from the gripper so that the fingers show at the bottom of the image, and the stereo
baseline along flange y. The D405 housing (see `assets/cameras/d405/d405.xml`) has its two M4
threads on the back along the baseline and the USB-C port on the side face at the end of the
baseline, so the plug points sideways along the gripper and needs no extra room.

## Hardware

- 2x M5x12 countersunk hex socket screws (DIN 7991, length includes the head), perpendicular to
  the flat, 18 mm apart. The rail is 10.6 mm thick at the screws (7.5 mm rail + 3.5 mm block
  minus the gaps), so the head seats are spot-faced 2.6 mm deep (Ø10.6) above the 90°
  countersinks: the screws then engage 4.0 mm of the 5 mm deep threads and stop 1 mm short of the
  bottom. M5x16 would bottom out, M5x8 does not reach. `screw_len` and `engagement` in the
  `.scad` set the spot-face depth.
- 2x M4x12 countersunk hex socket screws (DIN 7991) for the camera, driven from below the shelf
  into the two M4 threads on the back of the D405 (20 mm apart along the baseline, 5.5 mm deep).
  The shelf is 7.5 mm thick with 90° countersinks, so the screws engage 4.5 mm and stop 1 mm
  short of the bottom; M4x8 would only engage 0.5 mm, M4x16 bottoms out. The middle between the
  two side webs is open for the screwdriver.
- Any USB-C cable; the port faces sideways (along the finger opening direction).

## Printing

`print/` holds the part rotated onto its side (`rizon_d405_wrist_mount_print_oriented.stl` and
the same as `rizon_d405_wrist_mount.3mf`). Bambu printers only print sliced files, so open one of
them in Bambu Studio, slice, and use *Export plate sliced file* (`.gcode.3mf`) for the USB stick.

Orientation: print it lying on one of its two flat side faces (as in the files in `print/`), not
on the curved rail. The rail is cut by the same two planes as the webs and the shelf, so the
part rests on a full 46 x 133 mm face and needs no support: both curved faces of the rail, the
flat block, the camera contact face and all holes print as walls, the pin has a 45° V on its
underside, and the layers run along the rail, i.e. along the bending load of the camera. The
only horizontal undersides are the centre web and the upper side web over the two 16 mm gaps
between the webs; they are anchored on three sides (rail, shelf, sloped web face) and print as
short bridges, so supports can stay off.

Putting the concave face on the bed would need support on the mating surface and its scars
would eat the 0.3 mm gap; the convex face down would need support everywhere.

Settings for PLA: 0.2 mm layers, 4 walls, 4 top/bottom layers, 25–40 % gyroid or grid infill
(the part is mostly walls anyway, infill matters little), no brim needed. The part is about
50 cm³, roughly 60 g in PLA. PETG is the better choice if the arm gets warm or the mount stays
on for long; use the same settings.

## Design references

The wrist link envelope (tapered 98–99.2 mm cylinder, accessory pockets 78 mm above the flange)
and the Grav envelope (86.6 mm skirt, 84 mm body) were measured from the meshes in
`assets/robots/rizon4s` and `assets/grippers/flexiv_grav`; the D405 dimensions
(42 x 42 x 23 mm, two M4 threads on the back 20 mm apart along the baseline, USB-C on the side,
1/4-20 on the bottom) from the Intel D400 series datasheet and the D405 mesh in
`assets/cameras/d405`.
