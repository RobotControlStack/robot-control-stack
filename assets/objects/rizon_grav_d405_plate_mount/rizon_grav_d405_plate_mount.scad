// Wrist mount for an Intel RealSense D405 on a Flexiv Rizon 4s carrying the Flexiv Grav gripper.
//
// The part is an 8 mm spacer plate that is sandwiched between the Rizon tool flange
// (ISO 9409-1-50-4-M6) and the Grav, plus a short bracket that holds the camera on the
// front of the gripper, tilted towards the finger tips. Everything is expressed in the
// Rizon attachment-site (flange) frame in millimetres: z points out of the flange towards
// the fingers, the Grav fingers open along y, and the camera sits on the -x side, which
// faces away from the robot base in the rcs home pose.
//
// Build with `python build_meshes.py` (runs OpenSCAD for the visual and collision STLs).
// Numbers of the Grav envelope and the D405 were measured from the meshes in
// assets/grippers/flexiv_grav and assets/cameras/d405, the flange pattern from the Rizon
// link7 mesh, the D405 hole positions from the Intel D400 series datasheet.

part = "visual"; // "visual": full part, "collision": convex bracket for MuJoCo
$fn = 96;

// ---- Spacer plate / flange interface (ISO 9409-1-50-4-M6) ---------------------------
plate_d = 63;   // Rizon flange and Grav contact face are both 63 mm
plate_t = 8;    // the Grav and its TCP move out by this amount
bolt_pcd = 50;
bolt_d = 6.4;   // M6 clearance, the original Grav screws are reused 8 mm longer
bolt_angles = [45, 135, 225, 315];
pin_d = 6.1;    // 6 mm dowel pin
pin_angles = [0, 180, 270]; // the Grav has pin holes here, 90 deg is taken by its notch
bore_d = 32;    // clears the ISO 31.5 mm pilot, also passes cables

// ---- Bracket ------------------------------------------------------------------------
arm_x_in = -45; // inner face, 1.7 mm off the 86.6 mm Grav skirt, 3 mm off its 84 mm body
arm_t = 8;
arm_w = 46;
arm_h = 30;
web_t = 4;      // two side webs support the camera shelf, the middle stays open for the screws

// ---- Camera: Intel RealSense D405, 42 x 42 x 23 mm ----------------------------------
cam_w = 42;
cam_h = 42;
cam_d = 23;
cam_tol = 0.15;
cam_alpha = 20;           // tilt of the optical axis towards the fingers [deg]
cam_o = [-65, 0, 60];     // front glass centre in the flange frame [mm]
shelf_t = 7.5;           // M4x12 DIN 7991 through it: 4.5 mm engagement in the 5.5 mm threads
shelf_margin = 2;         // shelf and rim overhang around the camera footprint
rim_h = 3;
m4_hole_d = 4.5;         // two M4 threads on the camera back, 20 mm apart along the baseline, 5.5 mm deep
m4_spacing = 20;
m4_csk_d = 8.6;          // 90 deg countersink for DIN 7991 M4 heads (dk = 8) plus clearance

// Camera frame in the flange frame (the body frame of assets/cameras/d405): x is the optical axis,
// y the housing top (image up), z the stereo baseline. Image up points away from the gripper so
// that the fingers show at the bottom of the image; the baseline, the two M4 threads on the back
// and the USB-C port (on the -z face) run along flange y, so the plug points sideways, clear of
// the gripper.
ca = cam_alpha;
cam_x = [sin(ca), 0, cos(ca)];
cam_y = [-cos(ca), 0, sin(ca)];
cam_z = [0, -1, 0];
cam_mat = [
  [cam_x[0], cam_y[0], cam_z[0], cam_o[0]],
  [cam_x[1], cam_y[1], cam_z[1], cam_o[1]],
  [cam_x[2], cam_y[2], cam_z[2], cam_o[2]],
  [0, 0, 0, 1],
];

module cam_frame() {
  multmatrix(cam_mat) children();
}

module arm() {
  translate([arm_x_in - arm_t, -arm_w / 2, 0]) cube([arm_t, arm_w, arm_h]);
}

// Slab behind the camera back face, top lowered by `sink` (used for the collision mesh so
// that it does not touch the camera collision mesh).
module shelf(sink = 0) {
  translate([-cam_d - shelf_t, -(cam_w / 2 + shelf_margin), -(cam_h / 2 + shelf_margin)])
    cube([shelf_t - sink, cam_w + 2 * shelf_margin, cam_h + 2 * shelf_margin]);
}

// Rim around the camera back on three sides, open on the connector (-z) side.
module rim() {
  difference() {
    translate([-cam_d, -(cam_w / 2 + shelf_margin), -(cam_h / 2 + shelf_margin)])
      cube([rim_h, cam_w + 2 * shelf_margin, cam_h + 2 * shelf_margin]);
    translate([-cam_d - 1, -(cam_w / 2 + cam_tol), -(cam_h / 2 + shelf_margin + 1)])
      cube([rim_h + 2, cam_w + 2 * cam_tol, cam_h + shelf_margin + 1 + cam_tol]);
  }
}

// Camera screws from below the shelf: M4 through holes with 90 deg countersinks flush with the
// shelf underside, on the baseline (z) axis 20 mm apart.
module m4_holes() {
  csk_h = (m4_csk_d - m4_hole_d) / 2;
  for (s = [-1, 1])
    translate([-cam_d - shelf_t, 0, s * m4_spacing / 2]) rotate([0, 90, 0]) {
      translate([0, 0, -1]) cylinder(d = m4_hole_d, h = shelf_t + 2);
      translate([0, 0, -1]) cylinder(d1 = m4_csk_d + 2, d2 = m4_csk_d, h = 1);
      cylinder(d1 = m4_csk_d, d2 = m4_hole_d, h = csk_h);
    }
}

module bracket_hull(sink = 0) {
  hull() {
    arm();
    cam_frame() shelf(sink);
  }
}

module webs() {
  intersection() {
    bracket_hull();
    union() {
      for (s = [-1, 1])
        translate([-200, s * (arm_w / 2 - web_t / 2) - web_t / 2, -1]) cube([400, web_t, 200]);
    }
  }
}

module plate() {
  difference() {
    hull() {
      cylinder(d = plate_d, h = plate_t);
      translate([arm_x_in - arm_t, -arm_w / 2, 0]) cube([arm_t, arm_w, plate_t]);
    }
    translate([0, 0, -1]) cylinder(d = bore_d, h = plate_t + 2);
    for (a = bolt_angles)
      rotate([0, 0, a]) translate([bolt_pcd / 2, 0, -1]) cylinder(d = bolt_d, h = plate_t + 2);
    for (a = pin_angles)
      rotate([0, 0, a]) translate([bolt_pcd / 2, 0, -1]) cylinder(d = pin_d, h = plate_t + 2);
  }
}

module visual() {
  difference() {
    union() {
      plate();
      arm();
      webs();
      cam_frame() {
        shelf();
        rim();
      }
    }
    cam_frame() m4_holes();
  }
}

if (part == "collision") bracket_hull(1);
else visual();
