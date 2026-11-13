// Wrist mount for an Intel RealSense D405 on a Flexiv Rizon 4s, screwed to the accessory pocket
// on the front of the wrist link (link7). The gripper stays bolted directly to the flange, so no
// gripper or TCP offset changes.
//
// The Rizon wrist link is a slightly tapered ~98 mm cylinder. 78 mm above the tool flange it has
// three accessory pockets (front, back and one side): a flat milled into the cylinder, 37.5 mm
// wide, 22 mm tall and 3.5 mm deep at the centre, with two M5 threads 18 mm apart (5 mm deep,
// perpendicular to the flat) and an 8 mm alignment hole, 4 mm deep, in the middle, flanked by
// two deep curved pockets. A curved rail sits along the front of the link between the curved
// pockets; a flat block on its inside fills the pocket and rests on the flat, a pin locates in
// the alignment hole and two countersunk M5x12 screws hold it. The rail runs down past the
// flange and the Grav skirt and carries the camera on an inclined shelf, tilted towards the
// finger tips, connected to the rail by two side webs and a centre web.
//
// Everything is expressed in the Rizon attachment-site (flange) frame in millimetres: z points
// out of the flange towards the fingers, the Grav fingers open along y, and the camera sits on
// the -x side, which faces away from the robot base in the rcs home pose. The wrist link
// occupies z < 0.
//
// Build with `python build_meshes.py` (runs OpenSCAD for the visual and collision STLs).
// Wrist link and Grav envelopes were measured from the meshes in assets/robots/rizon4s and
// assets/grippers/flexiv_grav, the D405 from assets/cameras/d405 and the Intel datasheet.

part = "visual"; // "visual": full part, "collision": convex lower bracket for MuJoCo
$fn = 96;

// ---- Wrist link (link7) ---------------------------------------------------------------
link_r_low = 49.0;       // link radius near the flange (z > -40)
link_r_high = 49.6;      // link radius at the top end of the rail (z = rail_z0)
pocket_z = -77.8;        // accessory pocket centre, 77.8 mm above the flange face
pocket_depth = 3.5;      // the flat is this deep below the cylinder at its centre
pocket_w = 37.5;         // chord width of the flat
pocket_h = 22;           // height of the flat along the link
flat_r = link_r_high - pocket_depth;   // distance of the flat from the link axis
flat_gap = 0.2;          // block face stands off the flat by this much before the screws pull it in
block_side_clearance = 1.25;
block_z_clearance = 0.3;
hole_spacing = 18;       // the two M5 threads, perpendicular to the flat, 5 mm deep
screw_d = 5.5;           // M5 clearance
csk_d = 10.6;            // 90 deg countersink for DIN 7991 M5 heads (dk = 10) plus clearance
screw_len = 12;          // M5x12 DIN 7991 (length includes the head)
engagement = 4;          // thread engagement to aim for; the head seat is spot-faced to get it
pin_d = 8 - 0.3;         // alignment pin, 8 mm hole with radial clearance
pin_h = 4 - 0.5 + flat_gap; // protrusion from the block face: 4 mm hole depth, 0.5 mm clearance
pin_chamfer = 0.5;
pin_v_bottom = true;     // 45 deg V on the -y side of the pin so that it prints without support
                         // when the part lies on its -y side face (see README / print/)

// ---- Rail -------------------------------------------------------------------------------
rail_gap = 0.3;
rail_t = 7.5;
rail_w = 46;             // same width as the shelf; the side faces are flat (planes y = +-23) so the
                         // whole part has two flat faces and prints lying on one of them without support
rail_z0 = -92;           // top end, 14 mm above the pocket
rail_z1 = 14;            // bottom end, next to the Grav body just above the camera shelf
web_t = 4;               // two side webs support the camera shelf
web_center_t = 6;        // centre web between the two camera screws (which sit at y = +-10)
hull_z0 = -4;            // webs and collision hull start where the link has narrowed to the flange neck

// ---- Camera: Intel RealSense D405, 42 x 42 x 23 mm ------------------------------------
cam_w = 42;
cam_h = 42;
cam_d = 23;
cam_tol = 0.15;
cam_alpha = 20;          // tilt of the optical axis towards the fingers [deg]
cam_o = [-65, 0, 52];    // front glass centre in the flange frame [mm]
shelf_t = 7.5;           // M4x12 DIN 7991 through it: 4.5 mm engagement in the 5.5 mm threads
shelf_margin = 2;        // shelf and rim overhang around the camera footprint
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

// Inner radius of the rail: follows the taper of the link with a constant gap.
function rail_r_in(z) = rail_gap + link_r_low + (link_r_high - link_r_low) * (z - rail_z1) / (rail_z0 - rail_z1);

// Conical shell hugging the front of the link, cut to rail_w by two planes parallel to the
// x-z plane (not radially), so that its side faces are coplanar with the webs and the shelf.
module rail() {
  intersection() {
    rotate([0, 0, 180 - 60])
      rotate_extrude(angle = 120)
        polygon([
          [rail_r_in(rail_z1), rail_z1],
          [rail_r_in(rail_z1) + rail_t, rail_z1],
          [rail_r_in(rail_z0) + rail_t, rail_z0],
          [rail_r_in(rail_z0), rail_z0],
        ]);
    translate([-200, -rail_w / 2, -200]) cube([400, rail_w, 400]);
  }
}

// Flat block on the inside of the rail that fills the pocket and rests on its flat.
module pocket_block() {
  r_in = rail_r_in(pocket_z);
  translate([-(r_in + 1), -(pocket_w / 2 - block_side_clearance), pocket_z - (pocket_h / 2 - block_z_clearance)])
    cube([r_in + 1 - (flat_r + flat_gap), pocket_w - 2 * block_side_clearance, pocket_h - 2 * block_z_clearance]);
}

// Cylindrical pin, perpendicular to the flat, that locates the rail in the alignment hole. With
// pin_v_bottom the lower quarter (towards -y, the bed side of the print orientation) is replaced
// by two 45 deg faces meeting at the bottom of the circle, so the horizontal pin prints without
// support and still touches the hole at the top, the sides and the bottom apex.
module alignment_pin() {
  x0 = flat_r + flat_gap;   // block face
  intersection() {
    translate([-(x0 + 1), 0, pocket_z]) rotate([0, 90, 0]) {
      cylinder(d = pin_d, h = 1 + pin_h - pin_chamfer);
      translate([0, 0, 1 + pin_h - pin_chamfer]) cylinder(d1 = pin_d, d2 = pin_d - 2 * pin_chamfer, h = pin_chamfer);
    }
    if (pin_v_bottom)
      translate([-(x0 + 2), -pin_d / 2, pocket_z]) rotate([-45, 0, 0]) cube([pin_h + 4, pin_d, pin_d]);
    else
      translate([-(x0 + 2), -pin_d, pocket_z - pin_d]) cube([pin_h + 4, 2 * pin_d, 2 * pin_d]);
  }
}

// Screw holes perpendicular to the flat. The head seat is spot-faced down to seat_x so that the
// screw engages `engagement` mm of thread: the thread starts at the flat (flat_r), the rail's
// outer surface at the screws is further out than screw_len - engagement.
module rail_screw_holes() {
  seat_x = flat_r + screw_len - engagement;
  csk_h = (csk_d - screw_d) / 2;
  for (s = [-1, 1])
    translate([0, s * hole_spacing / 2, pocket_z]) {
      translate([-(flat_r - 2), 0, 0]) rotate([0, -90, 0]) cylinder(d = screw_d, h = 30);
      translate([-seat_x, 0, 0]) rotate([0, 90, 0]) cylinder(d1 = csk_d, d2 = screw_d, h = csk_h);
      translate([-seat_x, 0, 0]) rotate([0, -90, 0]) cylinder(d = csk_d, h = 30);
    }
}

// Lower part of the rail that the webs and the collision hull attach to. Above hull_z0 the link
// is still at full radius, and a convex hull of the curved rail would cut into it.
module rail_lower() {
  intersection() {
    rail();
    translate([-200, -200, hull_z0]) cube([400, 400, rail_z1 - hull_z0 + 1]);
  }
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

// y_clip narrows the rail part of the hull; the collision mesh uses it so that the chord between
// the rail's inner corners stays clear of the Grav's (coarse) collision hull.
module bracket_hull(sink = 0, y_clip = rail_w / 2) {
  hull() {
    intersection() {
      rail_lower();
      translate([-200, -y_clip, -200]) cube([400, 2 * y_clip, 400]);
    }
    cam_frame() shelf(sink);
  }
}

module webs() {
  web_y = (cam_w / 2 + shelf_margin) - web_t;
  intersection() {
    bracket_hull();
    union() {
      for (s = [-1, 1])
        translate([-200, s * (web_y + web_t / 2) - web_t / 2, -100]) cube([400, web_t, 300]);
      translate([-200, -web_center_t / 2, -100]) cube([400, web_center_t, 300]);
    }
  }
}

module visual() {
  difference() {
    union() {
      rail();
      pocket_block();
      alignment_pin();
      webs();
      cam_frame() {
        shelf();
        rim();
      }
    }
    rail_screw_holes();
    cam_frame() m4_holes();
  }
}

if (part == "collision") bracket_hull(1, rail_w / 2 - 3);
else visual();
