// GPS NTP server case
// Waveshare ESP32-S3-ETH (with its PoE module) on the bottom, a NEO-6M/7M-style GPS board with an
// edge-mount SMA jack stacked above it. Ethernet and USB-C leave one end, the antenna the other.
//
// The ESP32-S3-ETH numbers come from Waveshare's dimension drawing. The GPS numbers are a best
// guess for a 39 x 25.5 mm clone board: MEASURE YOURS and edit the [GPS board] section before
// printing. Open in OpenSCAD and use the Customizer (Window > Customizer).
//
// Assembly
//   1. Drop the S3 board straight down, RJ45 towards its opening. Two M2 x 6 self-tapping screws
//      at the TF-card end; the RJ45 jack rests on the floor pad at the other end.
//   2. Solder the five GPS wires (3V3, GND, GPIO15 PPS, GPIO16, GPIO17) to the S3 top side.
//   3. Lay the GPS board on the rails about 10 mm back from the end wall, slide the SMA thread
//      through its hole, then fit the washer and nut from outside.
//   4. Press the lid on. Its two fins hold the GPS board down on the rails.

/* [Output] */
part = "both"; // [both:Base and lid side by side, base:Base, lid:Lid (print orientation), assembly:Assembled preview with mock boards, section:Cut-away preview]

/* [Print] */
tol = 0.3;        // clearance around boards and the lid lip
wall = 2.0;
floor_t = 2.0;
lid_t = 2.0;
corner_r = 3.0;
$fn = 48;

/* [ESP32-S3-ETH] */
s3_len = 72.8;
s3_w = 21.0;
s3_pcb_t = 1.6;
s3_hole_from_end = 1.58;  // TF-card-end mounting holes: centre to board end
s3_hole_pitch_y = 18.25;  // across the board
s3_screw_d = 1.7;         // pilot for an M2 self-tapping screw
s3_post_d = 4.6;
rj45_w = 16.2;
rj45_h = 13.5;
rj45_protrude = 2.5;      // jack face beyond the board edge
under_clearance = 14.5;   // floor to board underside: the RJ45 and PoE module hang below. CHECK your PoE module is not taller.
usb_open_w = 13.0;        // large enough for a USB-C plug's overmould to enter the wall
usb_open_h = 7.5;
usb_centre_above_pcb = 1.7;
buttons = true;           // paper-clip holes for BOOT (one side) and RESET (the other)
button_from_end = 10.6;   // button centre from the USB end of the board
button_hole_d = 2.2;

/* [GPS board - MEASURE] */
gps_len = 39.0;
gps_w = 25.5;
gps_pcb_t = 1.6;
gps_top_h = 9.0;          // tallest part above the GPS PCB (patch antenna, shield can, battery)
gps_gap = 10.0;           // S3 top surface to GPS underside: wiring room, and anything under the GPS PCB
sma_flange = 2.0;         // GPS PCB edge to the SMA flange face that meets the inside of the wall
sma_y_offset = 0.0;       // SMA centre relative to the GPS board's centreline
sma_z_offset = 0.0;       // SMA centre relative to the middle of the PCB thickness
sma_hole_d = 6.6;         // 1/4-36 thread is 6.35
sma_nut_d = 11.5;         // recess so the nut still leaves thread for the antenna plug
sma_wall_left = 1.2;      // wall thickness left under the nut
gps_slide = 12.0;         // extra rail length so the board can be slid thread-first into the hole

/* [Vents and lid] */
vents = true;
vent_w = 2.0;
vent_pitch = 5.0;
lip_h = 4.0;
lip_t = 1.4;
detent_r = 0.7;

// ---------------------------------------------------------------------------
// Derived. Origin: inside corner at the RJ45 end, on the inner floor. X runs along the case.
s3_x0 = rj45_protrude + 0.3;  // the jack face stops just inside the wall, so the board drops straight in
in_len = s3_x0 + s3_len + 1.0;
in_w = max(s3_w, gps_w) + 2 * tol + 0.2;
cy = in_w / 2;
z_s3_bot = under_clearance;
z_s3_top = z_s3_bot + s3_pcb_t;
z_gps_bot = z_s3_top + gps_gap;
z_gps_top = z_gps_bot + gps_pcb_t;
in_h = z_gps_top + gps_top_h + 1.0;
rail_w = (in_w - gps_w) / 2 + 1.2;  // 1.2 mm of bearing under each long edge of the GPS board
gps_x1 = in_len - sma_flange;       // GPS board end nearest the antenna wall
gps_x0 = gps_x1 - gps_len;
sma_y = cy + sma_y_offset;
sma_z = z_gps_bot + gps_pcb_t / 2 + sma_z_offset;
post_x = s3_x0 + s3_len - s3_hole_from_end;
eps = 0.01;

module rounded_box(x0, y0, z0, lx, ly, lz, r) {
  hull()
    for (x = [x0 + r, x0 + lx - r], y = [y0 + r, y0 + ly - r])
      translate([x, y, z0]) cylinder(r = r, h = lz);
}

module shell() {
  difference() {
    rounded_box(-wall, -wall, -floor_t, in_len + 2 * wall, in_w + 2 * wall, in_h + floor_t, corner_r);
    translate([0, 0, 0]) cube([in_len, in_w, in_h + 1]);
  }
}

module rj45_cut() {
  zb = z_s3_bot - rj45_h;
  translate([-wall - 1, cy - rj45_w / 2 - tol, zb - tol]) cube([wall + 2, rj45_w + 2 * tol, rj45_h + 2 * tol]);
}

module usb_cut() {
  translate([-wall - 1, cy - usb_open_w / 2, z_s3_top + usb_centre_above_pcb - usb_open_h / 2])
    cube([wall + 2, usb_open_w, usb_open_h]);
}

module button_cuts() {
  for (y = [-wall - 1, in_w - 1])
    translate([s3_x0 + button_from_end, y, z_s3_top + 1.0]) rotate([-90, 0, 0]) cylinder(d = button_hole_d, h = wall + 2);
}

module sma_cut() {
  translate([in_len - 1, sma_y, sma_z]) rotate([0, 90, 0]) cylinder(d = sma_hole_d, h = wall + 2);
  translate([in_len + sma_wall_left, sma_y, sma_z]) rotate([0, 90, 0]) cylinder(d = sma_nut_d, h = wall);
}

module vent_cuts() {
  x_from = s3_x0 + 22;  // clear of the RJ45, over the PoE module
  x_to = in_len - 8;    // the high slots sit below the GPS rails, so they can run the full length
  for (x = [x_from : vent_pitch : x_to], y = [-wall - 1, in_w - 1]) {
    translate([x, y, 3]) cube([vent_w, wall + 2, under_clearance - 6]);                       // low: air in, past the PoE module
    translate([x, y, z_s3_top + 2]) cube([vent_w, wall + 2, max(gps_gap - rail_w - 4, 2)]);   // high: air out
  }
}

module detents(grow = 0) {
  for (y = [0, in_w])
    translate([in_len / 2, y, in_h - lip_h / 2]) sphere(r = detent_r + grow);
}

module s3_posts() {
  for (dy = [-s3_hole_pitch_y / 2, s3_hole_pitch_y / 2])
    translate([post_x, cy + dy, 0])
      difference() {
        cylinder(d = s3_post_d, h = z_s3_bot);
        translate([0, 0, z_s3_bot - 8]) cylinder(d = s3_screw_d, h = 8 + eps);
      }
}

module rj45_pad() {
  pad_h = z_s3_bot - rj45_h;
  if (pad_h > 0)
    translate([0, cy - rj45_w / 2, 0]) cube([s3_x0 + 14, rj45_w, pad_h]);
}

// Rails under the GPS board's long edges, chamfered underneath so they print without support
module gps_rails() {
  x0 = max(gps_x0 - gps_slide, 0);
  len = in_len - x0;
  for (side = [0, 1])
    translate([x0, side ? in_w : 0, 0]) mirror([0, side, 0])
      hull() {
        translate([0, 0, z_gps_bot - 0.8]) cube([len, rail_w, 0.8]);
        translate([0, 0, z_gps_bot - 0.8 - rail_w]) cube([len, eps, eps]);
      }
}

module base() {
  difference() {
    union() {
      shell();
      s3_posts();
      rj45_pad();
      gps_rails();
    }
    rj45_cut();
    usb_cut();
    sma_cut();
    if (buttons) button_cuts();
    if (vents) vent_cuts();
    detents(grow = 0.1);
  }
}

// Modelled in place (top of the case), printed flipped
module lid() {
  // Plate
  rounded_box(-wall, -wall, in_h, in_len + 2 * wall, in_w + 2 * wall, lid_t, corner_r);
  // Lip
  difference() {
    translate([tol, tol, in_h - lip_h]) cube([in_len - 2 * tol, in_w - 2 * tol, lip_h + eps]);
    translate([tol + lip_t, tol + lip_t, in_h - lip_h - 1]) cube([in_len - 2 * (tol + lip_t), in_w - 2 * (tol + lip_t), lip_h + 2]);
    // Keep clear of the SMA flange
    translate([in_len - 6, sma_y - 5, in_h - lip_h - 1]) cube([7, 10, lip_h + 2]);
  }
  // Bumps on the lip that click 0.4 mm into the dimples in the side walls
  intersection() {
    detents();
    translate([0, -0.4, in_h - lip_h]) cube([in_len, in_w + 0.8, lip_h]);
  }
  // Fins that hold the GPS board down on its rails, touching only the PCB's long edges
  fin_h = in_h - (z_gps_top + 0.2);
  for (side = [0, 1])
    translate([gps_x0 + 3, side ? in_w - tol - lip_t : tol, z_gps_top + 0.2]) cube([gps_len - 6, lip_t, fin_h + eps]);
}

module mock_boards() {
  color("darkgreen") translate([s3_x0, cy - s3_w / 2, z_s3_bot]) cube([s3_len, s3_w, s3_pcb_t]);
  color("silver") translate([s3_x0 - rj45_protrude, cy - rj45_w / 2, z_s3_bot - rj45_h]) cube([21, rj45_w, rj45_h]);
  color("dimgray") translate([s3_x0 + 22, cy - 9, z_s3_bot - 12]) cube([30, 18, 12]);  // PoE module, approximate
  color("navy") translate([gps_x0, cy - gps_w / 2, z_gps_bot]) cube([gps_len, gps_w, gps_pcb_t]);
  color("tan") translate([gps_x0 + 7, cy - 12.5, z_gps_top]) cube([25, 25, gps_top_h - 2]);
  color("gold") translate([gps_x1, sma_y, sma_z]) rotate([0, 90, 0]) cylinder(d = 6.35, h = sma_flange + 9.5);
}

// Cut-away along the centreline, to check clearances
module section_view() {
  difference() {
    union() { base(); lid(); }
    translate([-50, -50, -50]) cube([300, 50 + cy, 200]);
  }
  mock_boards();
}

if (part == "section") section_view();
else if (part == "base") base();
else if (part == "lid") translate([0, 0, in_h + lid_t]) rotate([180, 0, 0]) translate([0, -in_w, 0]) lid();
else if (part == "assembly") { base(); mock_boards(); %lid(); }
else {
  base();
  translate([0, in_w + 2 * wall + 8, in_h + lid_t - floor_t]) rotate([180, 0, 0]) translate([0, -in_w, 0]) lid();
}

echo(str("Outer size: ", in_len + 2 * wall, " x ", in_w + 2 * wall, " x ", in_h + floor_t + lid_t, " mm"));
