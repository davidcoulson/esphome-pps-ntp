// Case for the GPS-disciplined NTP server: Waveshare ESP32-P4-ETH with its PoE module stacked on
// top, plus a u-blox NEO-7M module whose SMA connector passes through the side wall (the antenna
// lives in the attic on a KMR240 extension). Both boards drop into cradles: no screws into either.
//
//   part = "base" | "lid" | "assembly"
//
// All board numbers are measured or from Waveshare's dimension drawing.

part = "assembly";
// The SMA sits 2.35 mm from one long edge of the GPS board. With the module in its cradle (SMA
// towards the USB-C end, ceramic up), is the connector on the edge nearest the P4 ("inner") or the
// edge nearest the case wall ("outer")? Both STLs are exported; print the one that matches.
sma_side = "outer";

/* ---------------- boards (measured) ---------------- */
p4_l = 78.0;    p4_w = 21.0;    p4_pcb = 1.6;
rj45_protrude = 0.31;   // shell past the board end
usbc_protrude = 1.50;
rj45_h = 13.5;          // jack height above the board
usbc_h = 3.3;           // connector height above the board
poe_l = 36.55;  poe_w = 21.0;
poe_x0 = 21.02 - rj45_protrude;  // PoE nearest edge from the P4's RJ45 end (20.71)
stack_h = 22.74;        // P4 underside to the top of the PoE module

gps_l = 39.08;  gps_w = 25.15;  gps_pcb = 1.6;
gps_tall = 5.64;        // ceramic patch above the board
sma_z = 7.68;           // SMA centre above the bottom of the board
sma_edge = 2.35;        // SMA centre in from the nearest long edge
sma_hole_d = 7.0;

/* ---------------- case ---------------- */
wall = 2.0;  floor_t = 2.0;  lid_t = 2.0;
clear = 0.4;            // fit clearance around boards
tail_h = 4.0;           // under the P4: solder tails and wire soldering room
gps_lift = 3.0;         // GPS board off the floor: its own solder tails
post_d = 6.0;  screw_d = 2.8;  head_d = 6.2;   // M3 self-tapping lid screws
rib = 1.6;              // side ribs that locate the boards
gap = 8.0;              // between the P4 strip and the GPS strip: the four wires and their slack

in_l = p4_l + rj45_protrude + usbc_protrude + 2 * clear;        // P4 spans the full length
in_w = 2 * clear + p4_w + gap + gps_w + 2 * clear + 2 * rib;
in_h = tail_h + stack_h + 1.0;                                    // 1 mm over the PoE
out_l = in_l + 2 * wall;  out_w = in_w + 2 * wall;  out_h = floor_t + in_h + lid_t;

p4_x = wall + clear + rj45_protrude;   // board's RJ45 end
p4_y = wall + rib + clear;
p4_z = floor_t + tail_h;
gps_x = wall + in_l - clear - gps_l;   // against the +X (USB-C) wall, SMA through that wall;
                                       // ~40 mm of empty strip beyond the pads for wire slack
gps_y = p4_y + p4_w + clear + gap + rib + clear;
gps_z = floor_t + gps_lift;
sma_y = sma_side == "outer" ? gps_y + gps_w - sma_edge : gps_y + sma_edge;
post_xy = [[wall + 4, wall + 4], [out_l - wall - 4, wall + 4], [wall + 4, out_w - wall - 4], [out_l - wall - 4, out_w - wall - 4]];

module rrect(l, w, r) offset(r = r) offset(delta = -r) square([l, w]);

module base() {
    difference() {
        union() {
            difference() {
                linear_extrude(floor_t + in_h) rrect(out_l, out_w, 2.5);
                translate([wall, wall, floor_t]) linear_extrude(in_h + 1) rrect(in_l, in_w, 1);
            }
            // P4 rests on four pads at its corners, between the tails; ribs along both long edges
            for (x = [p4_x + 2, p4_x + p4_l - 8], y = [p4_y, p4_y + p4_w - 3])
                translate([x, y, floor_t]) cube([6, 3, tail_h]);
            for (y = [p4_y - clear - rib, p4_y + p4_w + clear])
                translate([wall, y, floor_t]) cube([in_l, rib, tail_h + p4_pcb + 2]);
            // GPS cradle: ledges under both long edges, ribs outside them
            for (dy = [0, gps_w - 2])
                translate([gps_x, gps_y + dy, floor_t]) cube([gps_l, 2, gps_lift]);
            for (y = [gps_y - clear - rib, gps_y + gps_w + clear])
                translate([gps_x - 2, y, floor_t]) cube([gps_l + 4, rib, gps_lift + gps_pcb + 2]);
            translate([gps_x - clear - rib, gps_y - 2, floor_t]) cube([rib, gps_w + 4, gps_lift + gps_pcb + 2]);
            for (p = post_xy) translate([p[0], p[1], floor_t]) cylinder(d = post_d, h = in_h, $fn = 28);
        }
        for (p = post_xy) translate([p[0], p[1], floor_t + 3]) cylinder(d = screw_d, h = in_h, $fn = 24);
        // RJ45 through the -X wall; USB-C through the +X wall
        translate([-1, p4_y - 0.5, p4_z + p4_pcb - 0.5]) cube([wall + 2, p4_w + 1, rj45_h + 1.5]);
        // sized for a USB-C plug's overmould, not just the socket
        translate([out_l - wall - 1, p4_y + p4_w / 2 - 6, p4_z + p4_pcb - 1.5]) cube([wall + 2, 12, usbc_h + 3.5]);
        // SMA through the +X wall, centred on the connector
        translate([out_l - wall - 1, sma_y, gps_z + sma_z]) rotate([0, 90, 0]) cylinder(d = sma_hole_d, h = wall + 3, $fn = 36);
        // floor vents under the P4
        for (x = [p4_x + 10 : 6 : p4_x + p4_l - 12]) translate([x, p4_y + 3, -1]) cube([3, p4_w - 6, floor_t + 2]);
    }
}

module lid() {
    difference() {
        union() {
            linear_extrude(lid_t) rrect(out_l, out_w, 2.5);
            translate([wall + 0.25, wall + 0.25, -1.5]) linear_extrude(1.5) rrect(in_l - 0.5, in_w - 0.5, 1);
            // posts that hold the boards down: one on the P4 (clear of the PoE), two on the GPS
            hold_p4 = in_h - (tail_h + p4_pcb);
            translate([p4_x + p4_l - 14, p4_y + p4_w / 2 - 2, -hold_p4]) cube([6, 4, hold_p4 + 0.1]);
            hold_gps = in_h - (gps_lift + gps_pcb);
            for (x = [gps_x + 3, gps_x + gps_l - 7])
                translate([x, gps_y + 1, -hold_gps]) cube([4, 3, hold_gps + 0.1]);
        }
        for (p = post_xy) {
            translate([p[0], p[1], -3]) cylinder(d = 3.4, h = lid_t + 6, $fn = 24);
            translate([p[0], p[1], lid_t - 1.5]) cylinder(d1 = 3.4, d2 = head_d, h = 1.6, $fn = 24);
        }
        for (x = [gps_x + 6 : 6 : gps_x + gps_l - 8]) translate([x, gps_y + 5, -3]) cube([3, gps_w - 10, lid_t + 6]);
        for (x = [p4_x + 8 : 6 : p4_x + p4_l - 22]) translate([x, p4_y + 3, -3]) cube([3, p4_w - 6, lid_t + 6]);
    }
}

echo(str("outer ", out_l, " x ", out_w, " x ", out_h, " mm"));

if (part == "base") base();
else if (part == "lid") translate([0, 0, lid_t]) rotate([180, 0, 0]) lid();
else {
    base();
    color("orange", 0.55) translate([0, 0, floor_t + in_h]) lid();
    %translate([p4_x, p4_y, p4_z]) cube([p4_l, p4_w, p4_pcb]);
    %translate([p4_x + poe_x0, p4_y, p4_z + stack_h - 8]) cube([poe_l, poe_w, 8]);
    %translate([p4_x - rj45_protrude, p4_y + 2, p4_z + p4_pcb]) cube([16, p4_w - 4, rj45_h]);
    %translate([gps_x, gps_y, gps_z]) cube([gps_l, gps_w, gps_pcb]);
    %translate([gps_x + 6, gps_y + 3, gps_z + gps_pcb]) cube([25, 18, gps_tall]);
    %translate([gps_x + gps_l - 4, sma_y, gps_z + sma_z]) rotate([0, 90, 0]) cylinder(d = 6.3, h = wall + 10, $fn = 24);
}
