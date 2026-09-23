# Case: ESP32-P4-ETH + PoE module + NEO-7M

OpenSCAD source: `gps-ntp-p4-case.scad`. Outer size 84.6 × 67.0 × 31.7 mm.

- P4-ETH (with the PoE module stacked on top) drops into a cradle along one side; RJ45 out one end, USB-C out the other.
- NEO-7M drops into a cradle beside it, SMA out the USB-C end, ceramic up. Header removed, four wires soldered.
- 8 mm channel between the boards and ~40 mm of empty strip past the GPS pads for wire slack; 4 mm under the P4 for solder tails.
- Lid: four M3 self-tapping screws into the corner posts; posts on the lid hold both boards down.

`gps-ntp-p4-base.stl` is cut for the SMA on the GPS board's edge nearest the P4 (`sma_side = "inner"`, checked against the real module); flip the parameter in the .scad if a different module has it on the other edge.

The lid-screw posts sit in the channel between the two board cradles (not at the case's geometric corners): both boards run nearly the full length/width of the case, so corner posts would land inside a board's footprint. First base print (with corner posts) confirmed exactly that -- 3 of 4 posts blocked the boards from dropping in; fixed by moving all four into the mid-channel.

Untested since the post fix: print the base first and check the RJ45, USB-C and SMA line up, and that both boards drop in cleanly, before printing the lid.
