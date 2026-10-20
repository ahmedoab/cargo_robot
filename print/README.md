# Print files: cargo robot, design v4

Every printed part of the robot, as STL files sorted by material. **34 files, 50 pieces in total.**

- **Units:** the STL files are in millimetres. Print them at 100 %.
- **Orientation:** every file is already turned to the way it should be printed, flat face on the build plate.
- **Supports:** none needed.
- **Quantity:** a file that is printed more than once ends in `_xN`. For example, `guide_rail_x4.stl` means print 4.
- **Printer:** the biggest part is 200 × 150 × 173 mm, so any printer with a 220 × 220 × 250 mm build volume or larger works.

The full list with sizes and weights is in [`../cad/output/print_list.csv`](../cad/output/print_list.csv).

## What goes in which material

| Folder | Pieces | Plastic (approx.) | What they are | Why this material |
|---|---|---|---|---|
| `PETG/` | 30 | 2.6 kg | Body (4 tub tiles), cargo bed (4 tiles), guide rails, load-cell bridge and spacers, caster pods and shims, pusher carriage, blade and idler holder, battery tray | Carries load. Tough, doesn't crack at the bolt holes. |
| `ABS/` | 5 | 0.25 kg | Drive motor mounts and their clamp caps, the stepper motor's rod holder | They touch the motors, which get warm (the stepper runs at 50–70 °C). PLA would soften there. |
| `PLA/` | 10 | 0.2 kg | Rear panel (E-stop, switch, LEDs), front cover, sensor brackets, battery door | No load, no heat. Easiest to print accurately. |
| `TPU/` | 5 | 0.05 kg | Front bumper, battery pads, cable grommets | Has to be soft. |

About **3 kg of filament** in all. The 4 tub tiles are half of it, and also the longest prints.

## Print settings

The settings are a starting point; use your own proven profile for each material.

| | PLA | PETG | ABS | TPU (95A) |
|---|---|---|---|---|
| Nozzle / bed | 205 °C / 60 °C | 235 °C / 80 °C | 245 °C / 100 °C, in an enclosure | 225 °C / 50 °C, slow (25 mm/s) |
| Layer height | 0.2 mm | 0.2 mm (0.3 mm with a 0.6 mm nozzle is fine for the tub tiles) | 0.2 mm | 0.2 mm |
| Walls | 3 | 4 (tub tiles), 5 (pusher parts), 3 (bed tiles) | 5 | 3 |
| Infill | 20 % | 20–40 % gyroid (see each file's note) | 40 % | 15 % |

Each file's own note is in `print_list.csv`, in the "How to print" column.

## Parts that need care

- **Cargo bed tiles (`bed_*.stl`):** they're oriented with the **top face down**, on purpose. The package slides on that face, so print them on a **smooth** plate (glass or smooth PEI with glue stick as a release agent). A textured plate makes the package grip the bed.
- **Tub tiles (`tub_*.stl`):** 173 mm tall with thin walls. Use a brim, and keep the part-cooling fan low for PETG, so the layers bond well.
- **Holes are already modelled at their final size:** M3 = 3.4 mm, M4 = 4.5 mm, 8 mm rods = 8.1 mm, and the LM8UU bearing pockets 15.2 mm. Holes marked for self-tapping screws are 2.8 mm. Don't drill them out unless a part doesn't fit.
- **ABS parts:** if your ABS profile normally needs shrinkage compensation, use it. Check that the gearbox clamp of `motor_mount_*` still takes a 37 mm motor.
- **Two-sided pairs:** `motor_mount_left` / `_right` and `bridge_left` / `_right` are mirror images, not two copies of one part. Print one of each.

## Hardware you need to put it together

All joints use ordinary bolts and nuts; no heat-set inserts. The full list is in [`../cad/output/bom.csv`](../cad/output/bom.csv). The main items:

- M4 × 16 bolts + nuts, about 40
- M4 × 20 and M4 × 30 bolts + nuts, about 12
- An M3 assortment
- One M5 × 25 bolt
