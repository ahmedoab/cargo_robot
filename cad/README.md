# Cargo robot: mechanical design v3 (Phase II)

Design v3 is design v2 plus a rear pusher that slides the cargo off the back of the robot onto a conveyor.
The untouched v2 design is in `../backup_v2`, and `../PUSHER_IMPACT.md` compares the two.

**Overall size:** 434 × 360 × 263 mm (L × W × H), the same as v2.

**Layout:**

- Two drive wheels in the middle, and swivel casters (M8 threaded stems) at the front and rear.
- The bottom plate (6 mm plywood) carries the electronics. The top deck (5 mm acrylic) sits on 6 M6 threaded rods.
- The lidar, E-stop and master switch are at the front of the deck.
- The cargo plate rests on 4 load cells at the rear.
- **Pusher (new in v3):**
  - A blade (aluminium angle) spans the bed, 3 mm above the cargo plate, and rides on two 8 mm rods just outside the deck edges.
  - A T8 lead screw on the right side moves it 262 mm, turned by a third JGB37 gear motor at the front-right corner.
  - Two limit switches mark the ends of the stroke.
  - The usable bed is 257 × 280 mm.

![The pusher fully out](output/views/render_pusher_out.png)

## AutoCAD

`autocad/CargoRobot_v3.dxf` is the design in AutoCAD's own format.

- **Open it:** File → Open → *Files of type: DXF* → the file. Then File → Save As → *AutoCAD 2018 Drawing (*.dwg)*.
- **Model tab:** the full 3D robot, 100 parts on layers `3D-CHASSIS`, `3D-DRIVE`, `3D-ELECTRONICS`, `3D-SENSORS`, `3D-CONTROLS`, `3D-PAYLOAD`, `3D-PUSHER`.
  - Orbit with Shift + middle mouse button, or the ViewCube.
  - Set the visual style (top-left corner of the view) to *Shaded with Edges*.
- **Layout tabs 01–08:** A3 engineering drawings with dimensions, a title block and a bill of materials.
  - 01 general arrangement, 02 layout with the deck removed, 03–05 the three plates with hole tables, 06 printed parts, 07 pusher assembly, 08 BOM.
  - They're set up to plot on *DWG To PDF*, A3, 1:1.
  - Fill in the "DRAWN" box in the title block.
- **True 3D solids** (full AutoCAD, not LT): type `IMPORT` and pick `output/robot_assembly.step`.

`autocad/CargoRobot_v3_drawings.pdf` has the same 8 sheets as a PDF, for printing or showing without AutoCAD.

## Electrical schematics

`autocad/CargoRobot_v3_schematics.dxf` (AutoCAD, 3 layout tabs) and `autocad/CargoRobot_v3_schematics.pdf`:

- **E01 System block diagram:** every part and how power and signals flow between them.
- **E02 Power distribution:** battery → fuse → master switch → E-stop → three motor drivers → motors, the 5.1 V buck converter for the Pi, and the E-stop sense divider. Wire IDs P1–P19 match the power wire list in `BUILD_GUIDE.md` (week 2); P17–P19 are the pusher.
- **E03 Control & sensor wiring:** the Raspberry Pi 40-pin header with every used pin, the motor/encoder cables, the three BTS7960 drivers, the pusher's two limit switches, the 4 HX711 boards with their load cells, and the USB devices. Pin numbers come from `robot_ws/src/cargo_bot/cargo_bot/pins.py`.

## Open it in other CAD programs

- **Fusion 360:** Upload → `output/robot_assembly.step`
- **SolidWorks:** File → Open → change the file type to STEP → `output/robot_assembly.step`
- **Onshape:** Import → `output/robot_assembly.step`
- **FreeCAD:** File → Open → `output/robot_assembly.step`

## Make it

| File | How it's made |
|---|---|
| `output/dxf/bottom_plate.dxf` | Laser-cut 6 mm plywood. The same as v2. |
| `output/dxf/top_deck.dxf` | Laser-cut 5 mm acrylic. Every hole and slot is in the file, except the lidar's 4 small screw holes: drill those using the lidar as a template. New in v3: 5 holes of 4.5 mm for the pusher. A deck already cut from the v2 file only needs those 5 holes drilled by hand. |
| `output/dxf/cargo_plate.dxf` | Laser-cut 5 mm acrylic. The same as v2. |
| `output/parts/camera_mount.stl`, `load_cell_spacer.stl` (× 8) | 3D print, PETG |
| `output/parts/pusher_carriage_L.stl`, `pusher_carriage_R.stl`, `pusher_rail_holder_A.stl` (× 2), `pusher_rail_holder_B.stl`, `pusher_drive_bracket.stl` | 3D print, PETG. A first design that has not been test-printed: print one carriage and check the fits first. |
| `output/parts/pusher_blade.step` | Cut 296 mm from 25 × 25 × 2 mm aluminium angle and drill 4 holes |
| `output/bom.csv` | Everything else (bought parts, rods, nuts) |

## Change it

Edit the parameters at the top of `robot_cad.py`, then run:

```
.venv\Scripts\python robot_cad.py      # regenerate every file
.venv\Scripts\python check_render.py   # collision check (pusher parked, half-way, fully out), wheel-on-floor check, PNG renders
.venv\Scripts\python make_autocad.py   # AutoCAD DXF (3D model + 8 drawing sheets) + PDF
.venv\Scripts\python make_schematics.py # electrical schematics DXF + PDF
.venv\Scripts\python export_ros_mesh.py # robot body mesh for the ROS model (RViz / Gazebo)
```

The pusher's numbers are in the block marked `rear pusher (v3)`: the rod position, the blade size and its gap over the cargo plate, and where the stroke starts and ends.
