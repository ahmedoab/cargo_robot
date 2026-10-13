# Cargo robot: mechanical design v2 (Phase II)

**Overall size:** 434 × 360 × 263 mm (L × W × H).

**Layout:**

- Two drive wheels in the middle, and swivel casters (M8 threaded stems) at the front and rear.
- The bottom plate (6 mm plywood) carries the electronics. The top deck (5 mm acrylic) sits on 6 M6 threaded rods.
- The lidar, E-stop and master switch are at the front of the deck.
- The cargo plate rests on 4 load cells at the rear.

## AutoCAD

`autocad/CargoRobot_v2.dxf` is the design in AutoCAD's own format.

- **Open it:** File → Open → *Files of type: DXF* → the file. Then File → Save As → *AutoCAD 2018 Drawing (*.dwg)*.
- **Model tab:** the full 3D robot, 77 parts on layers `3D-CHASSIS`, `3D-DRIVE`, `3D-ELECTRONICS`, `3D-SENSORS`, `3D-CONTROLS`, `3D-PAYLOAD`.
  - Orbit with Shift + middle mouse button, or the ViewCube.
  - Set the visual style (top-left corner of the view) to *Shaded with Edges*.
- **Layout tabs 01–07:** A3 engineering drawings with dimensions, a title block and a bill of materials.
  - 01 general arrangement, 02 layout with the deck removed, 03–05 the three plates with hole tables, 06 printed parts, 07 BOM.
  - They're set up to plot on *DWG To PDF*, A3, 1:1.
  - Fill in the "DRAWN" box in the title block.
- **True 3D solids** (full AutoCAD, not LT): type `IMPORT` and pick `output/robot_assembly.step`.

`autocad/CargoRobot_v2_drawings.pdf` has the same 7 sheets as a PDF, for printing or showing without AutoCAD.

## Electrical schematics

`autocad/CargoRobot_v2_schematics.dxf` (AutoCAD, 3 layout tabs) and `autocad/CargoRobot_v2_schematics.pdf`:

- **E01 System block diagram:** every part and how power and signals flow between them.
- **E02 Power distribution:** battery → fuse → master switch → E-stop → motor drivers → motors, the 5.1 V buck converter for the Pi, and the E-stop sense divider. Wire IDs P1–P16 match the power wire list in `BUILD_GUIDE.md` (week 2).
- **E03 Control & sensor wiring:** the Raspberry Pi 40-pin header with every used pin, the motor/encoder cables, both BTS7960 drivers, the 4 HX711 boards with their load cells, and the USB devices. Pin numbers come from `robot_ws/src/cargo_bot/cargo_bot/pins.py`.

## Open it in other CAD programs

- **Fusion 360:** Upload → `output/robot_assembly.step`
- **SolidWorks:** File → Open → change the file type to STEP → `output/robot_assembly.step`
- **Onshape:** Import → `output/robot_assembly.step`
- **FreeCAD:** File → Open → `output/robot_assembly.step`

## Make it

| File | How it's made |
|---|---|
| `output/dxf/bottom_plate.dxf` | Laser-cut 6 mm plywood |
| `output/dxf/top_deck.dxf`, `cargo_plate.dxf` | Laser-cut 5 mm acrylic. Every hole and slot is in the file, except the lidar's 4 small screw holes: drill those using the lidar as a template. |
| `output/parts/camera_mount.stl`, `load_cell_spacer.stl` (× 8) | 3D print, PETG |
| `output/bom.csv` | Everything else (bought parts, rods, nuts) |

## Change it

Edit the parameters at the top of `robot_cad.py`, then run:

```
.venv\Scripts\python robot_cad.py      # regenerate every file
.venv\Scripts\python check_render.py   # collision check, wheel-on-floor check, PNG renders
.venv\Scripts\python make_autocad.py   # AutoCAD DXF (3D model + 7 drawing sheets) + PDF
.venv\Scripts\python make_schematics.py # electrical schematics DXF + PDF
.venv\Scripts\python export_ros_mesh.py # robot body mesh for the ROS model (RViz / Gazebo)
```
