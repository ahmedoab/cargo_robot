# Backup: design v2 (no pusher)

An untouched copy of design v2 as it was before the rear pusher was added (commit `11388c4`, 29 September 2026):

- `cad/`: CAD script, collision check, AutoCAD and schematics generators, and all generated files (STEP, STL, DXF,
  renders, `autocad/CargoRobot_v2.dxf`, the drawings PDF and the schematics PDF)
- `BUILD_GUIDE.md`, `SIMULATION.md`, `robot_ws/`, `tools/`: the guide, the simulation notes and the robot code
  at that point

The current design (v3, with the pusher) is in the folders above this one. `PUSHER_IMPACT.md` there compares the two.

To regenerate the v2 files, run the scripts in this folder with the main CAD environment, from the project folder:

```
cad\.venv\Scripts\python backup_v2\cad\robot_cad.py
```
