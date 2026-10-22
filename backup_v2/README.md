# Backup: design v2 before the tilting bed

An untouched copy of design v2 as it was on 29 September 2026 (commit `11388c4`), before the robot got its
tilting bed and its ESP32 (it still has the Raspberry Pi, the lidar and the USB camera):

- `cad/`: CAD script, collision check, AutoCAD and schematics generators, and all generated files (STEP, STL, DXF,
  renders, `autocad/CargoRobot_v2.dxf`, the drawings PDF and the schematics PDF)
- `BUILD_GUIDE.md`, `SIMULATION.md`, `robot_ws/`, `tools/`: the guide, the simulation notes and the ROS 2 code
  at that point

The current design is in the folders above this one.

To regenerate the backup's files, run its scripts with the main CAD environment, from the project folder:

```
cad\.venv\Scripts\python backup_v2\cad\robot_cad.py
```
