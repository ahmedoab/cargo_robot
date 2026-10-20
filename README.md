# Vision-guided cargo robot + package-sorting conveyor (Graduation Project II)

![The robot](cad/output/views/render_iso.png)

## The system

- **The robot (AGV):**
  - It follows a taped line and avoids obstacles.
  - It weighs its package and slows down when it's heavy.
  - At the conveyor, it stops beside the belt and pushes the package sideways onto it.
  - It is run by one ESP32.
- **The conveyor:**
  - It carries the package under a USB webcam.
  - A laptop runs the AI logo model and tells a second ESP32 which way to send the package.
  - That ESP32 swings a servo lever, so the package leaves at its exit. Unknown logos ride on to a reject bin.
- **No Raspberry Pi:** the laptop does all the vision.

## Folders

| Folder | What's in it |
|---|---|
| [`print/`](print/README.md) | **The files for the 3D printing engineer:** every printed part of the robot as STL, sorted by material (PLA, PETG, ABS, TPU) and already oriented for printing |
| [`cad/`](cad/README.md) | The robot's design (`robot_cad.py`, CadQuery). Outputs: STEP assembly, parts list, print list, pictures. Also covers the wiring and pin map, and the assembly order. |
| `HW & Time plan.pdf` | The original Phase II plan |

## Status (20 October 2026)

- **Robot design v4:** ready to print. It's collision-checked in CAD, but nothing has been printed or test-fitted yet. See [`cad/README.md`](cad/README.md).
- **Next:** the ESP32 firmware for the robot, then the conveyor's ESP32 and the laptop app.
- **Conveyor:** designed later; it isn't being printed yet. [`cad/README.md`](cad/README.md) lists what it must suit: belt height, the 40 mm gap, and clearance for the wheel.
