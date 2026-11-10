# Cargo Robot: Complete Build Guide (Phase II)

**From Monday 28 September 2026 to the final defense (week of 14 December 2026).**

This guide covers the whole build: parts, tools, fabrication, assembly, every wire, the ESP32 firmware, calibration, line following, the tilting bed, testing and the final report. Follow it top to bottom.

![The robot](cad/output/views/render_iso.png)

---

## Contents

- [0. How to use this guide](#0-how-to-use-this-guide)
- [1. The robot on one page](#1-the-robot-on-one-page)
- [2. Timeline](#2-timeline)
- [3. Safety rules](#3-safety-rules-read-before-you-touch-anything)
- [Week 0: Decisions, shopping, laptop setup (28 Sep – 4 Oct)](#week-0--decisions-shopping-laptop-setup-28-sep--4-oct)
- [Week 1: Fabrication & mechanical assembly (5 – 11 Oct)](#week-1--fabrication--mechanical-assembly-5--11-oct)
- [Week 2: Power & safety wiring (12 – 18 Oct)](#week-2--power--safety-wiring-12--18-oct)
- [Week 3: ESP32 setup, signal wiring, bench tests (19 – 25 Oct)](#week-3--esp32-setup-signal-wiring-bench-tests-19--25-oct)
- [Week 4: The firmware and the tilting bed (26 Oct – 1 Nov)](#week-4--the-firmware-and-the-tilting-bed-26-oct--1-nov)
- [Week 5: Driving: motor control, PID, odometry (2 – 8 Nov)](#week-5--driving-motor-control-pid-odometry-2--8-nov)
- [Week 6: Line following and station marks (9 – 15 Nov)](#week-6--line-following-and-station-marks-9--15-nov)
- [Week 7: Obstacles (16 – 22 Nov)](#week-7--obstacles-16--22-nov)
- [Week 8: Delivery missions (23 – 29 Nov)](#week-8--delivery-missions-23--29-nov)
- [Week 9: Field testing & metrics (30 Nov – 6 Dec)](#week-9--field-testing--metrics-30-nov--6-dec)
- [Week 10: Cable harness, covers, final checks (7 – 13 Dec)](#week-10--cable-harness-covers-final-checks-7--13-dec)
- [Week 11: Report, video, presentation, demo (14 – 20 Dec)](#week-11--report-video-presentation-demo-14--20-dec)
- [Troubleshooting](#troubleshooting)
- [Appendix A: Skills crash course](#appendix-a--skills-crash-course)
- [Appendix B: Full source code](#appendix-b--full-source-code)

---

## 0. How to use this guide

- **Each week has:** a goal, numbered steps with checkboxes `- [ ]`, and a **"Done when"** test. Don't start the next week until the "Done when" test passes (or you know exactly why it doesn't).
- **Where things happen:**
  - **[LAPTOP]**: in the Arduino IDE on your laptop (Windows, macOS or Linux), with the ESP32 plugged in by USB.
  - **[SERIAL]**: a command typed into the Arduino IDE's **Serial Monitor** (115200 baud, line ending "Newline"). The robot answers there.
- **You don't need to type any code.** All of it is already in the project folder (see [Appendix B](#appendix-b--full-source-code)); this guide explains it and shows how to use it.
- **New to a skill?** If a step needs something you've never done (soldering, crimping, using a multimeter, git), read [Appendix A](#appendix-a--skills-crash-course) first.

### The project folder

```
Gradfinal/                      <- this folder. Put it on GitHub
├── BUILD_GUIDE.md              <- this guide
├── cad/                        <- 3D design (Python/CadQuery) -> STEP, STL, DXF files
│   ├── robot_cad.py            <- the design; change numbers at the top, re-run
│   ├── wiring.py               <- every wire (P1-P19, S1-S28), in one place
│   ├── autocad/                <- the PDFs (and AutoCAD files):
│   │     CargoRobot_v2_drawings.pdf     10 drawing sheets (+ CargoRobot_v2.dxf with the 3D model)
│   │     CargoRobot_v2_schematics.pdf   4 schematic sheets (+ .dxf)
│   │     CargoRobot_v2_assembly.pdf     list of components + assembly step by step, with pictures
│   │     CargoRobot_v2_connections.pdf  every connection: wire lists, each part's pins, the schematics
│   └── output/                 <- robot_assembly.step, parts/*.stl, dxf/*.dxf, views/*.png, bom.csv
├── firmware/cargo_robot/       <- the ESP32 program (Arduino IDE)
│   ├── cargo_robot.ino         <- the mission and the safety rules
│   ├── config.h                <- the wiring: every pin number (single source of truth)
│   ├── settings.h / .cpp       <- tunable numbers, kept in the ESP32's flash
│   ├── robot.h / .cpp          <- the robot's parts and live readings, wheel control, bed motion
│   ├── hw.h / .cpp             <- drivers: motors, encoders, line sensors and panel pins, ultrasonic, servos
│   ├── control.h / .cpp        <- maths: wheel PID, line position, obstacle speed factor
│   └── console.h / .cpp        <- Serial-Monitor commands: bench tests, calibration, settings
└── backup_v2/                  <- design v2 as it was before the tilting bed (Raspberry Pi version)
```

### Words you will meet

| Word | Meaning |
|---|---|
| **ESP32** | The robot's computer: a small board with a dual-core processor, Wi-Fi and lots of pins. It runs one program (the firmware) from power-on. |
| **Firmware** | The program on the ESP32 (`firmware/cargo_robot/`). You upload it from the Arduino IDE. |
| **Arduino IDE** | The program on your laptop that compiles and uploads the firmware, and has the Serial Monitor. |
| **Serial Monitor** | A text window in the Arduino IDE: the robot prints there and you type commands to it. |
| **GPIO** | A pin of the ESP32; each one can be a 3.3 V digital input or output. |
| **PWM** | Switching a pin on and off very fast; the on-percentage ("duty") sets motor power. |
| **Encoder** | A sensor on the motor that sends pulses as it turns. It gives the wheel speed and distance. |
| **Servo** | A motor with a gearbox and an angle sensor inside: you send it an angle (as a pulse 500–2500 µs long, 50 times per second) and it turns there. Two of them tip the bed. |
| **IR line sensor** | A small board that shines infrared light at the floor and sees how much comes back: black tape reflects almost none. Its OUT pin is HIGH over black, LOW over the floor; a blue screw sets where the switch happens. |
| **Station mark** | A short strip of black tape across the line: all 3 line sensors see black at once. The robot stops at it: one at the pickup, one at the drop-off (the conveyor). |

---

## 1. The robot on one page

| | |
|---|---|
| Size | 407 × 382 × 218 mm (L × W × H), bed flat |
| Frame | 6 mm plywood bottom plate + 5 mm acrylic deck, held apart by 6 M6 threaded rods |
| Drive | 2 × JGB37-520 12 V 110 RPM gear motors with encoders and 65 mm wheels (middle axle), plus a printed ball caster front and rear. Differential drive, so it turns on the spot. |
| Brain | **One ESP32** (DevKitC, 38 pins), running an Arduino program. Everything is wired straight to it: no I/O expander. No Raspberry Pi, no Linux, no ROS. |
| Sensors | 3 IR line sensors (on a printed bar under the bottom plate, in front of the wheels), HC-SR04 ultrasonic (front, looking ahead). Nothing weighs the package. |
| Tilting bed | A plywood tilting plate carries the cargo plate on 4 printed risers. **Two 60 kg·cm servos** on the deck's right edge turn it about their shafts (the shafts are the hinge) to 40°; the package slides off to the right onto the conveyor. |
| Power | 3S LiPo 11.1 V 5000 mAh → 10 A fuse → master switch. The 12 V goes through the E-stop to the motor drivers and the 7.4 V servo buck; a 5.1 V buck powers the ESP32 and the sensors. |
| Safety | Latching E-stop cuts **motor and servo power** (the ESP32 stays on and sees it). The software also stops for obstacles and a lost line. |
| Packages | Up to **3 kg**, up to 20 × 20 × 20 cm. The robot can't weigh them, so this limit is a rule for the people loading it. |
| Speed | 0.2 m/s following the line (adjustable) |

```mermaid
flowchart LR
  BAT[3S LiPo 11.1 V] --> FUSE[10 A fuse] --> SW[Master switch] --> BUS12[12 V main bus]
  BUS12 --> BUCK[Buck 12→5.1 V] --> ESP[ESP32]
  BUS12 --> ESTOP[E-stop NC] --> MBUS[12 V motor bus] --> DRV[2 × BTS7960] --> MOT[2 × gear motor + encoder]
  MBUS --> SBUCK[Buck 12→7.4 V] --> SERVO[2 × bed servo]
  MBUS -. divider .-> ESP
  ESP -- PWM --> DRV
  ESP -- 2 × servo signal --> SERVO
  MOT -- encoder A/B --> ESP
  SONAR[HC-SR04 ultrasonic] --> ESP
  LINE[3 × IR line sensor] --> ESP
  ESP --> PANEL[LEDs, buzzer, motor enable]
  BTN[start button] --> ESP
```

**The whole system** has two more parts that are **not in this guide** (they are a separate project): the conveyor belt with its own ESP32, and a laptop with a camera that reads the logo on each package and tells the conveyor where to send it. The robot's only job at the conveyor is to stop beside it and tip the package onto the belt.

### What changed compared to the Phase II plan (tell your supervisor)

| Phase II plan | This guide | Why |
|---|---|---|
| Raspberry Pi **+ ESP32** | **ESP32 only** | The robot no longer runs the AI or maps rooms: it follows a line, avoids obstacles and tips the bed. One ESP32 does all of that in real time, starts in 1 s and can't corrupt an SD card. |
| RPLIDAR, SLAM (task 2.2) and Nav2 waypoints (task 2.4) | **No lidar.** An IR line sensor and station marks on the floor | A taped route is simpler and very repeatable. Task 2.2 becomes "line and station-mark tuning", task 2.4 becomes "pickup → drop-off missions". |
| Camera + ONNX vision on the robot (task 2.3.2) | **The laptop does the vision, at the conveyor** | The camera has to look at the packages on the belt anyway; the laptop has far more computing power than the robot. |
| Obstacle detection by lidar | **One ultrasonic sensor** looking ahead | Cheap, simple, and enough to stop for a person or a box on the line. |
| Unload by hand | **Tilting bed on 2 servos** | The robot delivers by itself: it tips the package onto the conveyor. |
| E-stop cuts the whole 12 V line | E-stop cuts the **motor and servo** 12 V only | The ESP32 stays on, sees the E-stop and does not lose its place. Everything that moves still stops at once. |
| 2020 aluminium or acrylic/MDF frame | Plywood + acrylic plates on **6 M6 threaded rods** | Cheapest and simplest: the rod is cut with a hacksaw, and the deck height is set with nuts. |
| Heavy-duty omni-wheel casters | **2 printed ball casters** (a 25 mm steel ball in a printed cup, on an M8 bolt) | Nothing swings round when the robot turns on the spot; the height is set with 2 nuts, so all 4 wheels can be made to touch the floor. |
| 12 V 300–500 RPM motors | **JGB37-520 12 V 110 RPM set** (motor, bracket, coupling, 65 mm wheel) | Bought as one set. 110 RPM with 65 mm wheels gives about 0.3 m/s top speed, plenty for 0.2 m/s line following, and 3.9 kg·cm rated torque per motor: about 4× what a 3 kg package needs. |
| IR line sensor array | **3 single IR modules** (3-pin obstacle-avoidance type) on a printed bar | Easy to find; 3 sensors are enough to follow the line, see it lost and see the station marks. |
| I/O expander (MCP23017), side distance sensor (VL53L0X) | **Neither** | With 3 line sensors everything fits on the ESP32's own pins; the drop-off check by the side sensor is dropped. Two parts fewer, and no I2C. |
| Load cells weigh the package (speed by weight, overload check) | **No weighing** | Fewer parts: no load cells, no HX711 boards, no calibration, and 4 printed risers instead. The robot drives at one speed; the 3 kg limit is a rule for the people loading it. |

---

## 2. Timeline

| Week | Dates (2026) | Goal | Done when |
|---|---|---|---|
| 0 | 28 Sep – 4 Oct | Approvals, **order everything**, laptop + GitHub ready | Parts ordered, DXF files at the laser shop, Arduino IDE uploads to an ESP32 |
| 1 | 5 – 11 Oct | Cut, print and assemble the mechanics, including the tilting bed | Rolling chassis with all 4 wheels on the floor; the bed swings freely by hand |
| 2 | 12 – 18 Oct | Power & safety wiring + bench tests | 12 V / 5.1 V / 7.4 V measured, E-stop kills motor and servo power, ESP32 runs from the battery |
| 3 | 19 – 25 Oct | Firmware uploaded, signal wiring, bench tests | `enc`, `motortest`, `line`, `sonar` all pass |
| 4 | 26 Oct – 1 Nov | Servo calibration, tipping tests | The bed tips a 3 kg box off 10 times out of 10 |
| 5 | 2 – 8 Nov | Driving: PID, odometry calibration | Robot drives 3 m straight and ends within 5 cm |
| 6 | 9 – 15 Nov | Line following, station marks | Follows the track for 10 laps and stops on every mark |
| 7 | 16 – 22 Nov | Obstacles | Robot slows for something ahead, stops for it, and drives on once it's gone |
| 8 | 23 – 29 Nov | Delivery mission | Pickup → drop-off (tip) → pickup works 5 times in a row |
| 9 | 30 Nov – 6 Dec | Field tests & measurements | All test tables filled in |
| 10 | 7 – 13 Dec | Cable harness, covers, final checks | Clean, labelled wiring; covers on |
| 11 | 14 – 20 Dec | Report, video, slides, demo rehearsal | Report submitted, video recorded, 2 full rehearsals done |

> **Parts can be late.** If they are, install the Arduino IDE and upload the firmware to a bare ESP32 early (section 3.1): the Serial Monitor commands work without anything connected, and you learn them before the robot exists.

**Suggested split for a team of 3–4:**

- **Mechanical lead:** weeks 1, 4 and 10.
- **Electrical lead:** weeks 2 and 3 (wiring).
- **Software lead:** weeks 3–8.
- **Everyone:** weeks 9 and 11. One person owns the report from week 5 onwards and writes each week's section as it happens.

---

## 3. Safety rules (read before you touch anything)

1. **LiPo batteries can catch fire.**
   - Charge only with a LiPo balance charger, in a LiPo bag, never unattended, never on a bed or carpet.
   - Never let a cell drop below 3.3 V (the voltage alarm beeps at 3.5 V: stop and charge).
   - A puffy (swollen) battery is dead: stop using it.
   - Store it at 3.8 V per cell (the charger's "storage" mode).
2. **Wheels in the air.** When testing motors, put the robot on a box so the drive wheels spin freely.
3. **Keep fingers out of the bed.** The servos are strong (60 kg·cm each) and the bed's right edge closes against the servo mounts. Never hold the bed or put a hand under it while it moves.
4. **Know where the E-stop is.** It stays within reach every time the robot moves by itself, and one person always watches the robot.
5. **Never put more than 3.3 V on an ESP32 pin.** 5 V on a GPIO pin kills the ESP32. That's why the encoders, line sensor and BTS7960 logic are powered from **3.3 V**, the ultrasonic ECHO line goes through a divider, and so does the 12 V sense line.
6. **Wiring changes only with the master switch OFF** and the battery unplugged.
7. **Fuse first:** the fuse must be the first thing after the battery's + wire.

---

## Week 0 — Decisions, shopping, laptop setup (28 Sep – 4 Oct)

**Goal:** by Sunday everything is ordered, the plates are at the laser shop, and your laptop uploads programs to an ESP32.

### 0.1 Get approval from your supervisor (Monday)

Show the drawings, either as `cad/autocad/CargoRobot_v2_drawings.pdf` or the same file opened in AutoCAD, the schematics `cad/autocad/CargoRobot_v2_schematics.pdf`, and the component list in `cad/autocad/CargoRobot_v2_assembly.pdf`. Also show the table "What changed compared to the Phase II plan" in section 1. Then ask:

- [ ] Is it OK to drop the Raspberry Pi and the lidar, and run the robot on one ESP32 with line following?
- [ ] Is it OK that the AI vision runs on a laptop at the conveyor, not on the robot?
- [ ] Is it OK that the E-stop cuts motor and servo power only (not the ESP32)?
- [ ] Is it OK to use 110 RPM motors with 65 mm wheels instead of 300–500 RPM motors?
- [ ] Is it OK to use printed ball casters instead of omni-wheel casters, and 3 IR line sensors instead of a sensor array?
- [ ] What is the heaviest package you must demonstrate? (This design tips up to 3 kg. The robot can't weigh packages, so everyone who loads it must know the limit.)
- [ ] Where is the test area, and can you put tape lines and station marks on its floor?

### 0.2 Shopping list

Buy locally what you can; online orders from abroad can take 2–4 weeks. **Order the servos and motors first.** Quantities include a few spares where parts are cheap and easy to break. `cad/output/bom.csv` has the same list in short form.

**Electronics**

| # | Item | Qty | What to check when buying |
|---|---|---|---|
| E1 | ESP32 DevKitC, 38 pins (ESP32-WROOM-32, USB) | 2 | One spare. Get the **38-pin** board, not the 30-pin one: the pin numbers in this guide are for it. |
| E2 | 38-pin ESP32 screw-terminal breakout board | 1 | Wires screw in instead of loose jumpers |
| E4 | **JGB37-520 12 V 110 RPM gear motor set**: motor with Hall encoder (6 wires) + steel L-bracket + hex coupling + **65 mm wheel** | 2 | One set per side. The shaft is 6 mm with a flat (sellers often write "5.5 mm D-shaft": that's the size across the flat); the coupling must fit it. The encoder cable has 2 motor wires + VCC, GND, A, B. |
| E6 | **Steel bearing ball, 25.4 mm (1 inch)**, or a 25 mm glass marble | 3 | For the 2 printed ball casters (1 spare). Steel rolls best; a marble works. |
| E6b | M8 × 50 bolt + 2 M8 nuts + 2 M8 washers | 2 sets | One per ball caster: the bolt's head sits in the cup, the nuts set the height |
| E7 | BTS7960 43 A motor driver module ("IBT-2") | 2 | |
| E8 | **Servo DS5160**, 60 kg·cm, metal gears, digital, 180°, 6–8.4 V, 65 × 30 × 48 mm | 2 | Both the same model. The printed mounts are made for its outline (maker's drawing). Note its spline (25T on most) for E9. |
| E9 | Aluminium **disc** servo horn for the servo's spline, **38 mm or smaller** | 2 | Not the 4-arm horn that often comes with the servo: its arms hit the deck. Any screw circle from 19 to 28 mm fits the printed adapter. |
| E10 | Buck converter **XL4016 8 A**, with voltage pot | 1 | For the servos: set to 7.4 V. It needs its heat sink. |
| E11 | XL4015 5 A buck converter module (with current-limit pot) | 2 | 1 spare. For the ESP32 and sensors: set to 5.1 V |
| E12 | **IR obstacle-avoidance module, 3 pins** (VCC, GND, OUT), one blue pot ("FC-51" / "HW-201" type) | 4 | 3 + 1 spare: the line sensors. **Not** the 4-pin KY-032 (made for 2–40 cm, unreliable this close to the floor). |
| E13 | HC-SR04 ultrasonic sensor | 2 | 1 spare |
| E17 | 3S LiPo 11.1 V 5000 mAh, ≥ 20 C, XT60 plug | 1 (2 is better) | A second battery saves your demo day |
| E18 | LiPo balance charger (IMAX B6 / ISDT / SkyRC class) + power supply | 1 | |
| E19 | LiPo fire-safe bag | 1 | |
| E20 | LiPo voltage alarm (1–8 S buzzer, plugs into the balance lead) | 1 | |
| E21 | Inline blade-fuse holder (14 AWG) + 10 A blade fuses | 1 + 5 | |
| E22 | Master switch: panel toggle switch, **12 V DC ≥ 15 A**, 12 mm mounting hole | 1 | The DC rating matters, not the AC rating |
| E23 | Emergency-stop mushroom button, **22 mm panel-mount**, latching, with an **NC contact block** (10 A) | 1 | "NC" = normally closed: power flows until pressed |
| E24 | Push button, 12 mm panel-mount, momentary, normally open | 1 | The start button |
| E25 | 5 mm LEDs green + red, with 5 mm panel clips (6 mm hole) | 1 each | |
| E26 | Active buzzer module, 3 pins, 3.3–5 V | 1 | |
| E27 | XT60 connector pairs | 4 | |
| E28 | WAGO 221 lever connectors: 221-415 (5-way) × 8, 221-413 (3-way) × 6 | 14 | Makes power buses without soldering |
| E29 | Resistors (¼ W): 1 kΩ × 3, 2 kΩ × 3, 10 kΩ × 5, 2.2 kΩ × 1, 330 Ω × 2; capacitor 100 nF × 1 | 1 set | Dividers and pull resistors |
| E30 | Small perfboard (5 × 7 cm) | 2 | The I/O board |
| E31 | Silicone wire 16 AWG red + black | 3 m each | Battery, fuse, switch, E-stop, drivers, servo supply |
| E32 | Silicone wire 20 AWG red + black | 2 m each | Bucks, motors |
| E33 | Jumper wires female-female and male-female, 20 cm | 40 each | Signals |
| E35 | Servo extension leads, 30 cm | 2 | Servo → I/O board |
| E36 | Ferrule kit + ferrule crimper | 1 | For every wire going into a screw terminal |
| E37 | Heat-shrink assortment, zip ties, spiral wrap, 20 mm velcro strap (1 m), double-sided foam tape | 1 | |
| E38 | VHB (strong double-sided foam) tape, 19 mm | 1 roll | Holds the small boards without drilling |
| E39 | USB cable (USB-A or USB-C to micro-USB, matching your ESP32), ≥ 1 m, **data** cable | 1 | Some cheap cables only charge: try it before the build |

**Mechanical**

| # | Item | Qty | Notes |
|---|---|---|---|
| M1 | Laser cutting: `bottom_plate.dxf` and `tilt_plate.dxf` in **6 mm plywood** | 1 each | They carry the weight. **Not acrylic** (it cracks). |
| M2 | Laser cutting: `top_deck.dxf` and `cargo_plate.dxf` in 5 mm acrylic | 1 each | |
| M3 | 3D prints (PETG): `servo_mount_front`, `servo_mount_rear`, `servo_clamp` × 4, `horn_adapter_front`, `horn_adapter_rear`, `bed_rest` × 2, `bed_riser` × 4, `line_sensor_bar`, `sonar_bracket`, `caster_cup` × 2, `caster_ring` × 2 (about 350 g, 18 hours) | 1 set | Any printer with a 200 × 200 mm bed. Every STL already lies in its printing position. Settings in section 1.2. |
| M4 | **M6 threaded rod**, 1 m (zinc-plated steel) | 1 | Cut into 6 pieces of 130 mm |
| M5 | M6 nuts + M6 washers | 26 each | 24 needed (4 per rod) |
| M6 | M3 screws + nuts for the motor brackets (length to suit: 6 mm plywood + the bracket) | 8 | You drill these holes to fit the bracket |
| M7 | M4 screws: × 10 (bed rests, 2), × 12 (servo mounts, 6), × 16 (servo ear clamps, 4), × 20 (horn adapters 4 + risers from below 8) | as listed | + 32 M4 nuts and M4 washers in all |
| M8 | M4 × 20 **countersunk (flat head)** | 8 | Down through the cargo plate into the risers: flush, so the package slides over them |
| M9 | M3 × 8 **countersunk** | 6 | Caster rings onto the cups (self-tapping into the print) |
| M10 | M3 × 8 screws | 8 | Servo horns onto the printed adapters (self-tapping) |
| M11 | M3 × 12 screws + nuts | 7 | Line sensor bar (2), IR modules (3), ultrasonic bracket (2) |
| M12 | Blue thread-locker; a little grease (Vaseline) | 1 | Coupling set screws, horn screws; the caster cups |
| M13 | Self-adhesive rubber pads, 18 mm, 2 mm thick | 2 | On the bed rests |
| M14 | Black matte tape **19 mm** (line) + white/light floor, masking tape | 1 roll each | Line following + station marks. Not 50 mm: it would cover all 3 sensors and look like a mark. |

**Tools you need:**

- Soldering iron, solder
- Multimeter (with a 10 A range)
- Wire stripper, side cutters, ferrule crimper
- Hacksaw and a file (to cut the threaded rod)
- Drill with a 3.5 mm bit (the only holes you drill yourself: motor brackets in plywood) and a 90° countersink bit (the cargo plate's 8 holes)
- Spanners 5.5 mm (M3), 7 mm (M4), 10 mm (M6 nuts) and 13 mm (M8 caster nuts), hex keys, screwdrivers, pliers; hot glue gun
- Heat gun or lighter, marker, tape measure, square ruler, protractor (or a phone angle app)

### 0.3 Send the plates to the laser shop (Monday/Tuesday)

1. Files: `cad/output/dxf/bottom_plate.dxf`, `tilt_plate.dxf`, `top_deck.dxf`, `cargo_plate.dxf`.
2. Tell the shop:
   - "Units are millimetres, cut every line, 1 piece each."
   - Material and thickness per the table above.
   - "Keep the protective film on the acrylic."
3. When you collect them, check them against the hole tables on sheets 03–06 of `CargoRobot_v2_drawings.pdf`:
   - **Bottom plate** (400 × 300 mm, plywood): 6 rod holes, 2 caster-bolt holes (8.5 mm), 4 small holes (2 at the front edge for the ultrasonic bracket, 2 for the line-sensor bar), 4 battery-strap slots, 3 cable slots.
   - **Top deck** (400 × 300 mm, acrylic): 6 rod holes, E-stop (22.5 mm), switch and button (12.2 mm), 2 LED holes, 8 M4 holes (servo mounts 6, bed rests 2), 2 servo-cable cut-outs.
   - **Tilting plate** (236 × 247 mm, plywood): 12 M4 holes (4 for the horn adapters, 8 for the risers).
   - **Cargo plate** (260 × 280 mm, acrylic): 8 M4 holes. **Countersink them yourself** on the top face (90° bit, slowly) so the flat screw heads sit flush.

### 0.4 Get the parts printed

Everything to print is in `cad/output/parts/`, and only that (the plates are DXF files, for the laser). **Each STL already lies in its printing position**: print it the way it opens in the slicer. Settings in [section 1.2](#12-3d-printing). About 18 hours of printing.

> **The servo mounts fit the DS5160** (the maker's drawing: 65 × 30 mm body, 75 mm over the ears, ears 31 mm above the case bottom). They clamp the ears instead of using the ears' notches, and they slide ±4 mm on the deck to put the servo shaft on the hinge axis. Nothing to measure before printing, as long as your servos are DS5160s (or the same size).
>
> **What the prints can't know:** where your motor bracket puts the axle (it depends on the bracket and on how the motor sits in it) and where your IR modules have their screw hole. So the **casters** set their height with nuts, and the **IR modules** sit in vertical slots: you adjust both on the robot, nothing has to be re-printed.

### 0.5 Set up your laptop

1. Install **Arduino IDE 2** from arduino.cc/en/software (Windows, macOS or Linux).
2. **Add the ESP32 boards:** *File → Preferences → Additional boards manager URLs*: paste `https://espressif.github.io/arduino-esp32/package_esp32_index.json` → OK.
3. *Tools → Board → Boards Manager* → search **esp32** → install **"esp32 by Espressif Systems"** (version 3.x). It is a big download (about 500 MB).
4. Plug in an ESP32 with the USB cable. No extra libraries are needed.
   - **Windows:** if no new COM port appears (*Tools → Port*), install the USB driver for the chip printed next to the USB socket: CP2102 → "CP210x VCP driver" from silabs.com; CH340 → "CH341SER" from wch-ic.com.
5. Test: *File → Examples → 01.Basics → Blink*. *Tools → Board → esp32 → **ESP32 Dev Module***, *Tools → Port* → the ESP32's port → **Upload**. The blue LED on the board blinks.

### 0.6 Put the project on GitHub

1. Create a free GitHub account for each team member. One person creates a **private** repository called `cargo_robot` and adds the others as collaborators.
2. [WINDOWS] Install **GitHub Desktop**. File → Add local repository → choose `C:\University\Gradfinal` → it offers to "create a repository here" → yes.
3. The project already has a `.gitignore` so the big CAD Python environment isn't uploaded.
4. GitHub Desktop → **Commit to main** → **Publish repository** (keep "private" ticked).
5. From now on, every change is committed with a short message ("tuned PID", "servo end points"). This gives you a history for the report, and a way back when something breaks.

**Done when:**

- [ ] Supervisor approved the changes.
- [ ] Parts ordered, with an expected delivery date written down.
- [ ] Plates at the laser shop, printing started.
- [ ] Blink runs on an ESP32 from your laptop.
- [ ] The repository is on GitHub.

---

## Week 1 — Fabrication & mechanical assembly (5 – 11 Oct)

**Goal:** a rolling chassis: plates, rods, motors, wheels, casters, sensors, E-stop, switch, button, LEDs, and the tilting bed with its servos, risers and cargo plate. There's no wiring yet.

Keep `cad/autocad/CargoRobot_v2_assembly.pdf` open while you work: it has the same steps as below with a picture of every stage. Sheet 07 of the drawings shows the tilting bed. All positions below are in millimetres, measured from the **centre** of the plate: +X is forward, +Y is left. Almost every hole is already cut in the plates, so the positions are mostly for checking.

![Top view](cad/output/views/render_top.png)

### 1.1 Laser-cut parts

Collect the four plates (section 0.3). Peel the film only from the side you'll write on. Mark the top face and the **FRONT** edge on each plate with a marker:

- **Bottom plate:** the 2 small ultrasonic-bracket holes are at the front edge.
- **Top deck:** the big 22.5 mm E-stop hole is at the front-left.
- **Tilting plate:** the 4 single holes (horn adapters) are along its **right** edge, at the front and rear corners.
- **Cargo plate:** it's symmetric, so any way round works.

### 1.2 3D printing

PETG, 0.2 mm layers, 245 °C nozzle / 80 °C bed, 4 walls. **Every STL already lies in its printing position:** print it as it opens in the slicer. Sheets 08–09 of the drawings show every part with its sizes.

| Part (file in `cad/output/parts/`) | Qty | Lies on | Infill | Supports |
|---|---|---|---|---|
| `servo_mount_front.stl`, `servo_mount_rear.stl` | 1 each | The face the servo's ears rest on | 40 % | None |
| `servo_clamp.stl` | 4 | Its flat face | 40 % | None |
| `horn_adapter_front.stl`, `horn_adapter_rear.stl` | 1 each | The round wall | 100 % | "Supports on build plate only" |
| `bed_rest.stl` | 2 | The end with the nut pocket | 40 % | None |
| `bed_riser.stl` | 4 | Its 80 × 16 mm face | 40 % | None |
| `line_sensor_bar.stl` | 1 | The flange's top face | 40 % | None |
| `sonar_bracket.stl` | 1 | Its base | 40 % | None |
| `caster_cup.stl` | 2 | Its top face (upside down) | 40 % | None |
| `caster_ring.stl` | 2 | Flat | 40 % | None |

### 1.3 Cut the threaded rods

1. Cut **6 pieces of 130 mm** from the M6 rod with a hacksaw.
2. **Trick:** before cutting, screw a nut past the cut mark. After cutting, unscrew it over the cut end. It re-forms the damaged thread.
3. File each cut end flat and lightly chamfered.

### 1.4 Assembly, step by step

Use nylock nuts where listed, and don't over-tighten anything that touches acrylic: firm with a small spanner is enough.

**Step 1: Motors into their brackets**

- [ ] Fix each motor to the steel L-bracket that came in its set, with the set's M3 screws. The shaft sticks out through the bracket's upright.

**Step 2: Brackets under the bottom plate** (the only holes you drill)

1. Turn the bottom plate upside down. Place a bracket so its flat base lies on the plate:
   - the upright (with the shaft) is **5 mm in from the side edge**;
   - the bracket is centred at X = 0 (the middle of the plate's length);
   - the motor body lies along the plate, pointing inwards.
2. Mark the bracket's 4 base holes, drill them to suit (about **3.5 mm**), and fix with M3 screws + nuts. Do the other side as a mirror image.
3. Route each motor's 6-wire cable up through the nearest cable slot (the 20 × 12 mm slots at X = +40, Y = ±70).

**Step 3: Drive wheels**

- [ ] Slide the hex coupling onto the motor shaft, its set screw on the **flat** of the shaft, a drop of blue thread-locker, tighten.
- [ ] Screw the 65 mm wheel onto the coupling with the set's screws.

**Step 4: Rods on the bottom plate**

For each rod:

1. Screw an M6 nut on ~7 mm from one end and add a washer. This is the bottom end.
2. Push the rod **up** through a 6.5 mm hole from below. Add a washer + nut on top and tighten the two nuts against the plate.
3. Screw a third nut down the rod and put a washer on it. Set it so the top of the washer is **exactly 100 mm above the plate's top surface**. Cut a 100 mm piece of wood or card as a gauge and use it on all 6 rods. The deck will rest on these 6 washers.

Rod positions (the holes are pre-cut): X = −190, +60, +190 at Y = ±135.

**Step 5: Ball casters**

1. Put a little grease (Vaseline) on the cone inside each `caster_cup`.
2. Push an **M8 × 50 bolt** up through the cup **from inside**, so its head sits in the hex pocket.
3. Drop the ball in, then screw the `caster_ring` on: 3 × M3 × 8 countersunk, from below. The ball must turn freely with a finger.
4. On the bolt: an M8 nut + washer, then push the bolt up through the 8.5 mm hole (X = +160 front, X = −160 rear), washer + nut on top, tighten with a 13 mm spanner.

The final height is set in section 1.5.

**Step 6: Front sensors**

1. **IR line sensors:** if a module's 3 pins stick straight out of its end, bend them **90° towards its front** (the side with the blue screw) with pliers, so the jumper wires plug in from the front.
2. Screw the 3 modules onto the front of `line_sensor_bar`: an M3 × 12 through each module's hole and the slot behind it, nut at the back. **LEDs pointing down, blue screw facing forwards**, module 1 on the robot's **left**, module 3 on its right.
3. Bolt the bar **under** the bottom plate: 2 × M3 × 12 down through the 2 holes just behind it (X = +69, Y = ±24), nuts under the flange. The bar's upright faces forwards. Run the 3 cables up through the slot behind the bar (X = +51).
4. Loosen each module's screw and slide it until its two LEDs are **5–10 mm above the floor** (put the robot on the floor for this, after section 1.5); tighten.
5. **Ultrasonic sensor:** push the HC-SR04 into `sonar_bracket` from behind: its two silver cans through the 18 mm windows, the board resting on the 2 small ledges, its 4 pins pointing **down** through the slot between them. Hold it with 2 dots of hot glue. Bolt the bracket on **top** of the bottom plate at the front edge, looking forward (2 × M3 × 12 + nut).

**Step 7: Deck parts** (before the deck goes on the rods)

1. **E-stop:**
   1. Unclip the contact block and unscrew the big ring nut from the button.
   2. Push the button through the **22.5 mm hole** (front-left) from **above**.
   3. Screw the ring nut on from below, then clip the contact block back on underneath.
   4. Attach 2 × 40 cm of 16 AWG wire to its **NC** terminals now (it's easier than later). Week 2 connects them.
2. **Master switch:** remove its nut, push it up through the **12.2 mm hole at the front middle** from **below**, add the washer + nut on top, attach 2 × 40 cm of 16 AWG wire.
3. **Start button:** the 12.2 mm hole at the **rear**: same way as the switch. Attach 2 × 30 cm jumper wires.
4. **LEDs:** push the 2 clips into the 6.2 mm holes next to the button (green nearer the button, red outside), then the LEDs into the clips. Note which leg is longer (+). Attach jumper wires.
5. **Servo mounts:** `servo_mount_front` on the front-right corner, `servo_mount_rear` on the rear-right corner. Put an M4 nut into each of the 3 slots in the mount's foot (the nut sits in the slot and can't turn), stand the mount on the deck, and push 3 × M4 × 12 up through the deck into the nuts. **Only snug them**: the mounts are lined up in step 12. Their uprights face each other across the deck's right edge; compare with `render_iso.png`.
6. **Bed rests:** push an M4 nut into the pocket **under** each `bed_rest`, stand it on its hole (X = ±40, Y = +95) and screw an M4 × 10 up through the deck into the nut. Stick a rubber pad on top.

**Step 8: Servos**

1. Each servo goes into its mount from the **bed side**, bottom first, through the window, until its **ears rest on the mount's face**. The shaft points towards the middle of the robot, at the deck's right edge.
2. **Ear clamps:** 2 per servo. An M4 × 16 through each `servo_clamp` and the mount, the clamp's narrow tip lying on the ear, a nut behind the mount. Tighten until the servo can't move. (The clamps hold the ears without using the ears' notches.)
3. Route each servo cable down through the cut-out next to its mount.

**Step 9: Put the deck on**

1. Lower the deck onto the 6 rods (rods through the 6.5 mm holes). It rests on the 6 washers at 100 mm.
2. Add a washer + nut on top of each rod and tighten.
3. Put a spirit level (or a phone level app) on the deck to check it's level. Adjust the nuts if needed.

**Step 10: Risers on the tilting plate**

The 4 printed risers hold the cargo plate high enough to clear the horn adapters. The tilting plate has 2 holes for each riser's **outer end**, near its front and rear edges.

- All 4 risers sit at Y = ±110, parallel to the robot's length. Their outer ends face outward (front risers to the front, rear risers to the rear); their inner ends point towards the middle of the bed.

1. Slide an M4 nut into each of a riser's 4 side slots, so it sits over the hole. All 4 risers are the same.
2. Stand the riser on the plate with its outer end over its 2 holes.
3. Push 2 × M4 × 20 + washers up **from below** into the riser's two outer nuts, and tighten.

**Step 11: Horn adapters and cargo plate**

1. Bolt `horn_adapter_front` and `horn_adapter_rear` on top of the tilting plate's two right-hand corners (round wall sticking out past the plate's end), each with 2 × M4 × 20 down through the plate and a nut underneath.
2. Screw an aluminium **disc** horn onto each adapter's round wall: turn the horn until 4 of its holes sit over the adapter's 4 slots, then 4 × **M3 × 8** (they cut their own thread; a drop of thread-locker). Not longer than 8 mm: a longer screw hits the adapter's base. Leave the horn's centre screw out for now.
3. **Countersink** the cargo plate's 8 holes on its top face (90° bit, by hand, slowly: acrylic melts if you rush), just deep enough for the M4 flat heads to sit flush.
4. Lay the cargo plate on the risers so its 8 holes sit over the risers' **inner ends**, and screw 8 × M4 × 20 countersunk **down** into the risers' inner nuts. Tighten gently: it's acrylic.
5. **No anti-slip mat:** the package must slide off when the bed tips.

**Step 12: Hang the bed on the servos, and line the servos up**

1. Lower the bed so it lies on the 2 rest posts, and push the two horns onto the servo splines. Each horn's hub must slide fully onto its spline. If one doesn't line up, turn that servo's shaft by hand (gently) one spline tooth at a time until it does.
2. Fit the horns' centre screws (thread-locker).
3. **Line up:** each servo mount can slide ±4 mm along the deck edge. Slide it until its horn sits **flat and centred** on the adapter's round wall, with no twist, then tighten the mount's 3 bolts under the deck.
4. Lift the bed's left edge by hand (servos unpowered they turn, stiffly): it must swing up to about 45° and back without touching anything. Watch the right edge of the cargo plate pass the servo mounts.

**Step 13: Check the bed**

- [ ] The bed lies level on its 2 rest posts and the 2 servo shafts.
- [ ] Nothing rubs anywhere between 0° and 45°.
- [ ] The cargo plate's right edge ends up just outside the deck when tipped (sheet 07 of the drawings: 6 mm).

### 1.5 Set the casters: all 4 wheels on the floor

The two drive wheels must carry most of the weight (for grip), and the caster balls must just touch the floor. The height depends on your motor bracket, so this is where you set it.

1. Put the robot on a flat floor. Slide a sheet of paper under each **caster ball**.
2. **Paper slides out freely** (the caster is too high):
   1. Loosen the top nut.
   2. Screw the lower nut **up** the bolt (away from the cup) by half a turn.
   3. Retighten the top nut.
3. **A drive wheel lifts off** (the caster is too low): do the opposite, moving the lower nut down towards the cup.
4. **Target:** each ball grips its paper, but a firm tug still pulls it out. Both drive wheels grip their paper hard.
5. Now set the IR modules' height (step 6.4): LEDs 5–10 mm above the floor.

### 1.6 Mechanical checks

- [ ] Push the robot by hand: it rolls straight, turns smoothly, the caster balls roll freely, nothing rubs.
- [ ] Put 3 kg on the cargo plate: nothing bends visibly, and no wheel lifts.
- [ ] Press the E-stop hard: the deck doesn't bend noticeably.
- [ ] Nothing under the robot is closer than 4 mm to the floor (the caster rings are the lowest point). Keep the track flat: no cables or door sills.

**Done when:** the chassis rolls freely with all 4 wheels on the floor and carries 3 kg, and the bed swings from 0° to 45° by hand without touching anything. Take photos for the report.

---

## Week 2 — Power & safety wiring (12 – 18 Oct)

**Goal:** battery → fuse → switch → 12 V bus → (E-stop → motor drivers and servo buck) and (buck → 5.1 V → ESP32). Everything is measured and safe.

> Work with the battery **unplugged**. Plug it in only for the measurements, with the master switch OFF first.

### 2.1 Power diagram

The full power schematic is sheet E02 in `cad/autocad/CargoRobot_v2_schematics.pdf` (also as an AutoCAD `.dxf`). Print it and keep it next to you while wiring. `cad/autocad/CargoRobot_v2_connections.pdf` has every wire of this week and the next in one place, also listed part by part.

```mermaid
flowchart LR
  B["LiPo +<br/>(XT60)"] -->|16 AWG red| F["Fuse 10 A"] -->|16 AWG| S["Master switch"] -->|16 AWG| P12["WAGO: +12 V MAIN bus"]
  P12 -->|20 AWG| BI["5 V buck IN+"]
  P12 -->|16 AWG| E["E-stop NC contact"] -->|16 AWG| M12["WAGO: +12 V MOTOR bus"]
  M12 -->|16 AWG| DL["Left BTS7960 B+"]
  M12 -->|16 AWG| DR["Right BTS7960 B+"]
  M12 -->|16 AWG| SB["7.4 V servo buck IN+"]
  M12 -->|signal| DIV["10k / 2.2k divider → GPIO 33"]
  BN["LiPo −"] -->|16 AWG black| G["WAGO: GND bus"]
  BO["5 V buck OUT"] --> V5["WAGO: 5 V bus → ESP32 5V pin"]
  SO["Servo buck OUT"] --> V7["WAGO: 7.4 V bus → 2 servos"]
```

### 2.2 Power wire list

<!-- power wires: generated by cad/make_docs.py from cad/wiring.py -->
| # | From | To | Wire | Notes |
|---|---|---|---|---|
| P1 | Battery BT1 **+ (XT60)** | Fuse holder F1 **in** | 16 AWG red, 10 cm | XT60 soldered |
| P2 | Fuse holder F1 **out** | Master switch SW1 **terminal 1** | 16 AWG red |  |
| P3 | Master switch SW1 **terminal 2** | WAGO +12 V MAIN | 16 AWG red | WAGO 221-415 |
| P4 | Battery BT1 **- (XT60)** | WAGO GND | 16 AWG black | WAGO 221-415, link a second one |
| P5 | WAGO +12 V MAIN | E-stop S1 **NC terminal 1** | 16 AWG red, 40 cm | ferrule |
| P6 | E-stop S1 **NC terminal 2** | WAGO +12 V MOTOR | 16 AWG red, 40 cm | ferrule |
| P7 | WAGO +12 V MOTOR | BTS7960 left U2 **B+** | 16 AWG red | ferrule |
| P8 | WAGO +12 V MOTOR | BTS7960 right U3 **B+** | 16 AWG red | ferrule |
| P9 | WAGO GND | BTS7960 left U2 **B-** | 16 AWG black | ferrule |
| P10 | WAGO GND | BTS7960 right U3 **B-** | 16 AWG black | ferrule |
| P11 | BTS7960 left U2 **M+** | Left motor M1 **red (motor +)** | 20 AWG or the motor's wire | ferrule |
| P11 | BTS7960 left U2 **M-** | Left motor M1 **white (motor -)** | 20 AWG or the motor's wire | ferrule |
| P12 | BTS7960 right U3 **M+** | Right motor M2 **red (motor +)** | 20 AWG or the motor's wire | ferrule |
| P12 | BTS7960 right U3 **M-** | Right motor M2 **white (motor -)** | 20 AWG or the motor's wire | ferrule |
| P13 | WAGO +12 V MAIN | 5 V buck U1 **IN+** | 20 AWG red | ferrule |
| P14 | WAGO GND | 5 V buck U1 **IN-** | 20 AWG black | ferrule |
| P15 | 5 V buck U1 **OUT+** | WAGO +5 V | 20 AWG red | set to 5.10 V first |
| P15 | 5 V buck U1 **OUT-** | WAGO GND | 20 AWG black |  |
| P15 | WAGO +5 V | ESP32 (breakout) **5V** | 20 AWG red |  |
| P15 | WAGO GND | ESP32 (breakout) **GND** | 20 AWG black | the logic ground starts here |
| P16 | Battery BT1 **balance plug** | LiPo voltage alarm | - | set it to 3.5 V per cell |
| P17 | WAGO +12 V MOTOR | Servo buck U5 **IN+** | 16 AWG red | ferrule |
| P18 | WAGO GND | Servo buck U5 **IN-** | 16 AWG black | ferrule |
| P19 | Servo buck U5 **OUT+** | WAGO +7.4 V | 16 AWG red | set to 7.4 V first |
| P19 | Servo buck U5 **OUT-** | WAGO GND | 16 AWG black |  |
| P19 | WAGO +7.4 V | Front servo SV1 **red (+)** | servo lead |  |
| P19 | WAGO GND | Front servo SV1 **brown (-)** | servo lead |  |
| P19 | WAGO +7.4 V | Rear servo SV2 **red (+)** | servo lead |  |
| P19 | WAGO GND | Rear servo SV2 **brown (-)** | servo lead |  |
<!-- end -->

> **Motor wire colours:** the most common JGB37-520 encoder cable is red = motor +, white = motor −, blue = encoder VCC, black = encoder GND, yellow = encoder A, green = encoder B. **Check your seller's diagram.** Only red and white go to the driver in this week; the other four are for week 3.

### 2.3 Building it, in order

1. **Battery lead.** Solder a male XT60 onto 10 cm of 16 AWG red + black. Heat-shrink each joint. Red goes to the XT60's **+** pin. Plug it into the battery: with the multimeter's red probe on the red wire, it must read **+11 to +12.6 V** (not a negative number).
2. **Fuse.** Crimp or solder the inline fuse holder into the red wire, as close to the XT60 as possible. Leave the fuse **out** for now.
3. **Switch** (P2, P3). Put the WAGO buses on the bottom plate in the space between the battery and the rear motor drivers (sheet 02 of the drawings), fixed with VHB tape.
   - Leave the wires going up to the deck (switch, E-stop, button, LEDs, servos) with **~10 cm of slack**, so the deck can be lifted off with the wires still connected.
4. **GND bus** (P4, P9, P10, P14, P18).
5. **E-stop** (P5, P6). With a multimeter on continuity between the two wire ends: button released → **beep**; pressed → **no beep**.
6. **Motor drivers** (P7–P12). Stick the BTS7960 boards to the bottom plate at the rear, one each side of the centre line, with **VHB tape**. Heatsinks up.
7. **5 V buck** (P13, P14). VHB at the rear-left. Don't connect its output yet.
8. **Servo buck** (P17, P18). VHB at the rear-right. Don't connect its output yet.
9. **Set both bucks (important):**
   1. Insert the 10 A fuse, connect the battery with the master switch **OFF**. Switch **ON**. Release the E-stop (the servo buck is after it).
   2. Multimeter on the 5 V buck's OUT+ / OUT−: turn its **voltage** pot (often many turns) to **5.10–5.20 V**. Turn its current pot fully clockwise.
   3. Multimeter on the servo buck's OUT+ / OUT−: turn its voltage pot to **7.4 V** (7.2–7.6 V). Never above 8.4 V: it destroys the servos.
   4. Switch **OFF**.
10. **5 V bus** (P15) and **7.4 V bus** (P19). Connect the buck outputs to their WAGOs. Don't connect the ESP32 or the servos yet.
11. **Battery alarm** (P16): plug it into the balance lead; set it to **3.5 V** if it's adjustable.

### 2.4 Bench tests: record the results for the report

With the battery **unplugged**:

| Test | How | Expect | Result |
|---|---|---|---|
| T2.1 No short circuit | Multimeter Ω between MAIN +12 V and GND | Not 0 Ω; it starts low and climbs as the bucks' capacitors charge | |
| T2.2 No short on motor bus | Ω between MOTOR +12 V and GND | Same as above | |

With the battery **connected**:

| Test | How | Expect | Result |
|---|---|---|---|
| T2.3 Switch off | Switch OFF, V at MAIN bus | 0 V | |
| T2.4 Main bus | Switch ON, V at MAIN bus | 11.1–12.6 V (the battery voltage) | |
| T2.5 E-stop | E-stop pressed: V at MOTOR bus and at the 7.4 V bus. Released (twist to unlatch): V again | 0 V / battery V and 7.4 V | |
| T2.6 5 V bus | V at the 5 V WAGO | 5.10–5.20 V | |
| T2.7 ESP32 boots | Connect the ESP32 breakout's 5V/GND (P15). Switch ON | The board's red power LED lights | |
| T2.8 Servos | Connect the servos to the 7.4 V bus (P19), signal wire not connected yet. Switch ON | Nothing moves, no buzzing; the 7.4 V bus stays at 7.2–7.6 V | |
| T2.9 Idle current | Multimeter in **10 A** mode in series with the fuse (take the fuse out, probes into the fuse holder) | ESP32 + sensors ≈ 0.1–0.2 A at 12 V | |

> **Never connect the ESP32's USB to the laptop while you probe the 12 V side with a meter in current mode.** Measure first, then plug in the USB.

### 2.5 Charging the LiPo

1. Charger mode **LiPo BALANCE**, **3S (11.1 V)**, current **5.0 A** (= 1 C for 5000 mAh).
2. Plug in both the main lead and the white balance plug.
3. Charge in the LiPo bag, on a tile or concrete floor, and stay nearby.
4. Full = 4.20 V per cell (12.6 V). Stop using it at 3.5 V per cell under load (the alarm beeps).
5. For storage longer than a week: charger **STORAGE** mode (3.8 V per cell).

**Done when:** T2.1–T2.8 pass. Pressing the E-stop kills motor and servo power but not the ESP32. The fuse is in.

---

## Week 3 — ESP32 setup, signal wiring, bench tests (19 – 25 Oct)

**Goal:** the firmware is on the ESP32, every sensor and motor is wired, and each one passes its bench test in the Serial Monitor.

### 3.1 Upload the firmware [LAPTOP]

1. Get the project onto your laptop (GitHub Desktop → *Clone repository*, or download the ZIP).
2. In the Arduino IDE: *File → Open* → `firmware/cargo_robot/cargo_robot.ino`. All the other files open as tabs.
3. *Tools → Board → esp32 → **ESP32 Dev Module***. *Tools → Port* → the ESP32.
4. **Upload.** If it stops at "Connecting....", hold the board's **BOOT** button until the upload starts.
5. *Tools → Serial Monitor*, **115200 baud**, line ending **Newline**. Press the board's **EN** (reset) button. You see `Cargo robot (ESP32) - type help`, followed by warnings about the parts that aren't wired yet. That's fine.
6. Type `help`: the list of commands.

> **Upload with the robot's power on or off?** Both work. With the battery off, the USB powers the ESP32 and you can do all the sensor tests except the motors and servos.

### 3.2 Signal wiring

Sheets E03 and E04 in `cad/autocad/CargoRobot_v2_schematics.pdf` show everything in this section, and `cad/autocad/CargoRobot_v2_connections.pdf` lists the same wires part by part (handy for checking one board at a time). Every wire with the same name is connected together. All numbers below are **GPIO numbers**, printed on the screw-terminal breakout ("25" or "GPIO25").

**Master switch OFF, battery unplugged, USB unplugged.**

**The I/O board.** Build it on the 5 × 7 cm perfboard, and stick it next to the ESP32 (sheet 02):

There is no I/O expander: every signal goes straight to an ESP32 pin. The board only carries resistors:

- 1 × 10 kΩ pull-down on the MOTOR_EN line (GPIO 13 → GND): the motors stay off until the ESP32 has started.
- 4 × 10 kΩ pull-ups on the encoder lines (GPIO 34, 35, 36, 39 have none inside).
- The ultrasonic ECHO divider: ECHO → 1 kΩ → junction → 2 kΩ → GND; the junction goes to the ESP32.
- The bus-sense divider: MOTOR +12 V → 10 kΩ → junction → 2.2 kΩ → GND (+ 100 nF across the 2.2 kΩ); the junction to GPIO 33.
- 2 × 330 Ω for the LEDs.
- Small buses (a row of linked pins, or WAGOs): **3V3** (from the ESP32's 3V3 pin), **GND**, **5V** (from the 5 V WAGO bus).

**Signal wire list** (24–26 AWG jumpers, ferrules into screw terminals):

<!-- signal wires: generated by cad/make_docs.py from cad/wiring.py -->
| # | From | To | Notes |
|---|---|---|---|
| S1 | BTS7960 left U2 **RPWM** | ESP32 (breakout) **GPIO 25** | forward PWM |
| S2 | BTS7960 left U2 **LPWM** | ESP32 (breakout) **GPIO 26** | reverse PWM |
| S3 | BTS7960 right U3 **RPWM** | ESP32 (breakout) **GPIO 27** | forward PWM |
| S4 | BTS7960 right U3 **LPWM** | ESP32 (breakout) **GPIO 14** | reverse PWM |
| S5 | BTS7960 left U2 **R_EN + L_EN (linked)** | ESP32 (breakout) **GPIO 13** | MOTOR_EN; 10k pull-down to GND |
| S5 | BTS7960 right U3 **R_EN + L_EN (linked)** | ESP32 (breakout) **GPIO 13** | MOTOR_EN |
| S6 | BTS7960 left U2 **VCC** | 3V3 bus | 3.3 V, not 5 V |
| S6 | BTS7960 right U3 **VCC** | 3V3 bus | 3.3 V, not 5 V |
| S7 | BTS7960 left U2 **GND (logic)** | GND bus |  |
| S7 | BTS7960 right U3 **GND (logic)** | GND bus |  |
| S8 | Left motor M1 **blue (encoder VCC)** | 3V3 bus | 3.3 V, never 5 V |
| S9 | Left motor M1 **black (encoder GND)** | GND bus |  |
| S10 | Left motor M1 **yellow (encoder A)** | ESP32 (breakout) **GPIO 34** | 10k pull-up to 3V3 |
| S11 | Left motor M1 **green (encoder B)** | ESP32 (breakout) **GPIO 35** | 10k pull-up to 3V3 |
| S12 | Right motor M2 **blue (encoder VCC)** | 3V3 bus | 3.3 V, never 5 V |
| S13 | Right motor M2 **black (encoder GND)** | GND bus |  |
| S14 | Right motor M2 **yellow (encoder A)** | ESP32 (breakout) **GPIO 36 (VP)** | 10k pull-up to 3V3 |
| S15 | Right motor M2 **green (encoder B)** | ESP32 (breakout) **GPIO 39 (VN)** | 10k pull-up to 3V3 |
| S16 | HC-SR04 (front) **VCC** | WAGO +5 V | 5 V |
| S17 | HC-SR04 (front) **GND** | GND bus |  |
| S18 | HC-SR04 (front) **TRIG** | ESP32 (breakout) **GPIO 4** |  |
| S19 | HC-SR04 (front) **ECHO** | ESP32 (breakout) **GPIO 23** | through a 1k / 2k divider (5 V -> 3.3 V) |
| S20 | Front servo SV1 **orange (signal)** | ESP32 (breakout) **GPIO 2** | servo lead |
| S21 | Rear servo SV2 **orange (signal)** | ESP32 (breakout) **GPIO 12** | servo lead; nothing else on this pin (must be LOW at start-up) |
| S22 | ESP32 (breakout) **3V3** | 3V3 bus | feeds every 3.3 V part |
| S22 | ESP32 (breakout) **GND** | GND bus |  |
| S23 | IR line sensor 1 (left) **VCC** | 3V3 bus | 3.3 V, not 5 V |
| S23 | IR line sensor 1 (left) **GND** | GND bus |  |
| S23 | IR line sensor 2 (middle) **VCC** | 3V3 bus | 3.3 V, not 5 V |
| S23 | IR line sensor 2 (middle) **GND** | GND bus |  |
| S23 | IR line sensor 3 (right) **VCC** | 3V3 bus | 3.3 V, not 5 V |
| S23 | IR line sensor 3 (right) **GND** | GND bus |  |
| S24 | IR line sensor 1 (left) **OUT** | ESP32 (breakout) **GPIO 16** | left |
| S24 | IR line sensor 2 (middle) **OUT** | ESP32 (breakout) **GPIO 17** | middle |
| S24 | IR line sensor 3 (right) **OUT** | ESP32 (breakout) **GPIO 18** | right |
| S25 | Start button S2 **terminal 1** | ESP32 (breakout) **GPIO 19** | internal pull-up |
| S25 | Start button S2 **terminal 2** | GND bus |  |
| S26 | Green LED D1 **+ (long leg)** | ESP32 (breakout) **GPIO 21** | through 330 ohm |
| S26 | Green LED D1 **- (short leg)** | GND bus |  |
| S26 | Red LED D2 **+ (long leg)** | ESP32 (breakout) **GPIO 22** | through 330 ohm |
| S26 | Red LED D2 **- (short leg)** | GND bus |  |
| S27 | Buzzer module **VCC** | 3V3 bus |  |
| S27 | Buzzer module **GND** | GND bus |  |
| S27 | Buzzer module **I/O** | ESP32 (breakout) **GPIO 32** |  |
| S28 | Bus-sense divider **top of 10k** | WAGO +12 V MOTOR | after the E-stop |
| S28 | Bus-sense divider **bottom of 2.2k** | GND bus | 100 nF across the 2.2k |
| S28 | Bus-sense divider **junction** | ESP32 (breakout) **GPIO 33** | about 2.3 V at 12.6 V; 0 V = E-stop pressed |
<!-- end -->

These pins are defined in one place, `firmware/cargo_robot/config.h`. If you ever move a wire, change it there too (and re-run `cad/make_schematics.py` so the drawings stay right).

**Before powering on**, check with the multimeter (continuity mode, battery unplugged):

- [ ] No continuity between 3V3 and GND, 5V and GND, 7.4 V and GND.
- [ ] No continuity between 3V3 and any 5 V, 7.4 V or 12 V bus.
- [ ] Measure the bus-sense junction with the battery in and the E-stop released: **2.0–2.3 V**. Above 3.3 V: the resistors are swapped. Fix it before connecting GPIO 33.
- [ ] The ECHO junction with the HC-SR04 powered: never more than 3.4 V.

### 3.3 Bench tests [SERIAL]

Put the robot **on a box, wheels in the air**. Battery in, master switch ON, E-stop released, USB to the laptop, Serial Monitor open. Every test is a command; **Enter** stops the live ones. Settings are changed with `set <name> <value>` and kept with `save` (see `list`).

**Test 1: Status**

Type `status`. One line: state, motor-bus volts (about your battery voltage), the 3 line sensors, the distance ahead (ultrasonic), the bed angle. Any `!!` warning at power-up names the part that isn't answering.

**Test 2: Encoders → `ticks_per_rev`**

```
enc
```

1. Turn each wheel slowly **forward** (the direction that would drive the robot forward) by hand. The counts must **increase**.
   - Count goes down → `set inv_l_enc 1` (or `inv_r_enc`).
   - Doesn't change → check the encoder's 3.3 V, GND, A and B wires and the 10 kΩ pull-ups.
2. Stick a piece of tape on the tyre. Press Enter, type `enc` again (the counts carry on from where they were: note them), and turn the wheel exactly **10 turns**.
3. `set ticks_per_rev <counts for 10 turns / 10>`. Sanity check: 11 pulses × 4 edges × gear ratio. For the 110 RPM motor (about 1:90) that's ≈ 3960.

**Test 3: Motor direction**

```
motortest
```

1. The test spins each wheel forward then backward (30 %, 2 s) and prints the encoder change. Watch each wheel:
   - On **FORWARD** the wheel must turn the way that drives the robot forward. If it turns the wrong way → `set inv_l_motor 1` (or `inv_r_motor`).
2. The test warns you if an encoder counts the wrong way and tells you what to type.
3. Run `motortest` again until both wheels are right with no warnings.

> Nothing moves? The test prints `motor power: OFF` when the E-stop is pressed. Also check that the R_EN/L_EN wires go to GPIO 13, and that driver VCC is on 3.3 V.

**Test 4: Minimum duty and top speed**

```
mintest
maxtest
```

Each prints the `set` command to type. Typical values: `min_duty` 0.08–0.2, `kf` 0.08–0.10.

**Test 5: Line sensors**

```
line
```

1. Put the robot on the floor, with a strip of black tape under the sensors: the sensors over it show `#` (3 characters: left, middle, right).
2. Each IR module has a blue screw and a small LED. With the robot on the floor (the LEDs 5–10 mm above it), turn each screw so its LED switches cleanly between the tape and the floor. Turn slowly: the switching point is a narrow band.
3. If the sensors show `#` over the **floor** and `.` over the tape: `set line_black 0`.
4. Slide the tape from left to right: `position` goes from +1 to −1. A strip **across** all 3 sensors shows `STATION MARK`.
5. One sensor never changes? Check its OUT wire (GPIO 16, 17 or 18), and that it's powered from 3.3 V.

**Test 6: Ultrasonic sensor**

```
sonar
```

Hold your hand 30 cm in front of the ultrasonic sensor: it reads about 30 cm.

**Save your settings and commit:**

```
save
```

Copy the output of `list` into a text file in the project (e.g. `firmware/settings_robot.txt`) and commit it: it's your record of the calibration.

**Done when:**

- [ ] Both encoders count up when driving forward, with the correct `ticks_per_rev`.
- [ ] Both wheels spin the right way.
- [ ] `min_duty` and `kf` are measured.
- [ ] `line` and `sonar` read sensibly.
- [ ] `status` shows `motor power` OFF (about 0 V) while the E-stop is pressed.

---

## Week 4 — The firmware and the tilting bed (26 Oct – 1 Nov)

**Goal:** you know how the firmware is organised, both servos are calibrated, and the bed tips packages off reliably.

### 4.1 How the firmware is organised

| File | Job |
|---|---|
| `config.h` | The wiring: every pin number, and a few fixed numbers |
| `settings.h / .cpp` | The ~33 tunable numbers (`list`), stored in the ESP32's flash |
| `hw.h / .cpp` | Drivers: motors (hardware PWM), encoders (the ESP32's pulse-counter hardware: no counts are missed), the line sensors, button, LEDs and buzzer (plain pins), the ultrasonic sensor (an interrupt), the servos (50 Hz pulses), the bus voltage |
| `control.h / .cpp` | Maths only: wheel PID, line position, the obstacle speed factor |
| `robot.h / .cpp` | `robotUpdate()`: called thousands of times per second. It reads every sensor that has news, runs the wheel controllers 100 times per second, moves the bed smoothly, drives the LEDs and buzzer |
| `console.h / .cpp` | The Serial-Monitor commands |
| `cargo_robot.ino` | `setup()`, `loop()` and the mission (week 8) |

```mermaid
flowchart LR
  LOOP["loop()"] --> UPD["robotUpdate()<br/>sensors, wheels, bed"]
  LOOP --> CON["consolePoll()<br/>Serial commands"]
  LOOP --> STEP["step()<br/>mission + safety"]
  STEP -- "driveTarget(v, w)" --> UPD
  STEP -- "bedMove(deg)" --> UPD
```

### 4.2 Calibrate the servos

Each servo needs two numbers: its pulse (µs) with the bed **flat**, and with the bed at **40°**. The front and rear servos are mirror images, so one goes up as the pulse grows and the other as it shrinks.

1. Robot on the floor, battery in, E-stop released, bed resting flat. Keep your hands off the bed.
2. Front servo, flat: `servo f 1500`. The front of the bed may jump. Change the number in steps of 10–20 µs (`servo f 1480`, `servo f 1520` …) until that servo holds the bed **exactly flat on its rest post**, without pushing it up and without buzzing. Write the number down: `set f_flat_us <number>`.
3. Front servo, tipped: increase (or decrease, if the bed goes down into the deck: stop at once) the pulse until the bed's front end is at **40°** (phone angle app on the cargo plate). `set f_tip_us <number>`. A 180° servo moves about 11 µs per degree, so expect about 1500 ± 444.
4. `servo f off`. Do the same for the rear servo with `servo r …`, `set r_flat_us …`, `set r_tip_us …`, `servo r off`.
5. Check: `bed 20` (both servos move smoothly to 20°: the bed must not twist; if one end is higher, adjust that servo's numbers), `bed 40`, `bed 0`, `bed off`. Then `save`.

> **If a servo moves the wrong way or hits its end**, its horn is on the wrong spline tooth: take the horn's centre screw out, lift the bed off that servo, turn the shaft to the flat position (`servo f <flat number>`), and push the horn on again with the bed flat.

### 4.3 Tipping tests (report data)

Use cardboard boxes of 0.5, 1, 2 and 3 kg (fill them with rice or books), 10–20 cm wide.

```
tip
```

The bed tips to `tip_deg` at `tip_speed` deg/s, holds for `tip_hold_s` and comes down. Look: did the box slide off? Do it 10 times per box, with the box in the middle and near each corner. Record the angle at which each box starts to slide (watch the angle in a phone video).

| Box | Starts sliding at (deg) | Delivered (out of 10) |
|---|---|---|
| 0.5 kg | | |
| 1 kg | | |
| 2 kg | | |
| 3 kg | | |

- Boxes stay on? Increase `tip_deg` (up to 45) or `tip_hold_s`, and check the cargo plate is clean and dry (no mat!).
- Boxes tumble instead of sliding (tall, narrow ones)? Lower `tip_speed` (10–15 deg/s).
- A servo buzzes or the bed shakes at 40° with 3 kg? The servo buck must be 7.4 V under load: measure it during the tip.

**Done when:** a 3 kg box slides off 10 times out of 10, and the bed comes back flat onto its posts every time.

---

## Week 5 — Driving: motor control, PID, odometry (2 – 8 Nov)

**Goal:** the wheel speed control is tuned, and the distance the robot measures is accurate to a few %.

### 5.1 How the driving code works

`robotUpdate()` runs the wheels 100 times per second:

1. It reads both encoder counts and turns the change into wheel speeds (rad/s), filtered a little, and adds up the distance travelled and the change of heading.
2. It ramps the wanted forward speed by `accel` (0.4 m/s² by default), so starts and stops are gentle and the package doesn't slide.
3. It turns the wanted robot speed (v m/s forward, w rad/s turning left) into wheel speeds:
   - left = (v − w·s/2) / r
   - right = (v + w·s/2) / r
4. A **PID controller per wheel** turns wanted vs. measured wheel speed into PWM duty:
   - duty = sign · (`min_duty` + `kf`·|target|) + `kp`·error + ∫`ki`·error
   - The feed-forward part (`kf`, `min_duty`, measured in week 3) does most of the work. `kp` and `ki` correct what's left.
5. Safety: the motors only run while GPIO 13 (MOTOR_EN) enables the drivers, and the mission switches them off whenever it stops.

### 5.2 First drive (wheels in the air, then on the floor)

```
drive 0.15 0 3
```

drives forward at 0.15 m/s for 3 s and prints both wheel speeds 4 times per second (target: 0.15 / 0.0325 = 4.6 rad/s), then the distance and turn it measured. Try `drive -0.15 0 3` (backwards), `drive 0 1 3` (turn left on the spot), `drive 0 -1 3`. Then on the floor, gently. **Enter** stops early.

### 5.3 Tune the wheel speed controller

Run `drive 0.3 0 3` with the wheels in the air and watch the printed speeds (target 6 rad/s):

| What you see | Change |
|---|---|
| Both settle clearly **below** 6 | increase `ki` (e.g. 0.10 → 0.2), or check `kf` |
| Slow to reach the speed | increase `kp` (0.02 → 0.04) |
| Speed jumps around / motors buzz / robot jerks | decrease `kp`, then `ki` |
| Jumps forward when starting | decrease `min_duty` a little |
| Doesn't start moving at low speeds | increase `min_duty` a little |

Change with `set`, try again, `save` when happy.

> A good test of the tuning: with 3 kg on the bed, it should still reach the commanded speed.

### 5.4 Calibrate the odometry (on the floor)

**Straight-line test → `wheel_radius`**

1. Put a tape mark on the floor at the robot's axle (the centre of a drive wheel). Measure out **3.00 m** in a straight line and mark the end.
2. `drive 0.15 0 25` and press **Enter** the moment the axle reaches the end mark. Note the printed distance (odometry), and measure the real distance D the axle travelled.
3. **New wheel_radius = old wheel_radius × D / odometry distance.** `set wheel_radius …`

**Spin test → `wheel_sep`**

1. Put a tape arrow on the floor pointing the way the robot faces.
2. `drive 0 0.6 60` and press Enter when the robot has turned **exactly 5 full turns** (1800°). Note the printed angle.
3. **New wheel_sep = old wheel_sep × printed angle / 1800.** `set wheel_sep …`

`save`, and repeat both tests once more to confirm.

### 5.5 The straight-line test (report data)

Drive 3 m 5 times (`drive 0.2 0 15`, stopping at the mark). Measure how far sideways from the line the robot ends, and how far the printed distance is from the real one.

| Run | Real distance (m) | Odometry (m) | Sideways error (cm) |
|---|---|---|---|
| 1 | | | |
| ... | | | |

**Done when:** the wheels follow the commanded speed within ~10 %, and the 3 m test ends within ~5 cm. `save`, and commit your `list` output.

---

## Week 6 — Line following and station marks (9 – 15 Nov)

**Goal:** the robot follows a tape track for 10 laps and stops on every station mark.

### 6.1 Lay the track

- **Line:** black matte tape, **19 mm wide** (wider tape would cover all 3 sensors and look like a mark), on a light, non-shiny floor. A closed loop, curves with a radius of at least 30 cm. Start with an oval about 3 × 1.5 m.
- **Station marks:** a strip of the same tape **across** the line, at least 12 cm long and 2–5 cm wide (so all 3 sensors see black at once). One for the **pickup**, one for the **drop-off**.
- At the drop-off, the conveyor will be on the robot's **right**, so put that mark on a straight part of the track that runs alongside where the belt will be. For now, use a box or a wall 5–10 cm to the right of where the robot stops.

### 6.2 How the line following works

- The 3 sensors (18 mm apart) give a position: +1 = line under the left sensor, +0.5 = between left and middle, 0 = centred, −1 = under the right sensor. 19 mm tape is always under at least one of them while the robot is on the line.
- Turning speed = (`k_line` × position + `kd_line` × how fast the position changes), scaled with the forward speed (so the path is the same when the robot slows down).
- **Line lost** (no sensor on black) for more than `lost_s`: the robot stops and raises an alarm.
- **Station mark** (`mark_min` = all 3 sensors on black for 2 readings): the robot drives on `stop_mm` (98 mm: from the sensors to the middle of the bed) and stops, so the middle of the bed is level with the mark.

### 6.3 Tune it

There's no "follow only" command: use the mission in week 8, but first try it on the bench:

1. `line` with the robot on the track: check the position goes + when the tape is under the left sensors.
2. Put the robot on the track **just after the pickup mark**, facing the drop-off. Put a 0.5 kg box on the bed and type `run 1`: it sets off at once.
3. It follows the line to the drop-off mark, stops, tips (that's week 8), and follows on to the pickup.

| Symptom | Fix |
|---|---|
| Snakes left-right along a straight line | lower `k_line` (2.5 → 1.8), raise `kd_line` a little |
| Loses the line in curves | raise `k_line` (→ 3.5) and/or lower `follow_speed` (→ 0.15) |
| Turns the **wrong** way, off the line at once | modules 1 and 3 are swapped: swap their OUT wires (GPIO 16 and 18), or `set k_line -2.5` |
| Stops at a mark too early / too late | change `stop_mm` |
| Sees a station mark in a sharp curve | make the curve gentler (radius ≥ 30 cm), or the tape narrower |
| Doesn't see a mark | make the mark longer (≥ 12 cm across), and check all 3 LEDs switch over it (`line`) |
| Wobbles a lot even on straights | the 3 sensors only give 5 positions: lower `k_line` (→ 1.8) and `follow_speed` (→ 0.15) |

### 6.4 Line-following tests (report data)

| Test | How | Record |
|---|---|---|
| L1 | 10 laps at 0.2 m/s, empty | laps without losing the line, lap time |
| L2 | 10 stops at a mark | distance from the middle of the bed to the mark (cm) |

**Done when:** ≥ 9/10 laps without losing the line, and every stop within ±3 cm of its mark. `save`.

---

## Week 7 — Obstacles (16 – 22 Nov)

**Goal:** the robot slows down for something ahead, stops for it, and drives on once it's gone.

### 7.1 Obstacles

| What the ultrasonic sensor sees ahead | What the robot does |
|---|---|
| Nothing closer than `slow_cm` (60 cm) | full speed |
| Something 60 → 25 cm ahead | slows down in proportion |
| Something closer than `stop_cm` (25 cm) | **stops**, red LED flashes, one beep per second ("please remove the obstacle"), drives on 1 s (`clear_s`) after the way is clear |

The sensor looks straight ahead along the line, so the conveyor and walls beside the track don't stop the robot.

Test it during a mission: walk in front of the robot, put a box on the track.

**Done when:**

- [ ] It slows down as a box comes closer than 60 cm.
- [ ] It stops for a box in front and drives on when it's taken away.
- [ ] It does the same with 3 kg on the bed (the wheel speed control keeps the speed up).

---

## Week 8 — Delivery missions (23 – 29 Nov)

**Goal:** the robot runs the full delivery by itself: pickup → drop-off (tip) → pickup, 5 times in a row.

### 8.1 The mission

Put the robot on the track **just after the pickup mark** (middle of the bed level with the mark), facing the drop-off, and power it on. Put the first package on the bed, then press the **start button** or type `run` (`run 5` = 5 deliveries then stop). It sets off at once.

The robot can't weigh: **never load more than 3 kg.**

| State | What happens | Lights |
|---|---|---|
| LOAD | Back at the pickup: waits for you to put the next package on and press the start button, then beeps and sets off | green on |
| FOLLOW | Follows the line to the next station mark: slow/stop for obstacles | green on |
| DOCK | Drives on `stop_mm`, stops level with the mark | green on |
| SETTLE | At the drop-off: waits half a second for the robot to stand still | |
| TIP | Tips the bed to 40° at 20°/s | green flashing |
| HOLD | Holds it there for `tip_hold_s` (2 s) | green flashing |
| LOWER | Lowers the bed, switches the servos off and goes on to the pickup. It can't check that the package fell off: watch it | |
| ALARM | Line lost: red LED, beep every 2 s. Fix it, press the start button: it carries on from where it stopped | red on |
| ESTOP | E-stop pressed: everything stops (the servos lose power; the bed sinks back onto its posts). When released: back to IDLE; press start to begin again | red on |

`stop` in the Serial Monitor stops everything at any time and lowers the bed.

### 8.2 With the real conveyor

When the conveyor exists, put the drop-off mark where the middle of the bed is level with the belt, and the belt 2–15 cm to the right of the robot's deck edge, with its top **below 190 mm** from the floor (the package leaves the bed at about 199 mm, 6 mm outside the deck). Nothing checks that the belt is there: put the mark exactly where it should be.

### 8.3 Run the delivery mission

Load at the pickup (by hand) and press the start button, watch it deliver, repeat. Record a video of 5 deliveries in a row for the report.

**Done when:** the delivery mission runs **5 times in a row** without help, with 1 kg and with 3 kg packages.

---

## Week 9 — Field testing & metrics (30 Nov – 6 Dec)

**Goal:** measured numbers for the report. Each test lists what to do, what to record and what counts as good. Put every result into a spreadsheet: mean, standard deviation, min and max.

**Recording data:** `watch` prints a status line twice a second. In the Serial Monitor, untick *autoscroll*, select the lines, copy them into a spreadsheet. Film every test with a phone as well; slow motion (240 fps) is great for timing.

### T1: E-stop response time

1. The robot follows the line at 0.2 m/s. Film it from the side in slow motion.
2. Press the E-stop as it passes a tape mark.
3. Count frames from the finger touching the button to the wheels stopping (at 240 fps each frame = 4.2 ms). Measure the stopping distance from the mark.
4. Repeat 5 times, and again with 3 kg on board.
5. **Good:** < 0.3 s and < 10 cm.

### T2: Obstacle reaction

1. The robot follows a straight line. Someone places a box on the line 1 m in front of it.
2. **Record** (from video): the time from the box landing to visible slow-down, the distance at which it stops, whether it touched the box.
3. Repeat 5 times.
4. **Good:** reacts within 0.5 s, stops 20–30 cm from the box, never touches it.

### T3: Station-stop accuracy

1. Run 10 deliveries.
2. At each stop, measure the distance from the middle of the bed to the mark, along the line.
3. **Good:** within ±3 cm.

### T4: Unloading reliability

1. 10 deliveries each with 0.5, 1, 2 and 3 kg boxes.
2. **Record:** delivered / not delivered, the time from stopping to driving on.
3. **Good:** 10/10 for every weight, under 6 s each.

### T5: Battery life under full payload

1. Fully charged battery, 3 kg box.
2. `run` and keep loading at the pickup (or use a fixed box and let someone put it back after each delivery).
3. **Record:** the start time, the number of deliveries, and the time when the voltage alarm beeps (3.5 V per cell).
4. Estimate: runtime ≈ 5 Ah × 0.8 / average current (clamp meter or 10 A multimeter on the battery lead while driving).

### T6: Line following reliability

1. 10 laps of the track. **Record:** lap time and the number of times the line was lost.
2. **Good:** ≥ 9/10 laps without losing the line.

### Results table (copy into the report)

| Test | Metric | Result | Target | Pass? |
|---|---|---|---|---|
| T1 | E-stop time / distance | | < 0.3 s / 10 cm | |
| T2 | obstacle reaction time | | < 0.5 s | |
| T3 | station-stop error (mean/max) | | < 3 cm | |
| T4 | unloading success @3 kg | | 10/10 | |
| T5 | runtime @ 3 kg | | ≥ 1 h | |
| T6 | line-follow laps without loss | | ≥ 9/10 | |

**Done when:** every row has a result, backed by a video or data file.

---

## Week 10 — Cable harness, covers, final checks (7 – 13 Dec)

**Goal:** wiring that looks professional and survives the demo, protective covers, and a robot that is ready the moment it is switched on.

### 10.1 Cable harness

1. **Label every wire** at both ends (heat-shrink labels or tape + marker) with its ID from the wire lists (P1…P19, S1…S28).
2. **Route power and signals apart:**
   - power wires (red/black, motor and servo wires) along the **left** side;
   - signal wires along the **right** side;
   - where they must cross, cross at 90°.
3. **Twist** each motor's two power wires together (≈ 1 twist per 2 cm). This reduces electrical noise into the encoders.
4. **Bundle** with zip ties every 8–10 cm. Use spiral wrap where a bundle passes the cable slots or bends.
5. **Strain relief:** every cable that goes to a moving or removable part (the motors, the top deck, **the bed**) gets a zip tie 3–5 cm before its connector. The bed bundle is tied at the hinge zip tie on the tilting plate.
6. **Ferrules:** every stranded wire in a screw terminal must have a crimped ferrule. Tug each wire: none may come out.
7. **Service loop:** leave ~10 cm of slack on the wires between the bottom plate and the top deck.
8. Tip the bed to 45° (`bed 45`): no wire may pull tight. `bed 0`, `bed off`.
9. Take a clear photo from above for the report.

### 10.2 Covers

- The simplest cover is 2–3 mm acrylic or PVC-foam side panels, 400 × 100 mm, one per side (plus front/rear pieces if you want). Hold them against the outside of the rods with small zip ties through 4 mm holes, or VHB tape on the plate edges.
- Leave openings for: the master switch, the ESP32's USB socket (left side), the ultrasonic sensor (front), and airflow at the motor drivers and bucks.
- **Never** cover the space under the bed's right edge or the servo horns.

### 10.3 Power-on routine

The ESP32 starts its program by itself at power-on (no laptop needed):

> **Power-on:**
> 1. Cargo bed flat on its rests.
> 2. E-stop released.
> 3. Master switch ON. The green LED blinks slowly after about 1 s (IDLE).
> 4. Put the robot on the track after the pickup mark, put the first package on (3 kg at most), press the start button.
>
> **Power-off:** master switch OFF. Nothing to shut down first.

### 10.4 Back up the settings

`list` in the Serial Monitor → copy into `firmware/settings_robot.txt` → commit. If an ESP32 dies on demo day, upload the firmware to the spare (2 minutes) and type the settings back in with `set` + `save` (5 minutes).

**Done when:** tidy, labelled wiring (photo); covers on; the robot works from power-on without the laptop; the spare ESP32 has the firmware.

---

## Week 11 — Report, video, presentation, demo (14 – 20 Dec)

### 11.1 Report outline

1. **Introduction:** problem (indoor cargo delivery and sorting), objectives, scope (the robot; the conveyor and vision are the partner project).
2. **Background:** differential drive, line following, PID, ultrasonic ranging, servos.
3. **System design:**
   - requirements;
   - block diagram (section 1, sheet E01);
   - design changes from the Phase II plan, with reasons (the table in section 1).
4. **Mechanical design:**
   - CAD renders (`cad/output/views/*.png`), and the 10 drawing sheets from `cad/autocad/CargoRobot_v2_drawings.pdf` as an appendix;
   - material choices;
   - load path (package → cargo plate → risers → tilting plate → servos and rest posts → deck → threaded rods → bottom plate → wheels and casters);
   - **the tilting bed:** why servos on the hinge (no linkages), the torque calculation (sheet 07: 4.7 N·m for 3 kg, 40 % of the servos' stall torque), the tipping tests (week 4);
   - design choices that keep it simple to build: threaded rods, a bought motor set (motor, bracket, coupling, wheel), printed ball casters set with nuts, servo mounts that clamp the servo and slide into line, laser-cut holes, and `check_render.py`'s automatic checks (bolt paths, collisions, printability).
5. **Electrical design:**
   - power diagram (E02), wire lists P/S, the ESP32 pin map (E03, E04);
   - E-stop design and the divider calculation;
   - power budget (T2.9, T5).
6. **Software design:**
   - the firmware's structure (section 4.1) and the mission state machine (section 8.1);
   - the PID + odometry maths (section 5.1), line following (section 6.2), obstacle handling (section 7.1).
7. **Calibration:** encoders, PID, odometry (week 5 data), servos.
8. **Testing & results:** T1–T6 tables, graphs, discussion of every failure.
9. **Conclusion & future work:**
   - talk to the conveyor's ESP32 over Wi-Fi (ESP-NOW) so the conveyor knows a package is coming;
   - tip either side (a second pair of servos, or a centre pivot);
   - a docking check with the camera; charging contacts at the pickup;
   - a fleet of robots.
10. **Appendices:** BOM with costs, full wiring tables, code (from GitHub), and this timeline with the actual dates.

**Figures to collect:**

- CAD renders (flat and tipped); photos of each build stage;
- wiring photo;
- the track with its station marks;
- a speed-vs-time plot from the PID tuning;
- a frame sequence of the bed tipping a package off;
- test result charts.

### 11.2 Video

Film in landscape, 1080p, good light, 2–3 minutes total. Shot list:

1. Robot overview, 360° walk-around, then the bed tipping on the bench (15 s).
2. Line following a full lap (20 s).
3. **Full delivery mission:** loading at the pickup, start button, stop at the drop-off, bed tips, package slides onto the conveyor, return (60 s).
4. Obstacle: a person steps in front, the robot stops and beeps, then drives on (15 s).
5. E-stop press (10 s).
6. Slow motion of the package sliding off (10 s).

### 11.3 Slides

About 12–15 slides, 15 minutes:

1. Title
2. Problem & objectives
3. System overview (block diagram, robot + conveyor + laptop)
4. Mechanical design (renders + photo)
5. The tilting bed
6. Electrical design (power diagram, safety chain)
7. Firmware (structure, mission states)
8. Line following and station marks
9. Test results (the results table)
10. Video
11. Challenges & lessons learned
12. Conclusion & future work
13. Questions

### 11.4 Demo-day checklist

- [ ] Two batteries charged (a full one in the robot, a spare in the LiPo bag), plus the charger.
- [ ] Spare fuses, zip ties, screwdrivers, hex keys, multimeter, black tape.
- [ ] Spare ESP32 with the firmware, and the settings file printed.
- [ ] Laptop charged, Arduino IDE and the USB cable tested at home.
- [ ] Track and station marks laid (arrive 1 hour early if the room is new: 20 min of tape plus a few test laps).
- [ ] Test packages (0.5, 1, 3 kg boxes).
- [ ] The video on the laptop **and** a USB stick (plan B if the live demo fails).
- [ ] Two full rehearsals done in the week before, timed.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Upload stops at "Connecting...." | The ESP32 isn't entering download mode | Hold the **BOOT** button until the upload starts. Unplug the rear-servo signal wire (GPIO 12) and try again. |
| No COM port appears | USB driver or a charge-only cable | Install the CP210x or CH340 driver (section 0.5); try another USB cable |
| The ESP32 resets in a loop, `flash read err` | GPIO 12 is pulled HIGH at start-up | Only the rear servo's signal may be on GPIO 12; check for a short to 3.3 V |
| Motors don't move, no errors | E-stop pressed, or EN not reaching the drivers, or driver VCC not powered | `status` shows the motor-bus volts; check S5 (R_EN/L_EN → GPIO 13) and 3.3 V at the driver VCC |
| One wheel spins the wrong way | Motor wires reversed | `set inv_l_motor 1` (or right), `save` |
| A drive wheel slips / the robot rocks | A caster is set too low and lifts the drive wheel | Redo the caster setting (section 1.5) on a flat floor |
| A caster ball doesn't roll / squeaks | Dry seat, dirt, or the ring screwed on too tight | Grease the cup's cone; clean the ball; loosen the ring screws a quarter turn |
| Robot spins in circles when told to go straight | One encoder is inverted, or its motor is | Redo `motortest` until there are no warnings |
| Encoder count doesn't change | No pull-up, or wrong supply | 10 kΩ from each encoder line to 3.3 V; encoder VCC on 3.3 V |
| The bed shakes or a servo buzzes | Pulse beyond the end stop, or the servo supply sags | Recalibrate (section 4.2); measure 7.4 V during a tip; the servo buck must be the 8 A XL4016 |
| A servo twitches at power-on | Normal for some servos for a moment | If the bed moves more than a few degrees, check the signal wire isn't swapped with another |
| The bed tips but the box stays on | Too little angle, a sticky plate, or a box with a rubbery base | Raise `tip_deg` (max 45), clean the plate, use cardboard boxes |
| Robot snakes along the line | `k_line` too high | Lower `k_line`, raise `kd_line` a little |
| Line lost in curves | Too fast or `k_line` too low | Lower `follow_speed`, raise `k_line` |
| Stops at random places | It sees station marks in curves, or shadows | Make curves gentler; adjust the sensors' blue screws; avoid strong sun patches (these IR modules are sensitive to sunlight) |
| A line sensor shows `#` all the time | Its blue screw is set too far, or it's too high above the floor | Re-set the screw over the floor; lower the module in its slot (LEDs 5–10 mm above the floor) |
| "obstacle" with nothing in front | The ultrasonic sensor sees the floor or the robot | Check it points straight ahead and level; `sonar` shows what it reads |
| The robot stops at the right place but the box misses the belt | `stop_mm` or the mark's position | Move the mark, or change `stop_mm` |

---

## Appendix A — Skills crash course

### A.1 The Arduino IDE and the Serial Monitor

| Action | How |
|---|---|
| Open the firmware | *File → Open* → `firmware/cargo_robot/cargo_robot.ino` |
| Compile only (check for errors) | the **✓** button |
| Upload | the **→** button (hold BOOT if it hangs at "Connecting....") |
| Serial Monitor | *Tools → Serial Monitor*, 115200 baud, **Newline** |
| Restart the ESP32 | the **EN** button on the board |
| Find the right port | unplug the ESP32, look at *Tools → Port*, plug it in again: the new entry is it |

### A.2 Settings

- `list`: every setting with its value.
- `set kp 0.03`: change one; it works at once.
- `save`: keep everything in flash (survives power-off and firmware uploads).
- `defaults`: back to the values in `settings.h`.

### A.3 Git in 5 commands

```bash
git pull                          # get teammates' changes (do this first!)
git status                        # what changed
git add -A                        # stage all changes
git commit -m "short message"     # save a snapshot
git push                          # upload
```

**If `git pull` reports a conflict:** open the file, keep the right lines (delete the `<<<<`, `====`, `>>>>` markers), then `git add -A && git commit`. GitHub Desktop does the same with buttons.

### A.4 Soldering

1. Iron at 330–350 °C. Clean the tip on a damp sponge, then tin it (melt a little solder on it).
2. Twist the wire strands, then **tin the wire** (heat it and feed solder until it soaks in).
3. For a wire-to-wire joint, slide heat-shrink on **first**. Hold the parts together, heat both, feed solder into the joint (not onto the iron) for 2–3 s. Let it cool without moving.
4. **A good joint is shiny and smooth.** A dull, blobby joint is a "cold joint": redo it.
5. XT60 connectors: tin the cup, heat the cup, push the tinned wire in, hold still 3 s. Plug a mating connector on while soldering so the plastic keeps its shape.

### A.5 Ferrules and crimping

1. Strip 10 mm of the wire (match the ferrule's metal length).
2. Twist the strands and push the wire into the ferrule until the insulation sits in the plastic collar.
3. Crimp in the matching slot of the crimper.
4. Tug test: a hard pull must not remove it.

### A.6 Multimeter

| Measure | Dial | Probes |
|---|---|---|
| DC voltage | V⎓ (20 V range if manual) | black COM, red VΩ; touch across the two points |
| Continuity | Ω / diode (beep) | power OFF; beep = connected |
| Resistance | Ω | power OFF |
| Current (up to 10 A) | A⎓ 10 A | red probe moved to the **10 A** socket; the meter goes **in series** (break the circuit and put the meter in the gap). Move the red probe back afterwards, or you'll blow its fuse next time! |

### A.7 Threaded rod and nuts

- **Cutting:** clamp the rod in a vice (wrap it in cloth or cardboard to protect the thread). Cut with a hacksaw, using long, slow strokes.
- **Thread repair:** screw a nut on before cutting and unscrew it over the cut end afterwards (section 1.3).
- **Locking:** two nuts tightened against each other, or against a plate, won't loosen. For extra safety, add a drop of blue thread-locker on the top nuts once everything is adjusted.
- **Setting heights:** always measure from the same reference (the top of the bottom plate), using one gauge block for all rods.

### A.8 Regenerating the CAD files

After changing a number in `cad/robot_cad.py` [WINDOWS PowerShell in `C:\University\Gradfinal\cad`]:

```powershell
.venv\Scripts\python robot_cad.py        # STEP, STL, DXF, SVG, BOM -> output/
.venv\Scripts\python check_render.py     # collision check (bed flat and tipped), wheels on the floor, torque, PNG renders
.venv\Scripts\python make_autocad.py     # AutoCAD file (3D model + 10 drawing sheets) + drawings PDF
.venv\Scripts\python make_schematics.py  # electrical schematics (pins from firmware/cargo_robot/config.h) + PDF
.venv\Scripts\python make_docs.py        # assembly PDF + connections PDF; updates the wire tables and Appendix B here
```

The CAD environment needs CadQuery, ezdxf and matplotlib, plus `reportlab` and `pypdf` for `make_docs.py` (`.venv\Scripts\python -m pip install reportlab pypdf`). Wires live in `cad/wiring.py`: change a wire there, not in this guide.

**AutoCAD:**

1. Open `cad/autocad/CargoRobot_v2.dxf` (File → Open → *Files of type: DXF*).
2. Save it as a `.dwg`.
3. The Model tab has the 3D robot. Set the visual style to *Shaded with Edges* and orbit with Shift + middle mouse button.
4. The layout tabs 01–10 are the A3 drawing sheets, ready to plot.

**Other CAD programs:** open `output/robot_assembly.step` (or `robot_assembly_tipped.step`) in Fusion 360, SolidWorks, Onshape or FreeCAD.

---

<!-- appendix B: generated by cad/make_docs.py -->
## Appendix B — Full source code

Every file below is already in the project folder; this is a copy for reading, printing and the report. Paths are relative to the project folder.

### `firmware/cargo_robot/config.h`

```cpp
// Cargo robot - ESP32 wiring and fixed numbers.
//
// This file is the single source of truth for the wiring: the wire list in BUILD_GUIDE.md (week 3) and the
// schematics (cad/make_schematics.py reads this file) follow it. If you move a wire, change it here too.
// Numbers are the GPIO numbers printed on the ESP32 DevKit / screw-terminal breakout ("GPIO25" or just "25").
// Everything is wired straight to the ESP32: there is no I/O expander and no I2C.
#pragma once

// ---------------- ESP32 pins ----------------
// Drive: 2 x BTS7960 (IBT-2). RPWM = forward, LPWM = reverse. R_EN + L_EN of both come from PIN_MOTOR_EN.
constexpr int PIN_L_RPWM = 25;
constexpr int PIN_L_LPWM = 26;
constexpr int PIN_R_RPWM = 27;
constexpr int PIN_R_LPWM = 14;
constexpr int PIN_MOTOR_EN = 13;         // -> R_EN + L_EN of both drivers; 10k pull-down to GND: off at power-up
// Wheel encoders (JGB37-520 Hall, powered from 3.3 V). Input-only pins: each needs a 10k pull-up to 3.3 V.
constexpr int PIN_ENC_L_A = 34;
constexpr int PIN_ENC_L_B = 35;
constexpr int PIN_ENC_R_A = 36;          // marked "VP" on most boards
constexpr int PIN_ENC_R_B = 39;          // marked "VN"
// HC-SR04 ultrasonic sensor, looking straight ahead; ECHO through a 1k / 2k divider (5 V -> 3.3 V)
constexpr int PIN_SONAR_TRIG = 4;
constexpr int PIN_SONAR_ECHO = 23;
// Bed servos: signal wire only (their power comes from the 7.4 V servo buck converter)
constexpr int PIN_SERVO_FRONT = 2;
constexpr int PIN_SERVO_REAR = 12;       // GPIO12 must be LOW while the ESP32 starts: connect nothing else to it
// 3 IR line sensors (3-pin obstacle-avoidance modules, powered from 3.3 V), left to right
constexpr int LINE_N = 3;
constexpr int PIN_LINE[LINE_N] = {16, 17, 18};   // sensor 1 = left, 2 = middle, 3 = right
// Panel
constexpr int PIN_BUTTON = 19;           // <- start button to GND (internal pull-up)
constexpr int PIN_LED_GREEN = 21;        // -> green LED + 330 ohm to GND
constexpr int PIN_LED_RED = 22;          // -> red LED + 330 ohm to GND
constexpr int PIN_BUZZER = 32;           // -> active buzzer module, I/O pin
// Motor 12 V (after the E-stop) through a 10k / 2.2k divider: battery voltage, about 0 V = E-stop pressed
constexpr int PIN_BUS_SENSE = 33;
// Free: GPIO 0, 5 and 15. They are start-up (strapping) pins: only use them for something that can't pull them
// the wrong way while the ESP32 starts.

// ---------------- fixed numbers ----------------
constexpr float BUS_DIVIDER = (10.0f + 2.2f) / 2.2f;   // motor-bus volts per volt at PIN_BUS_SENSE
constexpr float BUS_ON_V = 7.0f;         // below this the motors have no power: E-stop pressed or battery out
constexpr int CONTROL_HZ = 100;          // wheel speed control and line following
constexpr int DRIVE_PWM_HZ = 20000;      // BTS7960 PWM, above hearing
constexpr int SONAR_PERIOD_MS = 60;      // one ultrasonic ping
constexpr float SONAR_MAX_CM = 400.0f;   // reported when nothing echoes back
```

### `firmware/cargo_robot/settings.h`

```cpp
// Tunable numbers, kept in the ESP32's flash so they survive a power cycle.
//
// Change them in the Serial Monitor:   set kp 0.03   (takes effect at once)   then   save   (keeps it).
// "list" prints them all, "defaults" goes back to the values below. Switches use 1 = yes, 0 = no.
#pragma once
#include <Arduino.h>

struct Settings {
  // ---- drive (week 3 bench tests, week 5) ----
  float wheel_radius = 0.0325f;   // m (65 mm wheels) - fine-tune with the 3 m straight-line test
  float wheel_sep = 0.356f;       // m, middle of the left tyre to middle of the right tyre - 5-turn spin test
  float ticks_per_rev = 3960;     // encoder counts per wheel turn (about 11 x 4 x 90) - measure with "enc"
  float max_wheel = 8.5f;         // rad/s: what the 110 RPM motors manage loaded (8.5 x 0.0325 m = 0.28 m/s)
  float kf = 0.09f;               // duty per rad/s - from "maxtest"
  float kp = 0.02f;
  float ki = 0.10f;
  float min_duty = 0.10f;         // duty that just starts a wheel - from "mintest"
  float accel = 0.4f;             // m/s^2: gentle, so the package does not slide
  float inv_l_motor = 0, inv_r_motor = 0, inv_l_enc = 0, inv_r_enc = 0;
  // ---- line following (week 6) ----
  float line_black = 1;           // 1 = a sensor output is HIGH over black tape, 0 = LOW over black
  float follow_speed = 0.20f;     // m/s, empty bed
  float k_line = 2.5f;            // rad/s of turning per unit of line error (-1..+1)
  float kd_line = 0.15f;
  float lost_s = 0.5f;            // stop if the line is gone this long
  float mark_min = 3;             // this many sensors on black at once = a station mark (all 3)
  float stop_mm = 98;             // drive on this far after a mark: sensor bar -> middle of the bed
  // ---- obstacles (week 7) ----
  float stop_cm = 25;             // something closer than this ahead: stop
  float slow_cm = 60;             // ... closer than this: slow down
  float clear_s = 1.0f;           // the way must be clear this long before driving on
  // ---- tilting bed (week 4, week 8) ----
  float tip_deg = 40;             // how far the bed tips to drop the package
  float tip_speed = 20;           // deg/s
  float f_flat_us = 1500, f_tip_us = 1944;   // front servo pulse, bed flat / bed at tip_deg - set with "servo"
  float r_flat_us = 1500, r_tip_us = 1056;   // rear servo: a mirror image, so it turns the other way
  float tip_hold_s = 2;           // seconds the bed stays tipped before it comes down
};

extern Settings cfg;

void settingsLoad();
bool settingsSave();
void settingsDefaults();
bool settingsSet(const char *name, float value);
bool settingsGet(const char *name, float &value);
void settingsList(Print &out);
```

### `firmware/cargo_robot/settings.cpp`

```cpp
#include "settings.h"

#include <Preferences.h>

Settings cfg;

namespace {

struct Entry {
  const char *name;   // also the key in flash (max 15 characters)
  float *value;
};

Entry entries[] = {
    {"wheel_radius", &cfg.wheel_radius}, {"wheel_sep", &cfg.wheel_sep}, {"ticks_per_rev", &cfg.ticks_per_rev},
    {"max_wheel", &cfg.max_wheel},       {"kf", &cfg.kf},               {"kp", &cfg.kp},
    {"ki", &cfg.ki},                     {"min_duty", &cfg.min_duty},   {"accel", &cfg.accel},
    {"inv_l_motor", &cfg.inv_l_motor},   {"inv_r_motor", &cfg.inv_r_motor},
    {"inv_l_enc", &cfg.inv_l_enc},       {"inv_r_enc", &cfg.inv_r_enc},
    {"line_black", &cfg.line_black},     {"follow_speed", &cfg.follow_speed}, {"k_line", &cfg.k_line},
    {"kd_line", &cfg.kd_line},           {"lost_s", &cfg.lost_s},       {"mark_min", &cfg.mark_min},
    {"stop_mm", &cfg.stop_mm},
    {"stop_cm", &cfg.stop_cm},           {"slow_cm", &cfg.slow_cm},     {"clear_s", &cfg.clear_s},
    {"tip_deg", &cfg.tip_deg},           {"tip_speed", &cfg.tip_speed},
    {"f_flat_us", &cfg.f_flat_us},       {"f_tip_us", &cfg.f_tip_us},
    {"r_flat_us", &cfg.r_flat_us},       {"r_tip_us", &cfg.r_tip_us},
    {"tip_hold_s", &cfg.tip_hold_s},
};

Entry *find(const char *name) {
  for (Entry &e : entries)
    if (strcmp(e.name, name) == 0) return &e;
  return nullptr;
}

}  // namespace

void settingsLoad() {
  Preferences p;
  p.begin("cargo", true);                 // read-only; missing keys keep their default
  for (Entry &e : entries) *e.value = p.getFloat(e.name, *e.value);
  p.end();
}

bool settingsSave() {
  Preferences p;
  if (!p.begin("cargo", false)) return false;
  bool ok = true;
  for (Entry &e : entries) ok &= p.putFloat(e.name, *e.value) == sizeof(float);
  p.end();
  return ok;
}

void settingsDefaults() {
  cfg = Settings();
  Preferences p;
  p.begin("cargo", false);
  p.clear();
  p.end();
}

bool settingsSet(const char *name, float value) {
  Entry *e = find(name);
  if (!e) return false;
  *e->value = value;
  return true;
}

bool settingsGet(const char *name, float &value) {
  Entry *e = find(name);
  if (!e) return false;
  value = *e->value;
  return true;
}

void settingsList(Print &out) {
  for (Entry &e : entries) out.printf("  %-14s %g\n", e.name, *e.value);
}
```

### `firmware/cargo_robot/hw.h`

```cpp
// Hardware drivers for the cargo robot's ESP32.
//
//   Motor       one BTS7960 channel (RPWM = forward, LPWM = reverse), hardware PWM
//   Encoder     quadrature counter in the ESP32's pulse-counter hardware (x4 decoding, no CPU load)
//   panel*()    the 3 line sensors, start button, LEDs, buzzer and motor enable: plain ESP32 pins
//   sonar::     the HC-SR04 in front; echo time measured with an interrupt
//   BedServo    one servo signal (50 Hz pulses); no pulses = the servo goes limp
//   busVolts()  motor 12 V bus, through the divider
//
// Pin numbers come from config.h.
#pragma once
#include <Arduino.h>
#include "driver/pulse_cnt.h"

class Motor {
 public:
  void begin(int pinFwd, int pinRev);
  void set(float duty);            // -1..1; 0 = brake
  bool invert = false;

 private:
  int fwd_ = -1, rev_ = -1;
};

class Encoder {
 public:
  bool begin(int pinA, int pinB);
  long count();                    // x4 counts since start, negative when turning backwards
  bool invert = false;

 private:
  pcnt_unit_handle_t unit_ = nullptr;
};

void panelBegin();                 // set the pins up: motors disabled, LEDs and buzzer off
uint8_t lineBits();                // bit i = output level of line sensor i + 1
bool buttonDown();                 // the start button is held
void panelOut(int pin, bool on);   // PIN_MOTOR_EN, PIN_BUZZER, PIN_LED_GREEN, PIN_LED_RED

namespace sonar {
void begin();
void update();                     // call often: pings every SONAR_PERIOD_MS
float cm();                        // distance straight ahead; SONAR_MAX_CM = nothing there
}  // namespace sonar

class BedServo {
 public:
  void begin(int pin);
  void pulse(float us);            // 500..2500 us
  void off();
  bool on() const { return on_; }

 private:
  int pin_ = -1;
  bool on_ = false;
};

float busVolts();
```

### `firmware/cargo_robot/hw.cpp`

```cpp
#include "hw.h"

#include "config.h"

// ---------------- Motor ----------------
void Motor::begin(int pinFwd, int pinRev) {
  fwd_ = pinFwd;
  rev_ = pinRev;
  ledcAttach(fwd_, DRIVE_PWM_HZ, 10);
  ledcAttach(rev_, DRIVE_PWM_HZ, 10);
  set(0);
}

void Motor::set(float duty) {
  duty = constrain(duty, -1.0f, 1.0f);
  if (invert) duty = -duty;
  uint32_t d = (uint32_t)(fabsf(duty) * 1023.0f + 0.5f);
  ledcWrite(duty >= 0 ? rev_ : fwd_, 0);          // both inputs low = both low-side switches on = brake
  ledcWrite(duty >= 0 ? fwd_ : rev_, d);
}

// ---------------- Encoder ----------------
bool Encoder::begin(int pinA, int pinB) {
  pcnt_unit_config_t unit = {};
  unit.low_limit = -30000;
  unit.high_limit = 30000;
  unit.flags.accum_count = 1;                     // keep counting past the limits
  if (pcnt_new_unit(&unit, &unit_) != ESP_OK) return false;
  pcnt_glitch_filter_config_t filter = {};
  filter.max_glitch_ns = 1000;                    // ignore spikes shorter than 1 us
  pcnt_unit_set_glitch_filter(unit_, &filter);
  pcnt_chan_config_t a = {}, b = {};
  a.edge_gpio_num = pinA;
  a.level_gpio_num = pinB;
  b.edge_gpio_num = pinB;
  b.level_gpio_num = pinA;
  pcnt_channel_handle_t chA, chB;
  pcnt_new_channel(unit_, &a, &chA);
  pcnt_new_channel(unit_, &b, &chB);
  // x4 decoding: count every edge of A and of B, direction from the level of the other channel
  pcnt_channel_set_edge_action(chA, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(chA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_channel_set_edge_action(chB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(chB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_unit_add_watch_point(unit_, unit.high_limit);
  pcnt_unit_add_watch_point(unit_, unit.low_limit);
  pcnt_unit_enable(unit_);
  pcnt_unit_clear_count(unit_);
  pcnt_unit_start(unit_);
  return true;
}

long Encoder::count() {
  int c = 0;
  if (unit_) pcnt_unit_get_count(unit_, &c);
  return invert ? -c : c;
}

// ---------------- panel and line sensors (plain ESP32 pins) ----------------
void panelBegin() {
  for (int pin : {PIN_MOTOR_EN, PIN_BUZZER, PIN_LED_GREEN, PIN_LED_RED}) {
    digitalWrite(pin, LOW);                       // motors disabled, buzzer and LEDs off
    pinMode(pin, OUTPUT);
  }
  for (int pin : PIN_LINE) pinMode(pin, INPUT);   // the modules drive their outputs
  pinMode(PIN_BUTTON, INPUT_PULLUP);
}

uint8_t lineBits() {
  uint8_t bits = 0;
  for (int i = 0; i < LINE_N; i++)
    if (digitalRead(PIN_LINE[i])) bits |= 1 << i;
  return bits;
}

bool buttonDown() { return digitalRead(PIN_BUTTON) == LOW; }   // pressed = pulled to GND

void panelOut(int pin, bool on) { digitalWrite(pin, on ? HIGH : LOW); }

// ---------------- sonar ----------------
namespace sonar {
namespace {
volatile uint32_t riseUs = 0, widthUs = 0;
volatile bool fresh = false;
float lastCm = SONAR_MAX_CM;
uint32_t lastPing = 0;

void IRAM_ATTR onEcho() {
  uint32_t now = micros();
  if (digitalRead(PIN_SONAR_ECHO)) {
    riseUs = now;
  } else {
    widthUs = now - riseUs;
    fresh = true;
  }
}
}  // namespace

void begin() {
  pinMode(PIN_SONAR_TRIG, OUTPUT);
  digitalWrite(PIN_SONAR_TRIG, LOW);
  pinMode(PIN_SONAR_ECHO, INPUT);
  attachInterrupt(PIN_SONAR_ECHO, onEcho, CHANGE);
}

void update() {
  if (millis() - lastPing < SONAR_PERIOD_MS) return;
  lastPing = millis();
  lastCm = fresh ? min(widthUs / 58.0f, SONAR_MAX_CM) : SONAR_MAX_CM;   // result of the previous ping
  fresh = false;
  digitalWrite(PIN_SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SONAR_TRIG, LOW);
}

float cm() { return lastCm; }
}  // namespace sonar

// ---------------- BedServo ----------------
void BedServo::begin(int pin) {
  pin_ = pin;
  ledcAttach(pin_, 50, 16);
  off();
}

void BedServo::pulse(float us) {
  ledcWrite(pin_, (uint32_t)(constrain(us, 500.0f, 2500.0f) * 65536.0f / 20000.0f));
  on_ = true;
}

void BedServo::off() {
  ledcWrite(pin_, 0);
  on_ = false;
}

// ---------------- motor bus voltage ----------------
float busVolts() { return analogReadMilliVolts(PIN_BUS_SENSE) / 1000.0f * BUS_DIVIDER; }
```

### `firmware/cargo_robot/control.h`

```cpp
// Maths used by the main program (no hardware in here).
#pragma once
#include <Arduino.h>

// Wheel speed controller: feed-forward + PI. Target and measurement in rad/s, output = motor duty -1..1.
//   duty = sign(target) * (min_duty + kf * |target|) + kp * error + integral
struct WheelPID {
  float integral = 0;
  float update(float target, float measured, float dt, float kf, float kp, float ki, float minDuty);
  void reset() { integral = 0; }
};

struct LowPass {
  float alpha, value = 0;
  explicit LowPass(float a) : alpha(a) {}
  float update(float x) { return value += alpha * (x - value); }
};

// Distance travelled (m) and heading change (rad, + = turned left) from the distance each wheel rolled.
struct Odometry {
  float distance = 0, heading = 0;
  void update(float dLeft, float dRight, float wheelSep);
};

// Robot speed (v m/s forward, w rad/s turning left) -> wheel speeds (rad/s), scaled down together if too fast.
void twistToWheels(float v, float w, float wheelSep, float wheelRadius, float maxWheel, float &left, float &right);

// 1.0 when the way ahead is clear (beyond slowCm) -> 0.0 at stopCm and closer.
float obstacleFactor(float cm, float stopCm, float slowCm);

// The line sensors as a position: +1 = line under the leftmost sensor, -1 = rightmost, 0 = centred.
struct Line {
  int count = 0;                   // sensors on black
  float pos = 0;
  uint8_t black = 0;               // bit i = sensor i + 1 is on black
};
Line readLine(uint8_t bits, int n, bool blackIsHigh);   // n sensors, bit i = sensor i + 1
```

### `firmware/cargo_robot/control.cpp`

```cpp
#include "control.h"

float WheelPID::update(float target, float measured, float dt, float kf, float kp, float ki, float minDuty) {
  if (fabsf(target) < 1e-3f) {
    reset();
    return 0;
  }
  float error = target - measured;
  integral = constrain(integral + ki * error * dt, -0.3f, 0.3f);
  float ff = copysignf(minDuty + kf * fabsf(target), target);
  return constrain(ff + kp * error + integral, -1.0f, 1.0f);
}

void Odometry::update(float dLeft, float dRight, float wheelSep) {
  distance += (dLeft + dRight) / 2;
  heading += (dRight - dLeft) / wheelSep;
}

void twistToWheels(float v, float w, float wheelSep, float wheelRadius, float maxWheel, float &left, float &right) {
  left = (v - w * wheelSep / 2) / wheelRadius;
  right = (v + w * wheelSep / 2) / wheelRadius;
  float biggest = max(fabsf(left), fabsf(right));
  if (biggest > maxWheel) {                       // keep the curve, just slower
    left *= maxWheel / biggest;
    right *= maxWheel / biggest;
  }
}

float obstacleFactor(float cm, float stopCm, float slowCm) {
  return constrain((cm - stopCm) / (slowCm - stopCm), 0.0f, 1.0f);
}

Line readLine(uint8_t bits, int n, bool blackIsHigh) {
  Line l;
  l.black = (blackIsHigh ? bits : (uint8_t)~bits) & ((1 << n) - 1);
  float mid = (n - 1) / 2.0f, sum = 0;
  for (int i = 0; i < n; i++) {
    if (l.black & (1 << i)) {
      sum += (mid - i) / mid;                     // sensor 1 -> +1 ... sensor n -> -1
      l.count++;
    }
  }
  if (l.count) l.pos = sum / l.count;
  return l;
}
```

### `firmware/cargo_robot/robot.h`

```cpp
// The robot's parts and live readings, shared by the main program and the serial console.
#pragma once
#include "control.h"
#include "hw.h"

extern Motor motorL, motorR;
extern Encoder encL, encR;
extern BedServo servoFront, servoRear;
extern bool manualMotors;          // true while a bench test drives the motors directly (no wheel control)

// Updated by robotUpdate().
struct Readings {
  float busV = 0;                  // motor 12 V bus; below BUS_ON_V = E-stop pressed
  Line line;
  float sonarCm = 0;               // ultrasonic, straight ahead
  float wheelL = 0, wheelR = 0;    // rad/s
  Odometry odom;
  float bedDeg = 0;                // where the servos have been told to hold the bed
};
extern Readings rd;

enum class Light { Off, On, Slow, Fast };

void robotBegin();                 // start every part (called once from setup)
void robotUpdate();                // call as often as possible: sensors, wheel control, bed, lights, buzzer
void driveTarget(float v, float w);   // wanted speed: m/s forward, rad/s turning left (ramped by "accel")
void driveStop();                  // stop now
void motorsEnable(bool on);
void bedMove(float deg);           // move the bed smoothly (tip_speed) to deg; 0 = flat
bool bedMoving();
void bedRelax();                   // servos off: the bed rests flat on its two posts
void lights(Light green, Light red);
void beep(int ms);
void applyInversions();            // after changing inv_* settings
```

### `firmware/cargo_robot/robot.cpp`

```cpp
#include "robot.h"

#include "config.h"
#include "settings.h"

Motor motorL, motorR;
Encoder encL, encR;
BedServo servoFront, servoRear;
bool manualMotors = false;
Readings rd;

namespace {
WheelPID pidL, pidR;
LowPass speedL(0.5f), speedR(0.5f);
long lastCountL = 0, lastCountR = 0;
uint32_t lastTick = 0, beepUntil = 0;
float targetV = 0, targetW = 0, rampV = 0;
float bedTarget = 0;
bool motorsOn = false;
Light greenMode = Light::Off, redMode = Light::Off;

bool lit(Light mode) {
  switch (mode) {
    case Light::On: return true;
    case Light::Slow: return millis() % 1000 < 150;
    case Light::Fast: return millis() % 300 < 150;
    default: return false;
  }
}

void writeServos() {
  float k = cfg.tip_deg != 0 ? rd.bedDeg / cfg.tip_deg : 0;
  servoFront.pulse(cfg.f_flat_us + (cfg.f_tip_us - cfg.f_flat_us) * k);
  servoRear.pulse(cfg.r_flat_us + (cfg.r_tip_us - cfg.r_flat_us) * k);
}

void controlTick(float dt) {
  // wheels: encoder counts -> speeds and odometry
  long cl = encL.count(), cr = encR.count();
  float radPerTick = 2 * PI / cfg.ticks_per_rev;
  float dl = (cl - lastCountL) * radPerTick, dr = (cr - lastCountR) * radPerTick;
  lastCountL = cl;
  lastCountR = cr;
  rd.wheelL = speedL.update(dl / dt);
  rd.wheelR = speedR.update(dr / dt);
  rd.odom.update(dl * cfg.wheel_radius, dr * cfg.wheel_radius, cfg.wheel_sep);

  // speed ramp: gentle starts and stops, so the package does not slide
  float step = cfg.accel * dt;
  rampV += constrain(targetV - rampV, -step, step);
  if (!manualMotors) {
    float left = 0, right = 0;
    if (motorsOn) twistToWheels(rampV, targetW, cfg.wheel_sep, cfg.wheel_radius, cfg.max_wheel, left, right);
    motorL.set(pidL.update(left, rd.wheelL, dt, cfg.kf, cfg.kp, cfg.ki, cfg.min_duty));
    motorR.set(pidR.update(right, rd.wheelR, dt, cfg.kf, cfg.kp, cfg.ki, cfg.min_duty));
  }

  // line sensors
  rd.line = readLine(lineBits(), LINE_N, cfg.line_black != 0);

  // bed: move towards its target at tip_speed
  if (servoFront.on()) {
    float s = cfg.tip_speed * dt;
    rd.bedDeg += constrain(bedTarget - rd.bedDeg, -s, s);
    writeServos();
  }
}
}  // namespace

void robotBegin() {
  panelBegin();
  motorL.begin(PIN_L_RPWM, PIN_L_LPWM);
  motorR.begin(PIN_R_RPWM, PIN_R_LPWM);
  if (!encL.begin(PIN_ENC_L_A, PIN_ENC_L_B) || !encR.begin(PIN_ENC_R_A, PIN_ENC_R_B))
    Serial.println("!! encoder counter set-up failed");
  applyInversions();
  sonar::begin();
  servoFront.begin(PIN_SERVO_FRONT);
  servoRear.begin(PIN_SERVO_REAR);
  motorsEnable(false);
  lastTick = micros();
}

void applyInversions() {
  motorL.invert = cfg.inv_l_motor != 0;
  motorR.invert = cfg.inv_r_motor != 0;
  encL.invert = cfg.inv_l_enc != 0;
  encR.invert = cfg.inv_r_enc != 0;
  lastCountL = encL.count();
  lastCountR = encR.count();
}

void robotUpdate() {
  rd.busV = busVolts();
  sonar::update();
  rd.sonarCm = sonar::cm();

  uint32_t now = micros();
  if (now - lastTick >= 1000000 / CONTROL_HZ) {
    float dt = (now - lastTick) * 1e-6f;
    lastTick = now;
    controlTick(dt);
  }

  panelOut(PIN_LED_GREEN, lit(greenMode));
  panelOut(PIN_LED_RED, lit(redMode));
  panelOut(PIN_BUZZER, (int32_t)(beepUntil - millis()) > 0);
}

void driveTarget(float v, float w) {
  targetV = v;
  targetW = w;
}

void driveStop() {
  targetV = targetW = rampV = 0;
  pidL.reset();
  pidR.reset();
  motorL.set(0);
  motorR.set(0);
}

void motorsEnable(bool on) {
  if (!on) driveStop();
  motorsOn = on;
  panelOut(PIN_MOTOR_EN, on);
}

void bedMove(float deg) {
  if (!servoFront.on()) {                         // waking up: the bed is resting flat on its posts
    rd.bedDeg = 0;
    writeServos();
  }
  bedTarget = constrain(deg, 0.0f, 45.0f);
}

bool bedMoving() { return servoFront.on() && fabsf(rd.bedDeg - bedTarget) > 0.1f; }

void bedRelax() {
  servoFront.off();
  servoRear.off();
  rd.bedDeg = bedTarget = 0;
}

void lights(Light green, Light red) {
  greenMode = green;
  redMode = red;
}

void beep(int ms) { beepUntil = millis() + ms; }
```

### `firmware/cargo_robot/console.h`

```cpp
// Serial-Monitor commands (115200 baud, line ending "Newline"): bench tests, calibration and settings.
// Type "help" for the list.
#pragma once

void consolePoll();                // call from loop(): runs a command when a whole line has arrived
void startMission(int deliveries); // defined in cargo_robot.ino
void stopMission();
const char *stateName();
```

### `firmware/cargo_robot/console.cpp`

```cpp
#include "console.h"

#include "config.h"
#include "robot.h"
#include "settings.h"

namespace {

String lineBuf;

// ---------------- helpers for the tests ----------------
bool keyPressed() {
  if (!Serial.available()) return false;
  while (Serial.available()) Serial.read();
  return true;
}

void waitEnter(const char *prompt) {
  Serial.print(prompt);
  Serial.println("  (press Enter)");
  while (!Serial.available()) robotUpdate();
  while (Serial.available()) {
    Serial.read();
    delay(2);
  }
}

// Repeat print() 5 times a second until a key is pressed.
template <typename F>
void monitor(F print, int periodMs = 200) {
  Serial.println("(press Enter to stop)");
  uint32_t last = 0;
  while (!keyPressed()) {
    robotUpdate();
    if (millis() - last >= (uint32_t)periodMs) {
      last = millis();
      print();
    }
  }
}

void runFor(float seconds) {
  uint32_t t0 = millis();
  while (millis() - t0 < seconds * 1000) robotUpdate();
}

const char *SIDES[2] = {"left", "right"};
Motor *motors[2] = {&motorL, &motorR};
Encoder *encs[2] = {&encL, &encR};

void powerCheck() {
  robotUpdate();
  Serial.printf("motor power: %s (%.1f V)\n", rd.busV > BUS_ON_V ? "ON" : "OFF - E-stop pressed or battery out",
                rd.busV);
}

// ---------------- motor and encoder tests (wheels in the air!) ----------------
void testEncoders() {
  Serial.println("Turn each wheel FORWARD by hand: its count must go UP.");
  monitor([] {
    Serial.printf("left %8ld (%+6.2f turns)    right %8ld (%+6.2f turns)\n", encL.count(),
                  encL.count() / cfg.ticks_per_rev, encR.count(), encR.count() / cfg.ticks_per_rev);
  });
}

void testMotors(float duty) {
  powerCheck();
  manualMotors = true;
  motorsEnable(true);
  for (int s = 0; s < 2; s++) {
    for (int dir : {1, -1}) {
      Serial.printf("\n[%s wheel %s]", SIDES[s], dir > 0 ? "FORWARD" : "BACKWARD");
      waitEnter(" watch the wheel");
      long c0 = encs[s]->count();
      motors[s]->set(dir * duty);
      runFor(2.0f);
      motors[s]->set(0);
      runFor(0.5f);
      long delta = encs[s]->count() - c0;
      Serial.printf("  encoder change: %+ld counts (%+.2f wheel turns)\n", delta, delta / cfg.ticks_per_rev);
      if (delta == 0)
        Serial.println("  !! the encoder did not count: check its 3.3 V, GND, A/B wires and the 10k pull-ups");
      else if ((delta > 0) != (dir > 0))
        Serial.printf("  !! count has the wrong sign: type   set inv_%c_enc %d   then run motortest again\n",
                      SIDES[s][0], (s == 0 ? cfg.inv_l_enc : cfg.inv_r_enc) == 0 ? 1 : 0);
    }
  }
  motorsEnable(false);
  manualMotors = false;
  Serial.println("\nBy eye: on FORWARD, did each wheel turn the way that drives the robot FORWARD?");
  Serial.println("If not: set inv_l_motor 1 (or inv_r_motor 1), then run motortest again. Type save when done.");
}

void testMaxSpeed() {
  powerCheck();
  manualMotors = true;
  motorsEnable(true);
  float slowest = 1e9;
  for (int s = 0; s < 2; s++) {
    Serial.printf("\n[%s wheel 100 %%]", SIDES[s]);
    waitEnter("");
    motors[s]->set(1.0f);
    runFor(1.0f);                                 // let it spin up
    long c0 = encs[s]->count();
    runFor(2.0f);
    float w = (encs[s]->count() - c0) / cfg.ticks_per_rev * 2 * PI / 2.0f;
    motors[s]->set(0);
    runFor(0.5f);
    Serial.printf("  top speed %.1f rad/s = %.0f rpm\n", w, w * 30 / PI);
    slowest = min(slowest, fabsf(w));
  }
  motorsEnable(false);
  manualMotors = false;
  if (slowest > 0.1f) Serial.printf("\nType:   set kf %.3f   then   save\n", 1.0f / slowest);
}

void testMinDuty() {
  powerCheck();
  manualMotors = true;
  motorsEnable(true);
  float found = 0;
  for (int s = 0; s < 2; s++) {
    Serial.printf("\n[%s wheel slow ramp]", SIDES[s]);
    waitEnter("");
    float duty = 0;
    long c0 = encs[s]->count();
    while (duty < 0.5f && labs(encs[s]->count() - c0) < 20) {
      duty += 0.01f;
      motors[s]->set(duty);
      runFor(0.3f);
    }
    motors[s]->set(0);
    Serial.printf("  started moving at %.2f\n", duty);
    found = max(found, duty);
  }
  motorsEnable(false);
  manualMotors = false;
  Serial.printf("\nType:   set min_duty %.2f   then   save\n", 0.9f * found);
}

void testOneMotor(char side, float duty, float seconds) {
  powerCheck();
  manualMotors = true;
  motorsEnable(true);
  Motor *m = side == 'r' ? &motorR : &motorL;
  m->set(duty);
  uint32_t t0 = millis();
  while (millis() - t0 < seconds * 1000 && !keyPressed()) robotUpdate();
  m->set(0);
  motorsEnable(false);
  manualMotors = false;
}

void testDrive(float v, float w, float seconds) {
  powerCheck();
  Odometry start = rd.odom;
  motorsEnable(true);
  driveTarget(v, w);
  uint32_t t0 = millis(), last = 0;
  Serial.println("(press Enter to stop early)");
  while (millis() - t0 < seconds * 1000 && !keyPressed()) {
    robotUpdate();
    if (millis() - last >= 250) {
      last = millis();
      Serial.printf("wheels L %+5.2f R %+5.2f rad/s\n", rd.wheelL, rd.wheelR);
    }
  }
  driveTarget(0, 0);
  runFor(1.5f);                                   // ramp down
  motorsEnable(false);
  Serial.printf("odometry: %.3f m forward, turned %.1f deg\n", rd.odom.distance - start.distance,
                (rd.odom.heading - start.heading) * 180 / PI);
}

// ---------------- sensors ----------------
void testLine() {
  Serial.println("sensor 1 (left) .. 3 (right): # = black");
  monitor([] {
    char s[LINE_N + 1];
    for (int i = 0; i < LINE_N; i++) s[i] = rd.line.black & (1 << i) ? '#' : '.';
    s[LINE_N] = 0;
    Serial.printf("%s   on black %d   position %+.2f%s\n", s, rd.line.count, rd.line.pos,
                  rd.line.count >= cfg.mark_min ? "   STATION MARK" : "");
  }, 100);
}

void testSonar() {
  monitor([] {
    Serial.printf("ahead %5.0f cm   (%.0f = nothing there)\n", rd.sonarCm, SONAR_MAX_CM);
  });
}

// ---------------- bed ----------------
void testServo(char which, const String &arg) {
  BedServo *s = which == 'r' ? &servoRear : &servoFront;
  if (arg == "off") {
    s->off();
    return;
  }
  powerCheck();
  s->pulse(arg.toFloat());
  Serial.printf("%s servo at %.0f us. Change it in small steps (10-20 us); \"servo %c off\" to release.\n",
                which == 'r' ? "rear" : "front", arg.toFloat(), which);
}

void testTip() {
  powerCheck();
  Serial.printf("tipping to %.0f deg ...\n", cfg.tip_deg);
  uint32_t t0 = millis();
  bedMove(cfg.tip_deg);
  while (bedMoving()) robotUpdate();
  Serial.printf("tipped after %.1f s, holding %.1f s\n", (millis() - t0) / 1000.0f, cfg.tip_hold_s);
  runFor(cfg.tip_hold_s);
  bedMove(0);
  while (bedMoving()) robotUpdate();
  runFor(0.5f);
  bedRelax();
  Serial.println("bed flat again - did the package slide off?");
}

// ---------------- status ----------------
void status() {
  char bits[LINE_N + 1];
  for (int i = 0; i < LINE_N; i++) bits[i] = rd.line.black & (1 << i) ? '#' : '.';
  bits[LINE_N] = 0;
  Serial.printf("%-7s | %4.1f V | line %s | ahead %3.0f cm | bed %4.1f deg\n", stateName(), rd.busV, bits,
                rd.sonarCm, rd.bedDeg);
}

void help() {
  Serial.println(R"(
MISSION    run [n]          start (n deliveries, 0 = until stopped); the start button does the same
           stop             stop everything
           status | watch   one line of readings | the same, twice a second
DRIVE      enc              encoder counts (turn the wheels by hand)          WHEELS IN THE AIR for:
           motortest        each wheel forward/back 30 %, checks directions
           mintest | maxtest   measure min_duty | kf
           motor l|r <duty> [s]   one motor open loop, duty -1..1
           drive <v> <w> [s]      closed loop: v m/s, w rad/s, default 2 s; prints odometry
SENSORS    line | sonar     live line sensors | ultrasonic
BED        servo f|r <us>   one servo to a raw pulse (calibration); servo f|r off
           bed <deg>        both servos smoothly to deg (0 = flat); bed off = release
           tip              tip, hold, lower
SETTINGS   list | get <name> | set <name> <value> | save | defaults)");
}

void settingsCommand(const String &cmd, const String &a1, const String &a2) {
  if (cmd == "list") {
    settingsList(Serial);
  } else if (cmd == "get") {
    float v;
    if (settingsGet(a1.c_str(), v)) Serial.printf("%s = %g\n", a1.c_str(), v);
    else Serial.println("unknown setting (type list)");
  } else if (cmd == "set") {
    if (!settingsSet(a1.c_str(), a2.toFloat())) {
      Serial.println("unknown setting (type list)");
      return;
    }
    applyInversions();
    Serial.printf("%s = %g (in use now; type save to keep it after power-off)\n", a1.c_str(), a2.toFloat());
  } else if (cmd == "save") {
    Serial.println(settingsSave() ? "saved" : "!! save failed");
  } else if (cmd == "defaults") {
    settingsDefaults();
    applyInversions();
    Serial.println("back to the defaults (also in flash)");
  }
}

void runCommand(String line) {
  line.trim();
  line.toLowerCase();
  if (!line.length()) return;
  String w[4];
  int n = 0;
  for (int start = 0; n < 4 && start < (int)line.length();) {
    int sp = line.indexOf(' ', start);
    if (sp < 0) sp = line.length();
    if (sp > start) w[n++] = line.substring(start, sp);
    start = sp + 1;
  }
  const String &c = w[0];
  if (c == "stop") return stopMission();
  if (c == "status") return status();
  if (c == "help") return help();
  if (strcmp(stateName(), "IDLE") != 0) {
    Serial.println("busy: type stop first");
    return;
  }
  if (c == "run") return startMission(w[1].toInt());
  if (c == "watch") return monitor(status, 500);
  if (c == "enc") return testEncoders();
  if (c == "motortest") return testMotors(w[1].length() ? w[1].toFloat() : 0.3f);
  if (c == "mintest") return testMinDuty();
  if (c == "maxtest") return testMaxSpeed();
  if (c == "motor") return testOneMotor(w[1][0], w[2].toFloat(), w[3].length() ? w[3].toFloat() : 2.0f);
  if (c == "drive") return testDrive(w[1].toFloat(), w[2].toFloat(), w[3].length() ? w[3].toFloat() : 2.0f);
  if (c == "line") return testLine();
  if (c == "sonar") return testSonar();
  if (c == "servo") return testServo(w[1][0], w[2]);
  if (c == "bed") {
    if (w[1] == "off") return bedRelax();
    bedMove(w[1].toFloat());
    while (bedMoving()) robotUpdate();
    return;
  }
  if (c == "tip") return testTip();
  if (c == "list" || c == "get" || c == "set" || c == "save" || c == "defaults") return settingsCommand(c, w[1], w[2]);
  Serial.println("unknown command - type help");
}

}  // namespace

void consolePoll() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (lineBuf.length()) {
        Serial.printf("> %s\n", lineBuf.c_str());
        runCommand(lineBuf);
      }
      lineBuf = "";
    } else if (lineBuf.length() < 80) {
      lineBuf += ch;
    }
  }
}
```

### `firmware/cargo_robot/cargo_robot.ino`

```cpp
// Cargo robot - main program for the ESP32 (Arduino IDE 2, board "ESP32 Dev Module").
//
// The robot follows a black tape line. Stations are short strips of tape across the line ("station marks"):
// the pickup and the drop-off (on the robot's RIGHT). It carries one package on its tilting bed. 3 IR sensors
// see the line; all 3 on black = a station mark.
//
//   LOAD     at the pickup: wait for the start button (put the package on the bed, then press it)
//   FOLLOW   follow the line to the next station mark: slow or stop for obstacles
//   DOCK     drive on stop_mm so the middle of the bed is level with the mark, then stop
//   SETTLE   at the drop-off: wait half a second for the robot to stand still
//   TIP      tip the bed to tip_deg (at tip_speed); the package slides off to the right
//   HOLD     keep it tipped for tip_hold_s
//   LOWER    lower the bed, switch the servos off and FOLLOW on to the pickup
// Always:  E-stop pressed -> everything stops.  Line lost -> ALARM: red LED + beeps; fix it, then press the
//          start button to carry on.
// The robot does not weigh anything: keep packages within the limit in BUILD_GUIDE.md.
//
// Start a mission with the start button or "run" in the Serial Monitor (115200 baud, Newline). The robot must
// stand at the pickup with the first package on its bed: it sets off at once.
// "help" lists the test and calibration commands. Wiring: config.h. Settings: settings.h.
#include "config.h"
#include "console.h"
#include "robot.h"
#include "settings.h"

enum State { IDLE, LOAD, FOLLOW, DOCK, SETTLE, TIP, HOLD, LOWER, ALARM, ESTOP };
const char *NAMES[] = {"IDLE", "LOAD", "FOLLOW", "DOCK", "SETTLE", "TIP", "HOLD", "LOWER", "ALARM", "ESTOP"};

State state = IDLE, resumeTo = IDLE;    // resumeTo: where ALARM goes back to when the button is pressed
bool toDropoff = true;                  // the next station mark is the drop-off (else the pickup)
int deliveriesLeft = 0;                 // 0 = until stopped
uint32_t since = 0, condSince = 0;      // when the state started / when the condition we wait for became true
float dockStart = 0;
bool buttonWas = false;
bool relaxWhenFlat = false;             // after "stop": lower the bed, then switch the servos off

const char *stateName() { return NAMES[state]; }

void go(State s) {
  state = s;
  since = millis();
  condSince = 0;
  Serial.printf("[%s]\n", NAMES[s]);
}

// true once cond has held for seconds without a break
bool heldFor(bool cond, float seconds) {
  if (!cond) {
    condSince = 0;
    return false;
  }
  if (!condSince) condSince = millis();
  return millis() - condSince >= seconds * 1000;
}

void alarm(const char *why, State retry) {
  driveStop();
  motorsEnable(false);
  resumeTo = retry;
  Serial.printf("!! ALARM: %s - fix it, then press the start button\n", why);
  go(ALARM);
}

void depart();

void startMission(int deliveries) {
  if (rd.busV < BUS_ON_V) {
    Serial.println("motor power is off: release the E-stop first");
    return;
  }
  deliveriesLeft = deliveries;
  bedRelax();
  Serial.println(deliveries ? "mission started" : "mission started (until stopped)");
  depart();
}

void stopMission() {
  driveStop();
  motorsEnable(false);
  if (servoFront.on()) {
    bedMove(0);
    relaxWhenFlat = true;
  }
  go(IDLE);
}

bool buttonPressed() {                  // one press = one event
  bool now = buttonDown();
  bool edge = now && !buttonWas;
  buttonWas = now;
  return edge;
}

// ---------------- line following ----------------
bool markArmed = true;
int markTicks = 0;
float lastPos = 0, lastPosTime = 0;
uint32_t lostSince = 0, obstacleSince = 0;

// Steer along the line at the speed allowed by the obstacles. Returns true when a station mark
// has just been crossed (mark_min = all 3 sensors on black, for 2 readings in a row).
bool follow() {
  const Line &l = rd.line;
  if (l.count == 0) {
    if (!lostSince) lostSince = millis();
    if (millis() - lostSince > cfg.lost_s * 1000) {
      alarm("line lost", state);
      return false;
    }
  } else {
    lostSince = 0;
  }

  float clear = obstacleFactor(rd.sonarCm, cfg.stop_cm, cfg.slow_cm);
  if (clear == 0) {                     // something in front: stop and ask for it to be moved
    if (!obstacleSince) Serial.println("obstacle - please remove it");
    obstacleSince = millis();
    lights(Light::On, Light::Fast);
    if (millis() % 1000 < 30) beep(120);
  } else if (obstacleSince && millis() - obstacleSince < cfg.clear_s * 1000) {
    clear = 0;                          // wait until the way has been clear for clear_s
  } else {
    obstacleSince = 0;
    lights(Light::On, Light::Off);
  }

  float pos = l.count ? l.pos : lastPos;  // line just lost: keep turning the way it went
  float now = millis() / 1000.0f;
  float dpos = (now > lastPosTime && lastPosTime > 0) ? (pos - lastPos) / (now - lastPosTime) : 0;
  lastPos = pos;
  lastPosTime = now;
  float v = cfg.follow_speed * clear;
  float w = (cfg.k_line * pos + cfg.kd_line * dpos) * (v / cfg.follow_speed);
  driveTarget(v, w);

  if (l.count >= cfg.mark_min) {
    if (markArmed && ++markTicks >= 2) {
      markArmed = false;
      return true;
    }
  } else {
    markTicks = 0;
    if (l.count <= 1) markArmed = true;     // back on the plain line: the next mark can count
  }
  return false;
}

// ---------------- the mission ----------------
void depart() {                         // leave the pickup with a package on the bed
  Serial.println("off to the drop-off");
  beep(200);
  toDropoff = true;
  motorsEnable(true);
  markArmed = true;
  go(FOLLOW);
}

void step() {
  bool power = rd.busV > BUS_ON_V;
  if (!power && state != ESTOP && state != IDLE) {
    driveStop();
    motorsEnable(false);
    bedRelax();                         // the servos have lost power too; never jump back to an old angle
    Serial.println("E-STOP: motor power is off");
    lights(Light::Off, Light::On);
    go(ESTOP);
    return;
  }
  bool button = buttonPressed();

  switch (state) {
    case IDLE:
      lights(Light::Slow, power ? Light::Off : Light::On);
      if (relaxWhenFlat && !bedMoving()) {
        bedRelax();
        relaxWhenFlat = false;
      }
      if (button) startMission(0);
      break;

    case ESTOP:
      if (heldFor(power, 1.0f)) {
        Serial.println("motor power is back - press the start button or type run");
        go(IDLE);
      }
      break;

    case LOAD:
      lights(Light::On, Light::Off);
      motorsEnable(false);
      if (button) depart();
      break;

    case FOLLOW:
      if (follow()) {
        dockStart = rd.odom.distance;
        go(DOCK);
      }
      break;

    case DOCK:
      follow();
      if (state == DOCK && rd.odom.distance - dockStart >= cfg.stop_mm / 1000) {
        driveStop();
        motorsEnable(false);
        if (toDropoff) {
          go(SETTLE);
        } else if (deliveriesLeft && --deliveriesLeft == 0) {
          Serial.println("all deliveries done");
          go(IDLE);
        } else {
          Serial.println("at the pickup: put the next package on the bed, then press the start button");
          go(LOAD);
        }
      }
      break;

    case SETTLE:
      if (millis() - since < 500) break;  // let the robot stand still
      bedMove(cfg.tip_deg);
      lights(Light::Fast, Light::Off);
      go(TIP);
      break;

    case TIP:
      if (!bedMoving()) go(HOLD);
      break;

    case HOLD:
      if (millis() - since >= cfg.tip_hold_s * 1000) {
        bedMove(0);
        go(LOWER);
      }
      break;

    case LOWER:
      if (bedMoving()) break;
      bedRelax();                       // resting flat on its posts now
      Serial.println("package delivered");
      toDropoff = false;
      motorsEnable(true);
      markArmed = true;
      lights(Light::On, Light::Off);
      go(FOLLOW);
      break;

    case ALARM:
      lights(Light::Off, Light::On);
      if (millis() % 2000 < 30) beep(150);
      if (button) {
        Serial.println("carrying on");
        if (resumeTo == FOLLOW || resumeTo == DOCK) motorsEnable(true);
        go(resumeTo);
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nCargo robot (ESP32) - type help");
  settingsLoad();
  robotBegin();
  robotUpdate();
  Serial.printf("motor power %s (%.1f V)\n", rd.busV > BUS_ON_V ? "ON" : "OFF", rd.busV);
  go(IDLE);
}

void loop() {
  robotUpdate();
  consolePoll();
  step();
}
```
