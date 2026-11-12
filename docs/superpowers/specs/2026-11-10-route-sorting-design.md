# Design: the robot takes packages from the conveyor and delivers them by company route

Date: 2026-11-10. Status: draft, not yet approved. **The software parts are on hold** until the user has
new information (section 13).

## 1. What changes, and why

Until now the robot carried a package from a hand-loaded pickup to a conveyor and tipped it onto the belt. A
second ESP32 ran the conveyor, and a laptop with a camera sorted the packages on the belt.

The user changed the system concept. In their words: the conveyor (with a camera, **no ESP32**) drops each
package onto the robot. The camera sends the package label to the laptop. The laptop decides, from the
company on the label, which route the robot takes, and sends that to the robot's ESP32. After the main line
divides, each company has its own branch (the user's example: red-and-white stripes for Amazon, black-and-
yellow for another company). The robot follows that branch and drops the package off.

So the **sorting moves from the conveyor to the robot**: the robot itself becomes the thing that sends each
package to the right company.

## 2. Decisions the user made

- **No colour sensor.** The robot does not recognise colours. The laptop tells it which way to turn ("left"
  or "right") where the line splits. The coloured stripes are **signs for people only**, laid beside each
  branch. The robot follows plain black tape everywhere. (The user chose this "plain version" over adding a
  colour sensor or a bar-code check at each branch.)
- The user approved this outline ("perfect"):
  1. A package rides the conveyor; the camera reads the label; the laptop picks the route from the company.
     The conveyor has no ESP32 - it just runs.
  2. The laptop sends the route to the robot over Wi-Fi.
  3. The robot waits at the end of the conveyor; the package drops onto its bed. It sets off only once it has
     **both** a package on the bed **and** a route from the laptop.
  4. It follows the black main line; where the line splits it takes the side the laptop said.
  5. At a stop mark on its branch it stops and **tips the package off to the right** (into a box or onto a
     table beside the line).
  6. It turns around on the spot, drives back up the branch to the main line, back to the conveyor, and
     turns around again, ready for the next package.
  - Speed by weight, obstacle stop, the E-stop and the alarms all stay as they are.
  - **Hardware:** no new sensors. The conveyor feeds from the robot's right.
  - **No walls on the bed** (the user, 2026-11-10, declining the low walls I proposed for the front, back
    and left edges). So the robot's hardware does not change at all.
  - ~~The side distance sensor checks the conveyor at the pickup~~: **the side sensor was removed** (the user,
    2026-11-10), so nothing checks that the conveyor is beside the robot.
  - Two companies (one split) for now, but a route is stored as a **list of turns** so more companies can be
    added later.
- **Hardware changes since this design was written (the user, 2026-11-10), already in the repo:** JGB37-520
  110 RPM motor sets with 65 mm wheels (the robot sits 17.5 mm lower; with the 6 mm acrylic plates the bed top is about 220 mm), printed ball
  casters, **3 IR line sensors** 18 mm apart instead of the 8-sensor array, no I/O expander, **no side
  sensor**. The numbers below are updated for them.
- **No load cells** (the user, 2026-11-10, "remove the weight aspect of this project"). This is already done in
  the repo: the cargo plate stands on 4 printed risers, and the firmware no longer weighs. It changes this
  design in two places, marked **[no load cells]** below:
  - The robot **can't feel the package land**, so "a package on the bed" (step 3 above) is no longer
    something it can check. What tells it to set off is an open question (section 13).
  - Speed by weight, the overload refusal and the "package still on the bed" check after tipping are gone.

## 3. Goals

1. The robot delivers each package to its company's drop-off point by itself, chosen by the laptop, and comes
   back to the conveyor for the next one, many times in a row.
2. No change to the robot's hardware at all: same parts, same wiring, same 3D model.
3. The laptop side is as simple as possible: one short text message per package.
4. Everything in the repo stays consistent: firmware, 3D model and drawings, PDFs, `BUILD_GUIDE.md`,
   `cad/README.md`.

## 4. Non-goals

- Reading the label, the camera, or any image recognition on the laptop. That is a separate part of the
  system. This project only provides the way the laptop talks to the robot, plus a small test program.
- Building or controlling the conveyor. It only has to meet the interface in section 7.
- Recognising colours or companies on the robot.
- T-junctions, crossings, loops, or more than 4 splits on one route.

## 5. Anti-goals (would count as failure even if it works)

- Adding any new sensor, board or wire to the robot.
- The robot needing the internet or the building's Wi-Fi to work.
- Linkages, extra motors, or anything mechanically clever (the user prefers simple mechanisms).
- A design the user can't run and explain at the defence without help.

## 6. The track

Seen from above. Going out, the robot drives up the main line, then left along branch A or right along
branch B.

```
            [box A]
   (end) ===X====D====\    /====D====X=== (end)        X = end mark (turn-around), D = drop-off mark
                       \  /     [box B]                    (box on the robot's RIGHT as it drives out)
   red/white stripes    \/     black/yellow stripes     stripes = signs for people, >= 10 cm from the line
   beside branch A      ||     beside branch B
                        ||  <- main line (black tape)
                        ||
                      ==P==  <- [conveyor]   P = pickup mark; the belt ends BESIDE the robot, on its RIGHT
                        ||                       as it drives out (see 7.1)
                        X     main-line end mark (turn-around), >= 30 cm clear all round
```

- **All lines the robot follows are black matte tape**, 19-50 mm wide, on a light floor, curves of at least
  30 cm radius (as in the current guide).
- **Splits are Y-shaped**: the two branches leave the line on either side at a gentle angle (about 20-45°
  each). The robot must be on a straight piece for at least 30 cm before a split.
- **Marks** are the existing station marks: a bar of black tape across the line, at least 12 cm long and 2-5
  cm wide. Three kinds, all identical on the floor:
  - **P, pickup**: on the main line beside the conveyor.
  - **D, drop-off**: one per branch, on a straight part, with the drop box beside it on the robot's right.
  - **X, end mark**: where the robot turns around. One at the far end of each branch (at least 50 cm past
    D, so the robot doesn't hit the box while turning) and one at the conveyor end of the main line (past
    P). Nothing may be within **30 cm** of an end mark: that is the circle the robot's corners sweep when it
    turns on the spot (about 27.5 cm radius).
- **No other tape** within 10 cm of a line: the coloured stripes go at least 10 cm from the black line, on
  either side. The 3 line sensors see about 5 cm of floor, and the black stripes in black/yellow tape would
  otherwise look like a second line. (This replaces the "5 cm" said in chat.)
- More companies later: a branch can split again. Each extra split adds one turn to that company's route.

The robot knows the track only as this pattern. It does not store a map.

## 7. Interfaces

### 7.1 Conveyor -> robot (mechanical)

- The robot stops at P facing out, with the middle of the bed level with P (as it does now with
  `stop_mm`). The conveyor is on its right.
- **The belt ends beside the robot's path, not over it** (my call, after the review): nothing hangs over
  the robot. The belt surface is **0.5-2 cm above the bed top** (bed top about 220 mm, so the belt top is
  about 225-240 mm). The package leaves the end of the belt, crosses a **gap of about 3-5 cm** and lands
  on the bed with almost no drop. This keeps the bed from being knocked, and keeps the conveyor out of
  the ultrasonic sensor's view.
  - **There are no walls**, so the package has to come to rest on the bed by itself: the belt must run
    slowly enough that the package stops before the bed's left edge (the bed is 28 cm across). To be
    checked with the real conveyor.
  - Packages must be at least about 10 cm long in the direction the belt moves, so they can't fall into
    the gap.
  - The gap is there because the robot also drives past the belt end on its way in, with the conveyor on
    its LEFT. So no part of the conveyor may come within **19 cm of the line** at any height above 7 cm
    (the robot's deck and bed reach about 15 cm out), or within **23 cm** of the line lower down
    (the 65 mm wheels reach 19 cm). Those numbers include about 4 cm for the robot wobbling on the line.
- The conveyor stays outside the 30 cm turning circle of the main-line end mark.
- The conveyor runs without any electronics talking to it. The person loading it puts **one package at a
  time** on the belt, only when the robot says it is ready (see 7.2).
- This **replaces** the old conveyor interface in `BUILD_GUIDE.md` 8.2 (belt top below 200 mm, conveyor
  on the right at the drop-off) and the old conveyor model in git history (commit 6aa6d0b, belt at 195 mm,
  with its own ESP32).

### 7.2 Laptop <-> robot (Wi-Fi)

- **The robot makes its own Wi-Fi network** (my call): name `CargoRobot`, password set in `config.h`. The
  laptop joins it. Nothing depends on the building's Wi-Fi or the internet.
- The laptop connects to the robot on a fixed address and port and sends **one line of text per package**.
  The commands use the same style as the existing Serial Monitor commands:
  - `route L` / `route R`, or several turns for later layouts (`route RL` = right at the first split, left
    at the second). Up to 4 turns. A label for the log may follow: `route L amazon`.
  - `status`, and `stop` (stops everything, as in the Serial Monitor).
- The robot answers with short lines of text the laptop can show: whether it accepted the route, state
  changes, "ready for the next package", "delivered", and alarms with their reason.
- The **company -> turns table lives on the laptop**, not on the robot (the laptop "decides the route based
  on the company"). Moving a branch means editing that table only.
- `route ...` also works typed in the Serial Monitor, so the robot can be tested with no laptop at all.
- **Rules for the waiting route** (my calls, after the review; they keep a route from ever reaching the
  wrong package). **[no load cells]** These rules assumed the robot feels the package land. Without load
  cells, the rules that depend on that ("empty bed", "no package lands", "a package on the bed with no
  route") must be redone once section 13, question 1 is answered:
  - The robot accepts a route **only while it is waiting at P with an empty bed**. At any other time it
    answers "not ready". (A package put on the belt while the robot is away would fall on the floor
    anyway, so the laptop never needs to send early.)
  - The route arrives before the package lands (the camera sees the package on the belt) and waits.
  - It holds **at most one** waiting route. A new `route` **replaces** it (answer "replaced"), and
    `cancel` clears it. This covers a package that jams or is lifted off and replaced by another.
  - A waiting route **expires** if no package lands within a set time (setting, default 60 s): "route
    expired".
  - `stop`, the E-stop and a power cycle also clear it. The route is used up when the robot leaves P.
  - A package on the bed with no route after a set time (setting, default about 10 s) raises the alarm "no
    route from the laptop". The person takes the package off (or the laptop sends its route) and presses
    start; the robot goes back to waiting.
  - Unknown or unreadable label: the laptop sends nothing and tells the person. The "no route" alarm is the
    robot's backstop.

### 7.3 Laptop test program (in this repo)

A small Python program (standard library only) that joins the robot, keeps the company -> turns table at
the top of the file (`amazon: L`, second company: `R`), lets you type a company name to send its route, and
prints everything the robot says. The user tests the whole robot with it, and the label-reading program
(not part of this project) can use it or copy its few lines.

## 8. How the robot behaves

### 8.1 One delivery

1. **Wait at P** (green light). The bed is empty and level. The robot says "ready" to the laptop.
2. **Package and route.** **[no load cells]** It sets off once it has a route **and** the package is on the
   bed. How it knows the package is on the bed is open (section 13, question 1).
3. **Out along the main line**, at the speed allowed by obstacles, as now.
4. **At each split**, it takes the next turn of the route.
5. **On the branch**: the first mark is D. It stops level with it, tips the package off to the right,
   holds and lowers, exactly as the current TIP/HOLD/LOWER steps do. **[no load cells]** It can't check
   that the package fell off.
6. **On to the end mark X** of the branch, stop, **turn around on the spot**.
7. **Back**: it passes D again, and at each split on the way back takes the **opposite side** of what it took
   going out, in reverse order. Example: out `RL` (right at the 1st split, left at the 2nd) -> back: right
   at the 2nd split, then left at the 1st. On a Y-split that is always the way to the main line.
8. **Down the main line**: it passes P (the conveyor on its left now) and stops at the main-line end mark X,
   turns around on the spot.
9. **Forward to P**, stop level with it (the conveyor now on its right). Back to step 1.

The robot tells marks apart only by counting them on each leg: out = D, then X; back = D, P, then X;
after turning = P. While it is passing a split it must not count it as a mark.

### 8.2 Splits

- A split shows up on the 3 line sensors as **left and right on black, middle on the floor** (the two
  branches either side of the middle sensor).
- Going through a split, the robot steers by the left or right patch, as its route says, until it is
  clearly on one line again, then follows normally.
- If it misses a split, it drives straight between the branches, loses the line and raises the "line lost"
  alarm: a safe failure.

### 8.3 Turning around

- It turns on the spot (it already can: differential drive) about the middle of the drive axle, which is
  the middle of the bed.
- It stops at an end mark the same way it stops at P or D (middle of the bed level with the mark), turns by
  the wheel encoders to roughly 180°, then keeps turning slowly until the line sensor finds the line again.
  If it doesn't find the line within a sensible angle, it raises an alarm.
- The obstacle sensor does not stop a turn (it points wherever the robot is facing); the 30 cm clear circle
  is the track's job.

### 8.4 What stays the same

Obstacle slow/stop, line-lost alarm, E-stop, alarm-then-press-
start-to-carry-on, `run` / `run N` / `stop`, the start button, settings saved in flash, all bench and
calibration commands.

## 9. Hardware

- **Nothing changes for this design**: no walls (the user's decision). The ESP32's built-in Wi-Fi is the
  laptop link. (The battery-sense pin, GPIO33, is on ADC1, so Wi-Fi does not disturb it.)
- The other hardware changes of 2026-11-10 (section 2) are already in the repo.

## 10. What has to be updated in the repo

- **Firmware** (`firmware/cargo_robot/`): Wi-Fi link, route handling, splits, turning around, the new
  mission order, new settings for the new numbers, and bench commands to test splits, turning and the
  link on their own. Must still compile with the ESP32 core used now (3.3.12).
- **3D model and its outputs**: no change (no walls, and the CAD scripts never mention the conveyor).
- **Schematics**: the system block diagram (E01) shows the laptop's Wi-Fi link. No wiring changes.
  Regenerate in the existing order (robot_cad -> check_render -> make_autocad -> make_schematics ->
  make_docs).
- **`BUILD_GUIDE.md`**: the system description and "what changed" table; the timeline goals for weeks 6
  and 8; the track (section 6: layout figure, splits, end marks, coloured stripes, clear circles); the
  mission (week 8) and the conveyor interface (8.2); joining the robot's Wi-Fi and the laptop test program;
  the test tables (a new route-accuracy test, e.g. 10 packages each way, right branch every time); the
  troubleshooting table; Appendix B (regenerated).
- **`cad/README.md`**: the bed now drops packages into the drop box.
- **Laptop test program** (section 7.3), e.g. under `tools/`.
- No new backup folder: the current version stays in git history (`backup_v2/` stays as it is).

## 11. Left to the builder

- Firmware structure, state names, file split, setting names and defaults.
- How a split is detected and when it is "done", how marks are ignored while passing it, turning speeds and
  limits, the port number, the exact text of messages.
- The layout figure in the guide.
- Whether to add pure-logic checks (e.g. split detection, route reversal) that run on the laptop without
  the robot.

## 12. Risks

- **Not tested on hardware yet**, like the rest of the firmware. Splits and turning around need tuning on
  the real track; the guide must give the tuning steps and what to change for each symptom.
- **Passing the conveyor**: the robot drives past the belt end with only a few cm to spare (7.1). If it
  wobbles on the line there, it could touch it. To be checked on the track; the main line should be dead
  straight past the conveyor.
- **Packages landing badly**: a small or soft package could catch on the gap, and with no walls a package
  arriving too fast could slide off the far side of the bed. Packages at least 10 cm long and a slow belt
  (7.1); to be checked with the real conveyor.

## 13. On hold, and what is still open

The user (2026-11-10): "for the software parts hold until I get new info". Nothing is built until they come
back with it and approve this document.

Answered:

- **The conveyor is not built yet** (the user, 2026-11-10). So it can be built to section 7.1 (belt top at
  about 225-240 mm, ending beside the robot), and the old interface does not constrain it.

Still open, and only the user can answer:

1. **[no load cells] What tells the robot the package is on the bed?** Two simple options, no new parts:
   the laptop sends the route only once it has seen the package leave the belt (the route is the "go"),
   or a person presses the start button after the package lands. The first keeps people out of the loop;
   the second needs nobody to trust the camera's timing.
2. **The laptop's network**: does the laptop need the internet or another Wi-Fi network while it runs (an
   online label-reading service, a camera that connects over Wi-Fi)? If yes, the robot should join a phone
   hotspot or small router instead of making its own network (7.2).
- **The person loading the conveyor** must wait for "ready" (the conveyor has no electronics to stop it).
