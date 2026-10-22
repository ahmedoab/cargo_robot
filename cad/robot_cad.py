"""
Parametric CAD model - Phase II autonomous cargo robot, DESIGN v2 (simplified) with a tilting cargo bed.

Run:   python robot_cad.py          (needs: pip install cadquery)
Writes ./output/
  robot_assembly.step         full coloured assembly, bed flat -> SolidWorks / Fusion 360 / Onshape / FreeCAD
  robot_assembly_tipped.step  the same with the bed tipped to TIP_ANGLE
  parts/*.step, *.stl         the custom-made parts
  dxf/*.dxf                   the four plates, ready for the laser cutter (every hole already in them)
  views/*.svg                 line drawings
  bom.csv                     bill of materials

Design choices that keep it simple to build:
  - frame: 6 x M6 threaded rod + nuts, instead of 6 printed pillars with heat-set inserts
  - casters: 2 bought swivel casters with an M8 threaded stem - the height is set with the nuts,
    so getting all 4 wheels onto the floor takes a spanner, not shims
  - motor brackets: the standard steel bracket sold for JGB37 motors, instead of printed ones
  - one ESP32 runs everything: wheels, encoders, load cells, line sensor, ultrasonic sensor and the bed servos
  - tilting bed: two servos sit on the hinge line and turn the bed directly - no screws, levers or linkages.
    The servo shafts ARE the hinge. The bed rests on two posts when flat, so the servos only work while tipping.
  - the load cells ride on the tilting plate, so they weigh only the cargo plate and the package
  - the holes in the plates are laser-cut (load-cell slots that fit any common bar cell, servo mounts,
    the 22 mm E-stop hole, the switch, button and LED holes) - nothing is drilled by hand except the
    motor brackets in the plywood
  - E-stop, master switch, start button and LEDs mount through the deck, so their wiring stays inside
  - lower electronics bay (100 mm) -> lower centre of gravity, shorter wires
  - battery lies across the robot and slides out sideways for charging

Axes: X = forward, Y = left, Z = up. Origin = floor under the middle of the drive axle. Units: mm.
The bed tips to the RIGHT (-Y): positive TIP_ANGLE lifts its left edge.
Bought parts are simplified envelopes with the right outer sizes (good for layout and clearance checks).
"""
import csv
import math
from pathlib import Path

import cadquery as cq

# ============================== PARAMETERS ==============================
L, W = 400, 300                  # plate size (X, Y)
BASE_T, DECK_T, CARGO_T = 6, 5, 5    # bottom plate 6 mm plywood; deck and cargo plate 5 mm acrylic
TILT_T = 6                       # tilting plate: 6 mm plywood
BAY_H = 100                      # clear height of the electronics bay

WHEEL_D, WHEEL_W = 100, 30       # drive wheels
AXLE_Z = WHEEL_D / 2
MOTOR_DROP = 22                  # stock steel bracket: motor axis -> plate underside (casters absorb +-5 mm)
MOTOR_FACE_Y = W / 2 - 5         # outer face of the bracket's vertical plate
WHEEL_Y = W / 2 + 15             # wheel centre-plane

CASTER_X = 160                   # swivel-caster stems at X = +-160
CASTER_WHEEL_D, CASTER_WHEEL_W = 50, 20
CASTER_TRAIL = 15                # wheel axle sits this far behind the stem

ROD_PTS = [(x, s * (W / 2 - 15)) for x in (-190, 60, 190) for s in (-1, 1)]   # M6 threaded rods

ESTOP_POS = (150, 110)           # 22 mm E-stop, front-left of the deck
SWITCH_POS = (155, 45)           # 12 mm toggle switch (master), front of the deck
BUTTON_POS = (-165, 30)          # 12 mm start button, rear of the deck
LED_POS = {"green": (-165, 68), "red": (-165, 92)}

# ---- tilting bed ----
TIP_ANGLE = 40                   # bed angle when it drops the package (deg)
Y_AX = -122                      # hinge axis = the two servo shafts: Y position (bed tips towards -Y)
SERVO = dict(L=65, W=30, H=48,   # 60 kg.cm "1/5 scale" servo (e.g. DS5160): body L x W x H, shaft on the L x W face
             shaft=16,           # shaft centre -> near end of the body (along L)
             tab_h=34, tab_t=3.5, tab_ext=9.5,   # mounting tabs: height from the bottom, thickness, overhang
             hole_l=74, hole_w=18,               # tab hole spacing (along L, along W)
             boss_d=14, boss_h=1)
HORN_D, HORN_T, HORN_HUB = 38, 3, 5   # 25T aluminium disc horn
ADAPTER_WALL = 6                 # printed horn adapter: wall the horn screws onto
TILT_X = 118                    # tilting plate spans X = -118..118
TILT_Y1 = 125                    # ... and Y = Y_AX..125
CELL = (80, 12.7, 12.7)          # straight-bar load cell
CELL_X = (-55, 55)               # centres of the two cell columns
CELL_Y = 110
CELL_ZONE = (5, 23)              # screw-hole centres lie this far from a cell's end (fits common cells)
CARGO = (260, 280)               # cargo plate, centred on X = 0, Y = 0
CARGO_X = 0
REST_PTS = [(-40, 95), (40, 95)] # printed rest posts under the bed's left side

# ---- electronics and sensors ----
ESP_POS = (110, 62)              # ESP32 DevKit on its screw-terminal breakout (USB end towards +Y)
IO_POS = (110, -45)              # I/O board: MCP23017 + resistor dividers on perfboard
BATTERY = (50, 155, 35)          # 3S 5000 mAh LiPo lying across the robot (X, Y, Z)
LINE_X = 98                      # centre of the 8-channel IR line sensor board, under the bottom plate
LINE_BRKT_HOLES = [(LINE_X - 12, 0), (LINE_X + 12, 0)]
SONAR_HOLES = [(178, -20), (178, 20)]
TOF_X = -40                      # side distance sensor (VL53L0X), looking right (-Y)
TOF_HOLES = [(TOF_X - 9, -141), (TOF_X + 9, -141)]
SERVO_CABLE_SLOTS = [(150, -50), (-150, -50)]
BED_CABLE_SLOT = (0, -131)

# ---- derived Z levels ----
Z_BASE = AXLE_Z + MOTOR_DROP     # underside of bottom plate = 72
Z_LOWER = Z_BASE + BASE_T        # top of bottom plate = 78
Z_DECK = Z_LOWER + BAY_H         # underside of deck = 178
Z_DECK_TOP = Z_DECK + DECK_T     # 183
SERVO_FOOT = 7                   # printed servo mount: floor under the servo
Z_AX = Z_DECK_TOP + SERVO_FOOT + SERVO["W"] / 2   # servo shaft (hinge axis) = 205
Z_TILT = Z_AX - TILT_T / 2       # underside of the tilting plate = 202 (axis in the middle of the plate)
Z_TILT_TOP = Z_TILT + TILT_T     # 208
Z_CELL = Z_TILT_TOP + 5          # load cells sit on 5 mm spacers
Z_CARGO = Z_CELL + CELL[2] + 5   # underside of cargo plate
X_SERVO_TOP = TILT_X + ADAPTER_WALL + HORN_T + HORN_HUB + SERVO["boss_h"]   # servo shaft face = 133
AXIS = ((0, Y_AX, Z_AX), (1, Y_AX, Z_AX))    # tipping axis (along +X)


def cell_layout(cx):
    """(outer, x of the fixed-end zone centre, x of the loaded-end zone centre) for a cell centred at cx.
    The fixed end (screwed to the tilting plate) faces the outside of the bed; the loaded end carries the cargo plate."""
    outer = -1 if cx < CARGO_X else 1
    mid = (CELL_ZONE[0] + CELL_ZONE[1]) / 2
    fixed_end, loaded_end = cx + outer * CELL[0] / 2, cx - outer * CELL[0] / 2
    return outer, fixed_end - outer * mid, loaded_end + outer * mid


# ============================== COLOURS ==============================
ACRYLIC = (0.75, 0.88, 1.0, 0.45)
PLYWOOD = (0.85, 0.70, 0.48, 1)
PRINT = (1.0, 0.55, 0.1, 1)
STEEL = (0.62, 0.63, 0.66, 1)
DARK_STEEL = (0.30, 0.30, 0.33, 1)
ALU = (0.80, 0.82, 0.86, 1)
RUBBER = (0.08, 0.08, 0.08, 1)
PCB = (0.10, 0.45, 0.20, 1)
PCB_BLUE = (0.10, 0.25, 0.60, 1)
BLACK = (0.15, 0.15, 0.15, 1)
RED = (0.85, 0.10, 0.10, 1)
GREEN = (0.10, 0.75, 0.20, 1)
YELLOW = (0.95, 0.80, 0.10, 1)
BATT = (0.20, 0.30, 0.80, 1)
MOTOR = (0.35, 0.35, 0.38, 1)
SERVO_BLUE = (0.15, 0.20, 0.35, 1)
SILVER = (0.75, 0.75, 0.78, 1)


# ============================== HELPERS ==============================
def box(sx, sy, sz, cx=0, cy=0, cz=0):
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def span(x0, x1, y0, y1, z0, z1):
    """Box between two corners."""
    return box(x1 - x0, y1 - y0, z1 - z0, (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)


def cyl_z(d, h, cx=0, cy=0, z0=0):
    return cq.Workplane("XY").circle(d / 2).extrude(h).translate((cx, cy, z0))


def cyl_y(d, y0, y1, cx=0, cz=0):
    """Cylinder along +Y from y0 to y1 (y1 > y0)."""
    return (cq.Workplane("XY").circle(d / 2).extrude(y1 - y0)
            .rotate((0, 0, 0), (1, 0, 0), -90).translate((cx, y0, cz)))


def cyl_x(d, x0, x1, cy=0, cz=0):
    """Cylinder along +X from x0 to x1 (x1 > x0)."""
    return (cq.Workplane("YZ").circle(d / 2).extrude(x1 - x0).translate((x0, cy, cz)))


def hex_nut(af, h, cx, cy, z0, hole):
    """Hex nut: af = width across flats."""
    return (cq.Workplane("XY").polygon(6, af / math.cos(math.pi / 6)).extrude(h)
            .cut(cq.Workplane("XY").circle(hole / 2).extrude(h)).translate((cx, cy, z0)))


def washer(od, h, cx, cy, z0, hole):
    return cyl_z(od, h, cx, cy, z0).cut(cyl_z(hole, h, cx, cy, z0))


def drill(wp, pts, d, z0, h):
    """Cut vertical holes of diameter d at XY points, spanning z0..z0+h."""
    tool = cq.Workplane("XY").pushPoints(pts).circle(d / 2).extrude(h).translate((0, 0, z0))
    return wp.cut(tool)


def slot_x(cx, cy, z0, h, centres, width):
    """Round-ended slot along X: 'centres' = distance between the end-circle centres."""
    return cq.Workplane("XY").slot2D(centres + width, width).extrude(h).translate((cx, cy, z0))


def mirror_x(wp):
    """Mirror image front <-> rear (X -> -X)."""
    return wp.mirror("YZ")


# ============================== MODEL ==============================
groups = {}          # group name -> list of (name, workplane, colour)
fabricated = {}      # name -> workplane (exported as individual STEP/STL)


def add(group, name, wp, color, fab=False):
    groups.setdefault(group, []).append((name, wp, color))
    if fab:
        fabricated[name] = wp


# ---------- Chassis: 2 plates on 6 threaded rods ----------
def bottom_plate():
    zc, z0, h = Z_BASE + BASE_T / 2, Z_BASE - 1, BASE_T + 2
    p = box(L, W, BASE_T, 0, 0, zc)
    p = drill(p, ROD_PTS, 6.5, z0, h)
    p = drill(p, [(-CASTER_X, 0), (CASTER_X, 0)], 8.5, z0, h)
    p = drill(p, LINE_BRKT_HOLES + SONAR_HOLES, 3.4, z0, h)
    for sx in (-31, 31):                         # 2 velcro straps around the battery
        for sy in (-45, 45):
            p = p.cut(box(4, 22, h, sx, sy, zc))
    for sy in (-70, 70):                         # motor + encoder cables come up here
        p = p.cut(box(20, 12, h, 40, sy, zc))
    return p


def servo_mount_holes(front=True):
    s = 1 if front else -1
    xt = X_SERVO_TOP
    return [(s * (xt + 6), Y_AX - 10), (s * (xt + 6), Y_AX + 43), (s * (xt + 44), Y_AX + 16)]


def top_deck():
    zc, z0, h = Z_DECK + DECK_T / 2, Z_DECK - 1, DECK_T + 2
    p = box(L, W, DECK_T, 0, 0, zc)
    p = drill(p, ROD_PTS, 6.5, z0, h)
    p = drill(p, [ESTOP_POS], 22.5, z0, h)
    p = drill(p, [SWITCH_POS, BUTTON_POS], 12.2, z0, h)
    p = drill(p, list(LED_POS.values()), 6.2, z0, h)            # 5 mm LEDs in panel clips
    p = drill(p, servo_mount_holes(True) + servo_mount_holes(False) + REST_PTS, 4.5, z0, h)
    p = drill(p, TOF_HOLES, 3.4, z0, h)
    for x, y in SERVO_CABLE_SLOTS:                              # servo cables go down into the bay
        p = p.cut(box(16, 12, h, x, y, zc))
    p = p.cut(box(30, 10, h, *BED_CABLE_SLOT, zc))              # load-cell bundle from the bed, next to the hinge
    return p


add("chassis", "bottom_plate", bottom_plate(), PLYWOOD, fab=True)
add("chassis", "top_deck", top_deck(), ACRYLIC, fab=True)

ROD_Z0, ROD_Z1 = Z_BASE - 9, Z_DECK_TOP + 10
for i, (x, y) in enumerate(ROD_PTS):
    add("chassis", f"rod_{i}", cyl_z(6, ROD_Z1 - ROD_Z0, x, y, ROD_Z0), STEEL)
    hw = (hex_nut(10, 5, x, y, Z_BASE - 6.6, 6)                      # under the bottom plate
          .union(washer(12, 1.6, x, y, Z_BASE - 1.6, 6.4))
          .union(washer(12, 1.6, x, y, Z_LOWER, 6.4))                # on top of the bottom plate
          .union(hex_nut(10, 5, x, y, Z_LOWER + 1.6, 6))
          .union(hex_nut(10, 5, x, y, Z_DECK - 6.6, 6))              # under the deck
          .union(washer(12, 1.6, x, y, Z_DECK - 1.6, 6.4))
          .union(washer(12, 1.6, x, y, Z_DECK_TOP, 6.4))             # on top of the deck
          .union(hex_nut(10, 5, x, y, Z_DECK_TOP + 1.6, 6)))
    add("chassis", f"rod_nuts_{i}", hw, DARK_STEEL)


# ---------- Drive: stock steel brackets, gear motors, wheels ----------
def bracket_left():
    """Generic steel L-bracket for 37 mm gear motors (hangs under the plate)."""
    t = 2.5
    z_bot = AXLE_Z - 22
    vert = box(36, t, Z_BASE - z_bot, 0, MOTOR_FACE_Y - t / 2, (Z_BASE + z_bot) / 2)
    base = box(36, 32, t, 0, MOTOR_FACE_Y - 16, Z_BASE - t / 2)
    b = vert.union(base).cut(cyl_y(12.5, MOTOR_FACE_Y - t - 1, MOTOR_FACE_Y + 1, 0, AXLE_Z))
    for x in (-15.5, 15.5):
        b = b.cut(cyl_y(3.4, MOTOR_FACE_Y - t - 1, MOTOR_FACE_Y + 1, x, AXLE_Z))
    return drill(b, [(-12, MOTOR_FACE_Y - t - 20), (12, MOTOR_FACE_Y - t - 20)], 4.5, Z_BASE - t - 1, t + 2)


def gearmotor_left():
    f = MOTOR_FACE_Y - 2.5                      # gearbox face against the bracket
    return (cyl_y(37, f - 30, f, 0, AXLE_Z)
            .union(cyl_y(35, f - 60, f - 30, 0, AXLE_Z))
            .union(cyl_y(33, f - 72, f - 60, 0, AXLE_Z)))


def wheel(yc):
    y0, y1 = yc - WHEEL_W / 2, yc + WHEEL_W / 2
    rim_d = WHEEL_D - 24
    tire = cyl_y(WHEEL_D, y0, y1, 0, AXLE_Z).cut(cyl_y(rim_d, y0 - 1, y1 + 1, 0, AXLE_Z))
    rim = cyl_y(rim_d, y0 + 3, y1 - 3, 0, AXLE_Z).cut(cyl_y(6, y0, y1, 0, AXLE_Z))
    for i in range(6):
        a = math.radians(60 * i)
        rim = rim.cut(cyl_y(14, y0, y1, 22 * math.cos(a), AXLE_Z + 22 * math.sin(a)))
    return tire, rim


br = bracket_left()
add("drive", "motor_bracket_L", br, DARK_STEEL)
add("drive", "motor_bracket_R", br.mirror("XZ"), DARK_STEEL)
add("drive", "gearmotor_L", gearmotor_left(), MOTOR)
add("drive", "gearmotor_R", gearmotor_left().mirror("XZ"), MOTOR)
shaft = cyl_y(6, MOTOR_FACE_Y - 2.5, WHEEL_Y - 2, 0, AXLE_Z)
add("drive", "shaft_L", shaft, STEEL)
add("drive", "shaft_R", shaft.mirror("XZ"), STEEL)
for side, s in (("L", 1), ("R", -1)):
    tire, rim = wheel(s * WHEEL_Y)
    add("drive", f"tire_{side}", tire, RUBBER)
    add("drive", f"rim_{side}", rim, BLACK)


# ---------- Casters: bought 50 mm swivel casters, M8 threaded stem ----------
def swivel_caster(x):
    """Returns (body, wheel). Stem through an 8.5 mm plate hole, one M8 nut above and one below:
    turning the lower nut raises / lowers the caster."""
    cz = CASTER_WHEEL_D / 2
    ax = x - CASTER_TRAIL
    head_top = Z_BASE - 1.6 - 6.5
    body = (cyl_z(8, Z_LOWER + 10 - (head_top - 1), x, 0, head_top - 1)          # M8 stem
            .union(hex_nut(13, 6.5, x, 0, head_top, 8))
            .union(washer(16, 1.6, x, 0, Z_BASE - 1.6, 8.4))
            .union(washer(16, 1.6, x, 0, Z_LOWER, 8.4))
            .union(hex_nut(13, 6.5, x, 0, Z_LOWER + 1.6, 8))
            .union(cyl_z(40, 9, x, 0, head_top - 9)))                          # swivel bearing
    fork_z0 = cz - 7
    for s in (-1, 1):
        yc = s * (CASTER_WHEEL_W / 2 + 1 + 1.5)
        plate = box(CASTER_TRAIL + 16, 3, head_top - 9 - fork_z0 + 1, x - CASTER_TRAIL / 2, yc,
                    (fork_z0 + head_top - 8) / 2)
        body = body.union(plate)
    body = body.union(cyl_y(6, -CASTER_WHEEL_W / 2 - 4, CASTER_WHEEL_W / 2 + 4, ax, cz))   # axle
    wheel_ =(cyl_y(CASTER_WHEEL_D, -CASTER_WHEEL_W / 2, CASTER_WHEEL_W / 2, ax, cz)
              .cut(cyl_y(6.2, -CASTER_WHEEL_W / 2, CASTER_WHEEL_W / 2, ax, cz)))
    return body, wheel_


for tag, s in (("front", 1), ("rear", -1)):
    body, wh = swivel_caster(s * CASTER_X)
    add("drive", f"caster_{tag}", body, STEEL)
    add("drive", f"caster_wheel_{tag}", wh, RUBBER)


# ---------- Electronics bay (on the bottom plate, z = Z_LOWER) ----------
z = Z_LOWER
add("electronics", "battery_3S_5000mAh", box(*BATTERY, 0, 0, z + BATTERY[2] / 2), BATT)
for tag, y in (("L", 60), ("R", -60)):
    x = -150
    add("electronics", f"BTS7960_{tag}_pcb", box(50, 50, 1.6, x, y, z + 1.0), PCB)
    add("electronics", f"BTS7960_{tag}_heatsink", box(40, 40, 20, x, y, z + 1.8 + 10), BLACK)
ex_, ey_ = ESP_POS
add("electronics", "esp32_breakout_pcb", box(70, 57, 1.6, ex_, ey_, z + 1.8), PCB_BLUE)
add("electronics", "esp32_terminals", box(8, 50, 10, ex_ - 31, ey_, z + 2.6 + 5), (0.1, 0.4, 0.8, 1))
add("electronics", "esp32_devkit", box(28, 55, 1.6, ex_ + 4, ey_, z + 2.6 + 11 + 0.8), BLACK)
add("electronics", "esp32_module", box(18, 25, 3, ex_ + 4, ey_ - 12, z + 2.6 + 12.6 + 1.5), SILVER)
add("electronics", "esp32_usb", box(8, 6, 3, ex_ + 4, ey_ + 27.5 + 1, z + 2.6 + 12.6 + 1.5), STEEL)
ix, iy = IO_POS
add("electronics", "io_board_pcb", box(70, 50, 1.6, ix, iy, z + 1.8), (0.55, 0.40, 0.20, 1))
add("electronics", "io_board_mcp23017", box(36, 22, 6, ix - 10, iy + 8, z + 2.6 + 3), PCB_BLUE)
add("electronics", "buck_12V_5V", box(51, 26, 1.6, -60, 85, z + 1.0), PCB_BLUE)
add("electronics", "buck_inductor", cyl_z(20, 12, -60, 85, z + 1.8), BLACK)
add("electronics", "servo_buck_12V_7V4_pcb", box(60, 51, 1.6, -72, -95, z + 1.0), PCB_BLUE)
add("electronics", "servo_buck_heatsink", box(40, 20, 18, -72, -95, z + 1.8 + 9), BLACK)
add("electronics", "power_bus_wagos", box(60, 40, 20, -85, -20, z + 10), BLACK)
add("electronics", "inline_fuse_holder", box(40, 15, 15, 45, -110, z + 7.5), RED)
add("electronics", "buzzer_module", box(25, 15, 12, -110, 105, z + 6), BLACK)


# ---------- Line sensor (front, under the bottom plate) ----------
LINE_PCB = (22, 105, 1.6)        # 8 x TCRT5000 board: depth (X) x width (Y) x thickness
Z_LINE_PCB = 12.4                # underside of the board; the sensors look down from here (~5 mm above the floor)


def line_sensor_bracket():
    """Hangs under the bottom plate between the drive wheels and the front caster; the sensor board screws under its
    shelf. It stays behind X = LINE_X + 16, clear of the front caster however it swivels."""
    flange = span(LINE_X - 16, LINE_X + 16, -30, 30, Z_BASE - 6, Z_BASE)   # bolted under the bottom plate
    web = span(LINE_X - 3, LINE_X + 3, -30, 30, Z_LINE_PCB + 1.6 + 4, Z_BASE - 6)
    shelf = span(LINE_X - 13, LINE_X + 13, -56, 56, Z_LINE_PCB + 1.6, Z_LINE_PCB + 1.6 + 4)   # the board hangs under it
    b = flange.union(web).union(shelf)
    b = drill(b, LINE_BRKT_HOLES, 3.4, Z_BASE - 8, 10)
    return drill(b, [(LINE_X, -50), (LINE_X, 50)], 3.0, Z_LINE_PCB, 8)  # M3 screws into the board's end holes


add("sensors", "line_sensor_bracket", line_sensor_bracket(), PRINT, fab=True)
add("sensors", "line_sensor_pcb", box(*LINE_PCB, LINE_X, 0, Z_LINE_PCB + 0.8), PCB)
for i in range(8):                                                     # sensor 1 = leftmost (+Y) ... 8 = rightmost
    y = (3.5 - i) * 12
    add("sensors", f"tcrt5000_{i + 1}", box(5.8, 10.2, 7, LINE_X, y, Z_LINE_PCB - 3.5), BLACK)


# ---------- Ultrasonic sensor (HC-SR04) at the front of the bay, looking straight ahead ----------
SONAR_X = 196.5                  # middle of the holder face (3 mm thick, its front at X = 198)
SONAR_Z = Z_LOWER + 30           # height of the transducer centres


def sonar_bracket():
    """Base on the bottom plate + one upright face; the HC-SR04 sits behind it, its 2 transducers through 2 windows.
    Two M3 bolts hold its base to the bottom plate."""
    face = span(SONAR_X - 1.5, SONAR_X + 1.5, -26, 26, Z_LOWER + 4, SONAR_Z + 14)
    for y in (-13, 13):
        face = face.cut(cyl_x(16.5, SONAR_X - 3, SONAR_X + 3, y, SONAR_Z))
    b = span(170, 200, -32, 32, Z_LOWER, Z_LOWER + 4).union(face)
    for y0 in (-27, 24):                                               # 2 ribs behind the face, beside the board
        rib = (cq.Workplane("XZ").polyline([(180, Z_LOWER + 4), (SONAR_X - 1.5, Z_LOWER + 4),
                                            (SONAR_X - 1.5, SONAR_Z + 4)]).close().extrude(-3).translate((0, y0, 0)))
        b = b.union(rib)
    return drill(b, SONAR_HOLES, 3.4, Z_LOWER - 1, 6)


add("sensors", "sonar_bracket", sonar_bracket(), PRINT, fab=True)
add("sensors", "hc_sr04_pcb", span(SONAR_X - 3.1, SONAR_X - 1.5, -22.5, 22.5, SONAR_Z - 10, SONAR_Z + 10), PCB_BLUE)
add("sensors", "hc_sr04_cans", cyl_x(16, SONAR_X - 1.5, SONAR_X + 10.5, -13, SONAR_Z)
    .union(cyl_x(16, SONAR_X - 1.5, SONAR_X + 10.5, 13, SONAR_Z)), SILVER)


# ---------- Side distance sensor (VL53L0X), under the deck's right edge, looking right ----------
def tof_bracket():
    top = span(TOF_X - 16, TOF_X + 16, -150, -133, Z_DECK - 4, Z_DECK)
    face = span(TOF_X - 16, TOF_X + 16, -150, -147, Z_DECK - 32, Z_DECK - 4)
    b = top.union(face).cut(span(TOF_X - 5, TOF_X + 5, -151, -146, Z_DECK - 22, Z_DECK - 14))   # sensor window
    for sx in (-1, 1):                                                  # 2 ribs, clear of the sensor board
        b = b.union(span(TOF_X + sx * 15 - 1, TOF_X + sx * 15 + 1, -147, -137, Z_DECK - 16, Z_DECK - 4))
    return drill(b, TOF_HOLES, 3.4, Z_DECK - 6, 8)


add("sensors", "tof_bracket", tof_bracket(), PRINT, fab=True)
add("sensors", "vl53l0x_pcb", span(TOF_X - 12.5, TOF_X + 12.5, -147, -145.4, Z_DECK - 25, Z_DECK - 11), PCB_BLUE)
add("sensors", "vl53l0x_sensor", span(TOF_X - 2.5, TOF_X + 2.5, -147.8, -147, Z_DECK - 20, Z_DECK - 16), BLACK)


# ---------- Safety controls and status: panel-mounted through the deck ----------
ex, ey = ESTOP_POS
add("controls", "estop_bushing", cyl_z(22, 15, ex, ey, Z_DECK - 4), BLACK)
add("controls", "estop_collar", cyl_z(30, 6, ex, ey, Z_DECK_TOP).cut(cyl_z(22, 6, ex, ey, Z_DECK_TOP)), YELLOW)
add("controls", "estop_mushroom", cyl_z(40, 15, ex, ey, Z_DECK_TOP + 6), RED)
add("controls", "estop_contact_block", box(30, 40, 40, ex, ey, Z_DECK - 4 - 20), BLACK)
add("controls", "estop_nut", cyl_z(30, 4, ex, ey, Z_DECK - 4).cut(cyl_z(22, 4, ex, ey, Z_DECK - 4)), BLACK)
sx, sy = SWITCH_POS
add("controls", "switch_bushing", cyl_z(12, 20, sx, sy, Z_DECK - 8), STEEL)
add("controls", "switch_body", box(18, 28, 25, sx, sy, Z_DECK - 8 - 12.5), BLACK)
add("controls", "switch_nut", cyl_z(16, 3, sx, sy, Z_DECK_TOP).cut(cyl_z(12, 3, sx, sy, Z_DECK_TOP)), STEEL)
add("controls", "switch_lever", cyl_z(6, 18, sx, sy, Z_DECK - 8 + 20), RED)
bx, by = BUTTON_POS
add("controls", "button_body", cyl_z(12, 20, bx, by, Z_DECK - 20), STEEL)
add("controls", "button_nut", cyl_z(16, 3, bx, by, Z_DECK_TOP).cut(cyl_z(12, 3, bx, by, Z_DECK_TOP)), STEEL)
add("controls", "button_cap", cyl_z(10, 5, bx, by, Z_DECK_TOP + 3), GREEN)
for name, (lx, ly) in LED_POS.items():
    clip = (cyl_z(6, 13.5, lx, ly, Z_DECK - 7).union(cyl_z(8, 1.5, lx, ly, Z_DECK_TOP))
            .union(cyl_z(8, 2, lx, ly, Z_DECK - 2)).cut(cyl_z(5, 15, lx, ly, Z_DECK - 7)))
    add("controls", f"led_{name}_clip", clip, BLACK)
    add("controls", f"led_{name}", cyl_z(5, 6, lx, ly, Z_DECK_TOP + 1.5), GREEN if name == "green" else RED)


# ---------- Tilting bed: fixed half (2 servos on printed mounts, 2 rest posts) ----------
SV = SERVO


def servo_body_front():
    """Servo with its shaft pointing -X (towards the bed), its body lying along +Y from the shaft."""
    xt = X_SERVO_TOP
    y0, y1 = Y_AX - SV["shaft"], Y_AX - SV["shaft"] + SV["L"]
    zw = SV["W"] / 2
    body = span(xt, xt + SV["H"], y0, y1, Z_AX - zw, Z_AX + zw)
    xt0 = xt + SV["H"] - SV["tab_h"]
    tabs = span(xt0, xt0 + SV["tab_t"], y0 - SV["tab_ext"], y1 + SV["tab_ext"], Z_AX - zw, Z_AX + zw)
    yc = (y0 + y1) / 2
    for hy in (yc - SV["hole_l"] / 2, yc + SV["hole_l"] / 2):
        for hz in (Z_AX - SV["hole_w"] / 2, Z_AX + SV["hole_w"] / 2):
            tabs = tabs.cut(cyl_x(4.5, xt0 - 1, xt0 + SV["tab_t"] + 1, hy, hz))
    boss = cyl_x(SV["boss_d"], xt - SV["boss_h"], xt, Y_AX, Z_AX)
    return body.union(tabs).union(boss)


def servo_mount_front():
    """Printed L-bracket: a floor under the servo and an upright plate that its 4 tabs screw onto."""
    xt = X_SERVO_TOP
    y0, y1 = Y_AX - SV["shaft"], Y_AX - SV["shaft"] + SV["L"]
    zw = SV["W"] / 2
    xp = xt + SV["H"] - SV["tab_h"] + SV["tab_t"]                       # tab underside = plate face
    foot = span(xt + 1, xt + SV["H"], y0 - 4, y1 + 12, Z_DECK_TOP, Z_DECK_TOP + SERVO_FOOT)
    plate = span(xp, xp + 5, y0 - SV["tab_ext"] - 1.5, y1 + SV["tab_ext"] + 1.5, Z_DECK_TOP, Z_AX + zw + 7)
    plate = plate.cut(span(xp - 1, xp + 6, y0 - 1, y1 + 1, Z_AX - zw, Z_AX + zw + 1))   # window for the body
    m = foot.union(plate)
    for ry in (y0 - SV["tab_ext"] - 1.5, y1 + SV["tab_ext"] - 3.5):    # 2 ribs behind the plate
        rib = (cq.Workplane("XZ").polyline([(xp + 5, Z_DECK_TOP + SERVO_FOOT), (xt + SV["H"], Z_DECK_TOP + SERVO_FOOT),
                                            (xp + 5, Z_AX + zw)]).close().extrude(-5).translate((0, ry, 0)))
        m = m.union(rib)
    yc = (y0 + y1) / 2
    for hy in (yc - SV["hole_l"] / 2, yc + SV["hole_l"] / 2):           # 4 tab screws (M4 + nut behind the plate)
        for hz in (Z_AX - SV["hole_w"] / 2, Z_AX + SV["hole_w"] / 2):
            m = m.cut(cyl_x(4.5, xp - 1, xp + 6, hy, hz))
    return drill(m, servo_mount_holes(True), 4.5, Z_DECK_TOP - 1, SERVO_FOOT + 2)


def bed_rest(x, y):
    """Post the bed lies on when it is flat; an M4 bolt comes up through the deck into a nut trapped at the top."""
    post = cyl_z(20, 17, x, y, Z_DECK_TOP).cut(cyl_z(4.5, 17, x, y, Z_DECK_TOP))
    pocket = cq.Workplane("XY").polygon(6, 7.4 / math.cos(math.pi / 6)).extrude(4).translate((x, y, Z_DECK_TOP + 13))
    return post.cut(pocket)


sb = servo_body_front()
add("tilt", "servo_front", sb, SERVO_BLUE)
add("tilt", "servo_rear", mirror_x(sb), SERVO_BLUE)
sm = servo_mount_front()
add("tilt", "servo_mount_front", sm, PRINT, fab=True)
add("tilt", "servo_mount_rear", mirror_x(sm), PRINT, fab=True)
for i, (x, y) in enumerate(REST_PTS):
    add("tilt", f"bed_rest_{i}", bed_rest(x, y), PRINT)
    add("tilt", f"bed_rest_pad_{i}", cyl_z(18, 2, x, y, Z_DECK_TOP + 17), RUBBER)
fabricated["bed_rest"] = bed_rest(0, 0).translate((0, 0, -Z_DECK_TOP))


# ---------- Tilting bed: moving half (rotates about AXIS) ----------
SPACER_LEN = 28
ADAPTER_BOLTS = [(TILT_X - 10, Y_AX + 8), (TILT_X - 10, Y_AX + 21)]


def tilt_plate():
    zc, z0, h = Z_TILT + TILT_T / 2, Z_TILT - 1, TILT_T + 2
    p = span(-TILT_X, TILT_X, Y_AX, TILT_Y1, Z_TILT, Z_TILT_TOP)
    zone = CELL_ZONE[1] - CELL_ZONE[0]
    for cx in CELL_X:
        _, fixed_x, _ = cell_layout(cx)
        for s in (-1, 1):
            p = p.cut(slot_x(fixed_x, s * CELL_Y, z0, h, zone, 5.5))
    bolts = ADAPTER_BOLTS + [(-x, y) for x, y in ADAPTER_BOLTS]
    p = drill(p, bolts, 4.5, z0, h)
    for x in (-8, 8):                                   # zip tie that holds the load-cell bundle at the hinge
        p = p.cut(box(4, 2.5, h, x, Y_AX + 9, zc))
    return p


def cargo_plate():
    zc, z0, h = Z_CARGO + CARGO_T / 2, Z_CARGO - 1, CARGO_T + 2
    p = box(CARGO[0], CARGO[1], CARGO_T, CARGO_X, 0, zc)
    zone = CELL_ZONE[1] - CELL_ZONE[0]
    for cx in CELL_X:
        _, _, loaded_x = cell_layout(cx)
        for s in (-1, 1):
            p = p.cut(slot_x(loaded_x, s * CELL_Y, z0, h, zone, 5.5))
    return p


def horn_adapter_front():
    """Printed block on top of the tilting plate's front-right corner; the servo horn screws to its round wall."""
    x0, x1 = TILT_X, TILT_X + ADAPTER_WALL
    wall = cyl_x(34, x0, x1, Y_AX, Z_AX)                # round, so it never dips towards the deck as it turns
    flange = span(TILT_X - 20, TILT_X + 0.01, Y_AX, Y_AX + 28, Z_TILT_TOP, Z_TILT_TOP + 6)
    gusset = span(TILT_X - 8, TILT_X + 0.01, Y_AX, Y_AX + 10, Z_TILT_TOP + 6, Z_TILT_TOP + 12)
    a = wall.union(flange).union(gusset)
    for k in range(4):                                  # horn screws: 4 x M3 into 2.8 mm holes, on a 24 mm circle
        ang = math.radians(45 + 90 * k)
        a = a.cut(cyl_x(2.8, x0, x1 + 1, Y_AX + 12 * math.cos(ang), Z_AX + 12 * math.sin(ang)))
    a = a.cut(cyl_x(8, x0 - 1, x1 + 1, Y_AX, Z_AX))     # clearance for the horn's centre screw
    return drill(a, ADAPTER_BOLTS, 4.5, Z_TILT_TOP - 1, 8)


def horn_front():
    x0 = TILT_X + ADAPTER_WALL
    disc = cyl_x(HORN_D, x0, x0 + HORN_T, Y_AX, Z_AX)
    hub = cyl_x(14, x0 + HORN_T, x0 + HORN_T + HORN_HUB, Y_AX, Z_AX)
    return disc.union(hub)


def spacer_at(x, y, z0):
    return (box(SPACER_LEN, CELL[1], 5, x, y, z0 + 2.5)
            .cut(slot_x(x, y, z0 - 1, 7, CELL_ZONE[1] - CELL_ZONE[0], 5.5)))


def bed_items():
    """Every part that tips with the bed, in the flat position."""
    items = [("tilt_plate", tilt_plate(), PLYWOOD)]
    n = 0
    for cx in CELL_X:
        outer, fixed_x, loaded_x = cell_layout(cx)
        for s in (-1, 1):
            y = s * CELL_Y
            items.append((f"load_cell_{n}", box(*CELL, cx, y, Z_CELL + CELL[2] / 2), STEEL))
            items.append((f"cell_spacer_fixed_{n}", spacer_at(fixed_x, y, Z_TILT_TOP), PRINT))
            items.append((f"cell_spacer_load_{n}", spacer_at(loaded_x, y, Z_CELL + CELL[2]), PRINT))
            n += 1
    for i, (x, y) in enumerate([(-60, -50), (-60, 50), (60, -50), (60, 50)]):   # HX711s on the tilting plate
        items.append((f"hx711_{i}", box(34, 21, 1.6, x, y, Z_TILT_TOP + 0.8), PCB))
        items.append((f"hx711_chip_{i}", box(8, 6, 2, x, y, Z_TILT_TOP + 2.6), BLACK))
    ha = horn_adapter_front()
    items += [("horn_adapter_front", ha, PRINT), ("horn_adapter_rear", mirror_x(ha), PRINT),
              ("servo_horn_front", horn_front(), ALU), ("servo_horn_rear", mirror_x(horn_front()), ALU)]
    items.append(("cargo_plate", cargo_plate(), ACRYLIC))
    return items


def tipped(wp, angle):
    """The workplane turned about the hinge axis by angle degrees (positive = left edge up)."""
    return wp.rotate(AXIS[0], AXIS[1], angle) if angle else wp


BED_KG = 0.95                    # everything that tips (plates, cells, adapters, horns), estimated
SERVO_STALL_NM = 2 * 60 * 9.81 / 100    # two 60 kg.cm servos


def hinge_torque(package_kg, angle=0.0, package_h=200):
    """N.m the two servos must hold at a bed angle, for a box package_h mm tall in the middle of the bed."""
    r = math.radians(angle)
    arm = -Y_AX                                      # bed centre (Y = 0) -> hinge, mm
    lift = Z_CARGO + CARGO_T - Z_AX                  # cargo plate top above the hinge, mm
    pkg = package_kg * 9.81 * (arm * math.cos(r) - (lift + package_h / 2) * math.sin(r))
    bed = BED_KG * 9.81 * (arm * math.cos(r) - 15 * math.sin(r))
    return (pkg + bed) / 1000


BED = bed_items()
for name, wp, color in BED:
    add("payload", name, wp, color)
for name in ("tilt_plate", "cargo_plate", "horn_adapter_front", "horn_adapter_rear"):
    fabricated[name] = dict((n, w) for n, w, _ in BED)[name]
fabricated["load_cell_spacer"] = spacer_at(0, 0, 0)


# ============================== EXPORT ==============================
def build_assembly(bed_angle=0.0):
    assy = cq.Assembly(name="cargo_robot_v2")
    for gname, items in groups.items():
        sub = cq.Assembly(name=gname)
        for name, wp, color in items:
            if gname == "payload":
                wp = tipped(wp, bed_angle)
            sub.add(wp, name=name, color=cq.Color(*color))
        assy.add(sub, name=gname)
    return assy


BOM = [
    # group, item, qty, make/buy
    ("Compute", "ESP32 DevKitC 38-pin (ESP32-WROOM-32, USB) + 38-pin screw-terminal breakout board", 1, "buy (get 2)"),
    ("Compute", "MCP23017 I2C 16-bit I/O expander module (address 0x20)", 1, "buy"),
    ("Drive", "JGB37-520 12V 170 RPM gear motor with Hall encoder (6 wires)", 2, "buy"),
    ("Drive", "Steel mounting bracket for 37 mm gear motor (+ its M3 screws)", 2, "buy"),
    ("Drive", f"Wheel {WHEEL_D} mm x {WHEEL_W} mm with 6 mm D-shaft hub", 2, "buy"),
    ("Drive", f"Swivel caster {CASTER_WHEEL_D} mm, M8 threaded stem, >= 20 kg rating", 2, "buy"),
    ("Drive", "BTS7960 (IBT-2) 43 A motor driver", 2, "buy"),
    ("Sensors", "8-channel IR line sensor array, TCRT5000, digital outputs, 3.3-5 V", 1, "buy"),
    ("Sensors", "Line sensor bracket (line_sensor_bracket.stl)", 1, "3D print PETG"),
    ("Sensors", "HC-SR04 ultrasonic distance sensor", 1, "buy (get 2)"),
    ("Sensors", "Ultrasonic sensor bracket (sonar_bracket.stl)", 1, "3D print PETG"),
    ("Sensors", "VL53L0X time-of-flight distance sensor module", 1, "buy"),
    ("Sensors", "Side sensor bracket (tof_bracket.stl)", 1, "3D print PETG"),
    ("Payload", "Straight-bar load cell 10 kg, 80x12.7x12.7 mm", 4, "buy"),
    ("Payload", "HX711 module", 4, "buy"),
    ("Payload", "Load-cell spacer (load_cell_spacer.stl)", 8, "3D print PETG 100 %"),
    ("Payload", f"Cargo plate {CARGO[0]}x{CARGO[1]}x{CARGO_T} (cargo_plate.dxf)", 1, "laser-cut acrylic"),
    ("Tilting bed", f"Tilting plate {2 * TILT_X}x{TILT_Y1 - Y_AX}x{TILT_T} (tilt_plate.dxf)", 1, "laser-cut 6 mm plywood"),
    ("Tilting bed", "Servo 60 kg.cm, metal gears, 180 deg, 6-8.4 V, 25T spline, about 65x30x48 mm (e.g. DS5160)", 2,
     "buy"),
    ("Tilting bed", "25T aluminium disc servo horn, about 38 mm (often included with the servo)", 2, "buy"),
    ("Tilting bed", "Servo mount (servo_mount_front.stl + servo_mount_rear.stl)", 2, "3D print PETG"),
    ("Tilting bed", "Horn adapter (horn_adapter_front.stl + horn_adapter_rear.stl)", 2, "3D print PETG 100 %"),
    ("Tilting bed", "Bed rest post (bed_rest.stl) + 18 mm self-adhesive rubber pad 2 mm", 2, "3D print PETG"),
    ("Tilting bed", "Buck converter 10 A (e.g. 300 W XL4016 module), set to 7.4 V for the servos", 1, "buy"),
    ("Power", "3S LiPo 11.1 V 5000 mAh XT60 + balance charger + LiPo bag + voltage alarm", 1, "buy"),
    ("Power", "XL4015 5 A buck converter (5.1 V for the ESP32 and sensors)", 1, "buy"),
    ("Power", "Inline fuse holder + 10 A fuses, WAGO 221 connectors, XT60 pairs", 1, "buy"),
    ("Power", "Emergency stop 22 mm panel-mount, latching, NC contact block", 1, "buy"),
    ("Power", "Toggle switch 12 mm panel-mount, 12 V DC >= 15 A", 1, "buy"),
    ("Controls", "Push button 12 mm panel-mount, momentary, normally open (start)", 1, "buy"),
    ("Controls", "5 mm LED green + red, with panel clips, + 330 ohm resistors", 1, "buy"),
    ("Controls", "Active buzzer module, 3-pin, 3.3-5 V", 1, "buy"),
    ("Chassis", f"Bottom plate {L}x{W}x{BASE_T} (bottom_plate.dxf)", 1, "laser-cut 6 mm plywood"),
    ("Chassis", f"Top deck {L}x{W}x{DECK_T} (top_deck.dxf)", 1, "laser-cut 5 mm acrylic"),
    ("Chassis", f"M6 threaded rod, cut to {ROD_Z1 - ROD_Z0:.0f} mm", len(ROD_PTS), "cut from 1 m rod"),
    ("Chassis", "M6 nut + M6 washer", 4 * len(ROD_PTS), "buy"),
    ("Chassis", "M8 nut + M8 washer (casters)", 4, "buy"),
    ("Hardware", "M4x12 screws + nylock nuts (motor brackets, servo mounts, rest posts), M4x16 + nuts (horn "
                 "adapters), M4/M5x16 (load cells), M3 screws + nuts (sensor brackets), M2.5/M3 for the boards, "
                 "VHB tape, velcro strap, zip ties", 1, "buy"),
    ("Hardware", "Resistors 1k x1, 2k x1, 10k x5, 2.2k x1, 4.7k x2, 100 nF x1, small perfboard x2", 1, "buy"),
]


def main():
    out = Path(__file__).parent / "output"
    for sub in ("parts", "dxf", "views"):
        (out / sub).mkdir(parents=True, exist_ok=True)

    for angle, fname in ((0, "robot_assembly.step"), (TIP_ANGLE, "robot_assembly_tipped.step")):
        assy = build_assembly(angle)
        step_path = out / fname
        if hasattr(assy, "export"):
            assy.export(str(step_path))
        else:
            assy.save(str(step_path))
        print("wrote", step_path)

    for name, wp in fabricated.items():
        cq.exporters.export(wp, str(out / "parts" / f"{name}.step"))
        cq.exporters.export(wp, str(out / "parts" / f"{name}.stl"), tolerance=0.05, angularTolerance=0.1)
    print("wrote", len(fabricated), "parts")

    for name, zc in (("bottom_plate", Z_BASE + BASE_T / 2), ("top_deck", Z_DECK + DECK_T / 2),
                     ("tilt_plate", Z_TILT + TILT_T / 2), ("cargo_plate", Z_CARGO + CARGO_T / 2)):
        cq.exporters.export(fabricated[name].translate((0, 0, -zc)).section(), str(out / "dxf" / f"{name}.dxf"))
    print("wrote DXFs")

    comp = build_assembly(0).toCompound()
    views = {"iso": (1.2, -1.0, 0.8), "front": (1, 0, 0), "side": (0, -1, 0), "top": (0, 0, 1)}
    for vname, d in views.items():
        cq.exporters.export(comp, str(out / "views" / f"{vname}.svg"),
                            opt={"width": 1400, "height": 1000, "marginLeft": 60, "marginTop": 60,
                                 "projectionDir": d, "showAxes": False, "showHidden": False,
                                 "strokeWidth": 0.5})
    print("wrote SVG views")

    with open(out / "bom.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Group", "Item", "Qty", "Make/Buy"])
        w.writerows(BOM)
    print("wrote bom.csv")


if __name__ == "__main__":
    main()
