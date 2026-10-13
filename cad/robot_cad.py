"""
Parametric CAD model - Phase II autonomous cargo robot, DESIGN v3 (v2 + rear pusher).

Run:   python robot_cad.py          (needs: pip install cadquery)
Writes ./output/
  robot_assembly.step   full coloured assembly -> SolidWorks / Fusion 360 / Onshape / FreeCAD / Inventor
  parts/*.step, *.stl   the custom-made parts
  dxf/*.dxf             the three plates, ready for the laser cutter (every hole already in them)
  views/*.svg           line drawings
  bom.csv               bill of materials

New in v3 - the rear pusher, which slides the cargo off the back of the robot onto a conveyor:
  - a blade (aluminium angle) spans the bed 3 mm above the cargo plate, so the load cells never feel it
  - it rides on two 8 mm rods that run just outside the deck edges, on printed carriages with LM8UU bearings
  - a T8 lead screw on the right side moves it, turned by a third JGB37 gear motor through a third BTS7960
  - two limit switches (parked / fully out) end each stroke
  - it bolts on: the only change to the v2 plates is 5 extra 4.5 mm holes in the top deck
  The untouched v2 design is kept in ../backup_v2.

Design choices that keep it simple to build:
  - frame: 6 x M6 threaded rod + nuts, instead of 6 printed pillars with heat-set inserts
  - casters: 2 bought swivel casters with an M8 threaded stem - the height is set with the nuts,
    so getting all 4 wheels onto the floor takes a spanner, not shims
  - motor brackets: the standard steel bracket sold for JGB37 motors, instead of printed ones
  - the holes in the acrylic parts are laser-cut (load-cell slots that fit any common bar cell,
    the 22 mm E-stop hole, the switch hole) - only the lidar's 4 small screw holes are drilled by hand
  - E-stop and master switch mount through the deck, so their wiring stays inside
  - lower electronics bay (100 mm) -> lower centre of gravity, shorter wires
  - battery lies across the robot and slides out sideways for charging
  - printed parts: camera mount, load-cell spacers, and the pusher's 2 carriages, 3 rod holders, 1 drive bracket

Axes: X = forward, Y = left, Z = up. Origin = floor under the middle of the drive axle. Units: mm.
Bought parts are simplified envelopes with the right outer sizes (good for layout and clearance checks).
"""
import csv
import math
from pathlib import Path

import cadquery as cq

# ============================== PARAMETERS ==============================
L, W = 400, 300                  # plate size (X, Y)
BASE_T, DECK_T, CARGO_T = 6, 5, 5    # bottom plate 6 mm plywood; deck and cargo plate 5 mm acrylic
BAY_H = 100                      # clear height of the electronics bay

WHEEL_D, WHEEL_W = 100, 30       # drive wheels
AXLE_Z = WHEEL_D / 2
MOTOR_DROP = 22                  # stock steel bracket: motor axis -> plate underside (casters absorb +-5 mm)
MOTOR_FACE_Y = W / 2 - 5         # outer face of the bracket's vertical plate
WHEEL_Y = W / 2 + 15             # wheel centre-plane

CASTER_X = 160                   # swivel-caster stems at X = +-160
CASTER_WHEEL_D, CASTER_WHEEL_W = 50, 20
CASTER_TRAIL = 15                # wheel axle sits this far behind the stem

ROD_PTS = [(x, s * (W / 2 - 15)) for x in (-180, 60, 185) for s in (-1, 1)]   # M6 threaded rods

CAM_TILT = 35                    # line camera tilt below horizontal (deg)
LIDAR_X = 140                    # lidar centre on the deck
ESTOP_POS = (150, 110)           # 22 mm E-stop, front-left of the deck
SWITCH_POS = (150, -110)         # 12 mm toggle switch, front-right of the deck

CELL = (80, 12.7, 12.7)          # straight-bar load cell
CELL_X = (-150, 30)              # centres of the two cell columns
CELL_Y = 110
CELL_ZONE = (5, 23)              # screw-hole centres lie this far from a cell's end (fits common cells)
CARGO = (270, 280)
CARGO_X = -60                    # cargo plate centre X (rear 2/3 of the deck)

PI_POS = (120, 65)               # Raspberry Pi board centre (USB/Ethernet end faces -X)
PI_HOLES = [(PI_POS[0] + dx, PI_POS[1] + dy) for dx in (39, -19) for dy in (-24.5, 24.5)]
BATTERY = (50, 155, 35)          # 3S 5000 mAh LiPo lying across the robot (X, Y, Z)
CAM_FOOT_HOLES = [(184, -13), (184, 13)]

# ---- rear pusher (v3): the drive side is the right side (-Y), the E-stop side stays clear ----
RAIL_Y = W / 2 + 13              # 8 mm guide rods, one per side, just outside the deck edges
RAIL_LEN = 400                   # stock rod length, not cut
SCREW_RISE = 38                  # the T8 lead screw runs this far above the right-hand rod
SCREW_LEN = 310                  # cut from a 350 mm screw; its rear end is left free
BLADE = (25, 2)                  # push blade: aluminium angle 25 x 25 x 2
BLADE_LEN = 296                  # a little wider than the cargo plate, and short of the lead screw
BLADE_GAP = 3                    # blade's lower edge above the cargo plate (it must never touch the plate)
PUSH_HOME, PUSH_END = 62, -L / 2 # X of the pushing face: parked / fully out (flush with the rear edge)
PUSH_OFFSET = 28                 # pushing face -> centre of the carriage's lead-screw nut
BEARING_GAP = 46                 # between the centres of a carriage's two LM8UU bearings
HOLDER_BOLT = 38                 # rod holder: outer end face -> its M4 bolt
FRONT_HOLDER_X = 180             # outer end face of the front-left rod holder
BRACKET_BOLTS = (94, 170)        # X of the drive bracket's two M4 bolts
PUSHER_HOLES = ([(-L / 2 + HOLDER_BOLT, s * (W / 2 - 10)) for s in (-1, 1)]       # rear rod holders
                + [(FRONT_HOLDER_X - HOLDER_BOLT, W / 2 - 10)]                      # front-left rod holder
                + [(x, -(W / 2 - 11)) for x in BRACKET_BOLTS])                      # drive bracket (front right)

# ---- derived Z levels ----
Z_BASE = AXLE_Z + MOTOR_DROP     # underside of bottom plate = 72
Z_LOWER = Z_BASE + BASE_T        # top of bottom plate = 78
Z_DECK = Z_LOWER + BAY_H         # underside of deck = 178
Z_DECK_TOP = Z_DECK + DECK_T     # 183
Z_CELL = Z_DECK_TOP + 5          # load cells sit on 5 mm spacers
Z_CARGO = Z_CELL + CELL[2] + 5   # underside of cargo plate
RAIL_Z = Z_DECK                  # guide rods level with the deck's underside
SCREW_Z = RAIL_Z + SCREW_RISE    # 216
Z_BLADE = Z_CARGO + CARGO_T + BLADE_GAP          # lower edge of the blade = 213.7
Z_BLADE_SEAT = Z_BLADE + BLADE[0] - BLADE[1]     # underside of the blade's top leg (it rests on the carriages)


def cell_layout(cx):
    """(outer, x of the fixed-end zone centre, x of the loaded-end zone centre) for a cell centred at cx.
    The fixed end (screwed to the deck) faces the outside of the bed; the loaded end carries the cargo plate."""
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
BRASS = (0.80, 0.65, 0.30, 1)
RUBBER = (0.08, 0.08, 0.08, 1)
PCB = (0.10, 0.45, 0.20, 1)
PCB_BLUE = (0.10, 0.25, 0.60, 1)
BLACK = (0.15, 0.15, 0.15, 1)
RED = (0.85, 0.10, 0.10, 1)
YELLOW = (0.95, 0.80, 0.10, 1)
BATT = (0.20, 0.30, 0.80, 1)
MOTOR = (0.35, 0.35, 0.38, 1)
ALU = (0.80, 0.82, 0.86, 1)


# ============================== HELPERS ==============================
def box(sx, sy, sz, cx=0, cy=0, cz=0):
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def box2(x0, x1, y0, y1, z0, z1):
    """Box given by its lowest and highest corner."""
    return box(x1 - x0, y1 - y0, z1 - z0, (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)


def cyl_z(d, h, cx=0, cy=0, z0=0):
    return cq.Workplane("XY").circle(d / 2).extrude(h).translate((cx, cy, z0))


def cyl_y(d, y0, y1, cx=0, cz=0):
    """Cylinder along +Y from y0 to y1 (y1 > y0)."""
    return (cq.Workplane("XY").circle(d / 2).extrude(y1 - y0)
            .rotate((0, 0, 0), (1, 0, 0), -90).translate((cx, y0, cz)))


def cyl_x(d, x0, x1, cy=0, cz=0):
    """Cylinder along +X from x0 to x1 (x1 > x0)."""
    return (cq.Workplane("XY").circle(d / 2).extrude(x1 - x0)
            .rotate((0, 0, 0), (0, 1, 0), 90).translate((x0, cy, cz)))


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


def posts(pts, d, h, z0):
    return cq.Workplane("XY").pushPoints(pts).circle(d / 2).extrude(h).translate((0, 0, z0))


def standoffs(cx, cy, sx, sy, h, z0, inset=3.5, d=5):
    pts = [(cx + i * (sx / 2 - inset), cy + j * (sy / 2 - inset)) for i in (-1, 1) for j in (-1, 1)]
    return posts(pts, d, h, z0)


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
    p = drill(p, PI_HOLES, 2.7, z0, h)
    p = drill(p, CAM_FOOT_HOLES, 3.4, z0, h)
    for sx in (-31, 31):                         # 2 velcro straps around the battery
        for sy in (-45, 45):
            p = p.cut(box(4, 22, h, sx, sy, zc))
    for sy in (-70, 70):                         # motor + encoder cables come up here
        p = p.cut(box(20, 12, h, 40, sy, zc))
    return p


def top_deck():
    zc, z0, h = Z_DECK + DECK_T / 2, Z_DECK - 1, DECK_T + 2
    p = box(L, W, DECK_T, 0, 0, zc)
    p = drill(p, ROD_PTS, 6.5, z0, h)
    p = drill(p, [ESTOP_POS], 22.5, z0, h)
    p = drill(p, [SWITCH_POS], 12.2, z0, h)
    p = drill(p, PUSHER_HOLES, 4.5, z0, h)       # pusher: 3 rod holders + drive bracket (M4)
    p = p.cut(box(12, 80, h, 83, 0, zc))         # wiring pass-through (lidar USB, load-cell wires)
    zone = CELL_ZONE[1] - CELL_ZONE[0]
    for cx in CELL_X:
        _, fixed_x, _ = cell_layout(cx)
        for s in (-1, 1):
            p = p.cut(slot_x(fixed_x, s * CELL_Y, z0, h, zone, 5.5))
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
for tag, x, y in (("L", -150, 60), ("R", -150, -60), ("P", 120, -65)):     # P = pusher motor driver
    add("electronics", f"BTS7960_{tag}_pcb", box(50, 50, 1.6, x, y, z + 1.0), PCB)
    add("electronics", f"BTS7960_{tag}_heatsink", box(40, 40, 20, x, y, z + 1.8 + 10), BLACK)
px, py = PI_POS
add("electronics", "rpi_standoffs", posts(PI_HOLES, 5, 8, z), BRASS)
add("electronics", "rpi_pcb", box(85, 56, 1.6, px, py, z + 8.8), PCB)
add("electronics", "rpi_ports", box(21, 53, 15, px - 85 / 2 + 10.5, py, z + 9.6 + 7.5), STEEL)
add("electronics", "rpi_fan", box(30, 30, 10, px + 15, py, z + 9.6 + 5), BLACK)
add("electronics", "buck_12V_5V", box(51, 26, 1.6, -60, 85, z + 1.0), PCB_BLUE)
add("electronics", "buck_inductor", cyl_z(20, 12, -60, 85, z + 1.8), BLACK)
add("electronics", "power_bus_wagos", box(60, 40, 20, -60, -85, z + 10), BLACK)
add("electronics", "inline_fuse_holder", box(40, 15, 15, 45, -110, z + 7.5), RED)


# ---------- Line camera (front, tilted down) ----------
CAM_POS = (L / 2 + 12, 0, Z_LOWER + 18)


def camera_mount():
    back = box(4, 40, 20, L / 2 - 2, 0, Z_LOWER + 10)
    arm = box(10, 20, 4, L / 2 + 5, 0, CAM_POS[2])
    holder = box(4, 32, 32).rotate((0, 0, 0), (0, 1, 0), CAM_TILT).translate(CAM_POS)
    foot = box(24, 40, 4, L / 2 - 12, 0, Z_LOWER + 2)
    m = back.union(arm).union(holder).union(foot)
    return drill(m, CAM_FOOT_HOLES, 3.4, Z_LOWER - 1, 6)


def camera():
    """32 x 32 mm USB (UVC) camera board with an M12 wide-angle lens."""
    board = box(1.6, 32, 32, 2.8, 0, 0)
    lens = box(8, 16, 16, 7.6, 0, 0).union(cyl_z(14, 10).rotate((0, 0, 0), (0, 1, 0), 90).translate((11.6, 0, 0)))
    place = lambda s: s.rotate((0, 0, 0), (0, 1, 0), CAM_TILT).translate(CAM_POS)
    return place(board), place(lens)


add("sensors", "camera_mount", camera_mount(), PRINT, fab=True)
cam_board, cam_lens = camera()
add("sensors", "usb_camera_pcb", cam_board, PCB)
add("sensors", "usb_camera_lens", cam_lens, BLACK)

# ---------- Lidar (RPLIDAR A1M8 envelope) on the deck ----------
add("sensors", "lidar_standoffs", standoffs(LIDAR_X, 0, 80, 56, 25, Z_DECK_TOP, inset=0, d=6), BRASS)
add("sensors", "rplidar_base", box(97, 70, 25, LIDAR_X, 0, Z_DECK_TOP + 25 + 12.5), BLACK)
add("sensors", "rplidar_head", cyl_z(70, 30, LIDAR_X + 13, 0, Z_DECK_TOP + 50), (0.25, 0.25, 0.28, 1))

# ---------- Safety controls: panel-mounted through the deck ----------
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

# ---------- Payload: 4 load cells + cargo plate ----------
SPACER_LEN = 28


def spacer_at(x, y, z0):
    return (box(SPACER_LEN, CELL[1], 5, x, y, z0 + 2.5)
            .cut(slot_x(x, y, z0 - 1, 7, CELL_ZONE[1] - CELL_ZONE[0], 5.5)))


n = 0
for cx in CELL_X:
    outer, fixed_x, loaded_x = cell_layout(cx)
    for s in (-1, 1):
        y = s * CELL_Y
        add("payload", f"load_cell_{n}", box(*CELL, cx, y, Z_CELL + CELL[2] / 2), STEEL)
        add("payload", f"cell_spacer_fixed_{n}", spacer_at(fixed_x, y, Z_DECK_TOP), PRINT)
        add("payload", f"cell_spacer_load_{n}", spacer_at(loaded_x, y, Z_CELL + CELL[2]), PRINT)
        n += 1
fabricated["load_cell_spacer"] = spacer_at(0, 0, 0)

for i, (x, y) in enumerate([(-90, -45), (-90, 45), (-30, -45), (-30, 45)]):   # HX711s under the cargo plate
    add("payload", f"hx711_{i}", box(34, 21, 1.6, x, y, Z_DECK_TOP + 0.8), PCB)
    add("payload", f"hx711_chip_{i}", box(8, 6, 2, x, y, Z_DECK_TOP + 2.6), BLACK)
add("payload", "cargo_plate", cargo_plate(), ACRYLIC, fab=True)


# ---------- Rear pusher (v3): a blade on a lead screw slides the cargo off the rear ----------
# The left-hand version of every handed part is modelled; the right-hand one is its mirror image.
X_NUT_HOME = PUSH_HOME + PUSH_OFFSET             # lead-screw nut (= rear bearing) with the blade parked
X_BEAM_HOME = X_NUT_HOME + BEARING_GAP + 16      # front end of a carriage with the blade parked
X_KFL = X_NUT_HOME + 5.5                         # rear face of the lead screw's bearing block
X_PLATE = X_KFL + 40.5                           # rear face of the motor wall
X_LUG = X_BEAM_HOME + 16                         # rear face of the drive bracket's rod holder
HOME_REACH = FRONT_HOLDER_X - 15 - X_BEAM_HOME   # from the left carriage's front end to the HOME switch's plunger
SCREW_X1 = X_KFL + 25.5                          # front end of the lead screw, inside the coupler
MOTOR_DY = -7                                    # JGB37 gearbox: its centre is 7 mm inboard of the output shaft


def pusher_carriage(face, finger=False):
    """Left carriage with the blade's pushing face at X = face: two LM8UU bearings on the rod (pressed in
    from the rear, stopped by the front cap), a seat for the T8 nut and an arm for the blade.
    finger: the finger on the front end that presses the HOME switch. Only the left carriage has it;
    on the right it would hit the drive bracket's walls."""
    xn = face + PUSH_OFFSET
    x0, x1 = xn - 13, xn + BEARING_GAP + 16
    c = (box2(x0, x1, RAIL_Y - 10.5, RAIL_Y + 10.5, RAIL_Z - 10.5, RAIL_Z + 10.5)             # bearing beam
         .union(box2(x0, xn + 1, RAIL_Y - 11, RAIL_Y + 12, RAIL_Z + 10.5, Z_BLADE_SEAT))      # nut block
         .union(box2(face + BLADE[1], xn - 3, BLADE_LEN / 2 - 26, RAIL_Y + 12,                # arm under the blade's top leg
                     Z_BLADE_SEAT - 12, Z_BLADE_SEAT)))
    if finger:
        c = c.union(box2(x1 - 12, x1 + HOME_REACH, RAIL_Y - 4, RAIL_Y + 5, RAIL_Z + 10.5, RAIL_Z + 19))
    c = c.cut(cyl_x(15.2, x0 - 1, x1 - 3, RAIL_Y, RAIL_Z)).cut(cyl_x(10, x1 - 4, x1 + 1, RAIL_Y, RAIL_Z))
    c = c.cut(cyl_x(10.6, x0 - 1, xn + 2, RAIL_Y, SCREW_Z))                                   # T8 nut body
    for dy in (-5.66, 5.66):                                                                  # nut flange: 4 x M3 on a 16 mm circle
        for dz in (-5.66, 5.66):
            c = c.cut(cyl_x(2.6, x0 + 2, xn + 2, RAIL_Y + dy, SCREW_Z + dz))
    return drill(c, [(x, y) for x, y in blade_bolts(face) if y > 0], 4.3, Z_BLADE_SEAT - 13, 14)


def blade_bolts(face):
    """XY of the 4 M4 bolts that hold the blade on the two carriage arms."""
    return [(face + BLADE[0] / 2 + 1, s * (BLADE_LEN / 2 - d)) for s in (-1, 1) for d in (8, 20)]


def pusher_blade(face):
    """Aluminium angle: one leg hangs down as the pushing face, the other lies on top of the carriage arms.
    It stops short of the lead screw, so it is a plain length of angle with 4 holes."""
    a, t = BLADE
    b = (box2(face, face + t, -BLADE_LEN / 2, BLADE_LEN / 2, Z_BLADE, Z_BLADE + a)            # pushing face
         .union(box2(face, face + a, -BLADE_LEN / 2, BLADE_LEN / 2, Z_BLADE_SEAT, Z_BLADE + a)))
    return drill(b, blade_bolts(face), 4.3, Z_BLADE_SEAT - 1, t + 2)


def lm8uu(xc, y):
    return cyl_x(15, xc - 12, xc + 12, y, RAIL_Z).cut(cyl_x(8, xc - 13, xc + 13, y, RAIL_Z))


def t8_nut(xn):
    """Brass flange nut; the flange sits on the front face of the carriage's nut block."""
    return (cyl_x(22, xn + 1, xn + 4.5, -RAIL_Y, SCREW_Z).union(cyl_x(10.2, xn - 10.5, xn + 1, -RAIL_Y, SCREW_Z))
            .cut(cyl_x(8, xn - 11.5, xn + 5.5, -RAIL_Y, SCREW_Z)))


def pusher_moving(face=PUSH_HOME):
    """(name, shape, colour) of everything that travels with the blade, pushing face at X = face."""
    xn = face + PUSH_OFFSET
    parts = [("pusher_blade", pusher_blade(face), ALU),
             ("pusher_carriage_L", pusher_carriage(face, finger=True), PRINT),
             ("pusher_carriage_R", pusher_carriage(face).mirror("XZ"), PRINT),
             ("pusher_nut", t8_nut(xn), BRASS)]
    for side, s in (("L", 1), ("R", -1)):
        for i, xc in enumerate((xn, xn + BEARING_GAP)):
            parts.append((f"pusher_bearing_{side}{i}", lm8uu(xc, s * RAIL_Y), STEEL))
    return parts


def rail_holder(x0):
    """Left rod holder, outer end face at X = x0, foot pointing towards +X. It clips over the deck edge
    (block outside, strip on top, lip underneath) and is locked by one M4 bolt through the deck."""
    edge = W / 2
    h = (box2(x0, x0 + 14, edge, RAIL_Y + 13, RAIL_Z - 10, RAIL_Z + 13)                       # grips the rod
         .union(box2(x0, x0 + 14, edge - 8, edge, RAIL_Z - 8, RAIL_Z))                        # lip under the deck
         .union(box2(x0, x0 + 28, edge - 7.5, edge, Z_DECK_TOP, Z_DECK_TOP + 8))              # strip: passes the frame-rod nut
         .union(box2(x0 + 28, x0 + 48, edge - 18, edge, Z_DECK_TOP, Z_DECK_TOP + 8)))         # bolt pad
    h = h.cut(cyl_x(8.1, x0 - 1, x0 + 15, RAIL_Y, RAIL_Z))
    h = drill(h, [(x0 + HOLDER_BOLT, edge - 10)], 4.5, Z_DECK_TOP - 1, 10)
    h = drill(h, [(x0 + 7, RAIL_Y)], 2.6, RAIL_Z - 11, 8)                                     # M3 screw pinches the rod
    return drill(h, [(x0 + 6.9, RAIL_Y + 0.5 + d) for d in (-4.75, 4.75)], 1.7, RAIL_Z + 7, 7)   # limit switch on top (M2)


def drive_bracket():
    """Drive bracket, left-hand version: a base bar on the deck, a wall for the lead screw's bearing block,
    a wall for the motor and a holder for the front end of the rod.
    The outer half of each wall starts above the rod so the carriage can pass underneath."""
    edge, zt = W / 2, Z_DECK_TOP
    y_in, y_out, z_pass = edge - 22, RAIL_Y + 14, RAIL_Z + 13
    b = (box2(X_KFL - 7.5, X_LUG + 10, y_in, edge, zt, zt + 8)                                # base bar
         .union(box2(X_KFL + 5, X_KFL + 11, y_in, edge, zt + 8, SCREW_Z + 26))                # bearing wall
         .union(box2(X_KFL + 5, X_KFL + 11, edge, y_out, z_pass, SCREW_Z + 26))
         .union(box2(X_PLATE, X_PLATE + 4, y_in, edge, zt + 8, SCREW_Z + 22))                 # motor wall
         .union(box2(X_PLATE, X_PLATE + 4, edge, y_out, z_pass, SCREW_Z + 22))
         .union(box2(X_KFL + 11, X_PLATE, y_in, y_in + 4, zt + 8, SCREW_Z + 22))              # web joining the walls
         .union(box2(X_LUG - 4, X_LUG + 10, y_in, edge, zt, RAIL_Z + 17.5))                   # rod holder: on the deck,
         .union(box2(X_LUG, X_LUG + 10, edge, RAIL_Y + 13, RAIL_Z - 10, RAIL_Z + 17.5))       #   outside the edge,
         .union(box2(X_LUG, X_LUG + 10, edge - 8, edge, RAIL_Z - 8, RAIL_Z)))                 #   lip under the deck
    b = b.cut(cyl_x(25, X_KFL + 4, X_KFL + 12, RAIL_Y, SCREW_Z))                              # bearing block's boss
    for dz in (-18.5, 18.5):                                                                  # its 2 bolts (M4)
        b = b.cut(cyl_x(4.5, X_KFL + 4, X_KFL + 12, RAIL_Y, SCREW_Z + dz))
    b = b.cut(cyl_x(13, X_PLATE - 1, X_PLATE + 5, RAIL_Y, SCREW_Z))                           # motor shaft and boss
    for a in (90, 150, 210, 270):          # motor face: M3 on a 31 mm circle (the 2 holes next to the shaft are
                                           # left out: screw heads there would rub on the coupler)
        r = math.radians(a)
        b = b.cut(cyl_x(3.3, X_PLATE - 1, X_PLATE + 5, RAIL_Y + MOTOR_DY + 15.5 * math.cos(r),
                        SCREW_Z + 15.5 * math.sin(r)))
    b = b.cut(cyl_x(8.1, X_LUG - 1, X_LUG + 11, RAIL_Y, RAIL_Z))
    b = drill(b, [(x, edge - 11) for x in BRACKET_BOLTS], 4.5, zt - 1, 15)
    return drill(b, [(X_LUG + 5, RAIL_Y)], 2.6, RAIL_Z - 11, 8)                               # M3 screw pinches the rod


def limit_switch(x0, x1, y0, y1, z0, z1, plunger):
    """Subminiature lever microswitch (20 x 10 x 6.4 mm body) with its plunger pressed in."""
    return box2(x0, x1, y0, y1, z0, z1).union(box2(*plunger))


for side, s in (("L", 1), ("R", -1)):
    add("pusher", f"pusher_rail_{side}", cyl_x(8, -L / 2, -L / 2 + RAIL_LEN, s * RAIL_Y, RAIL_Z), STEEL)

holder_b = rail_holder(-L / 2)                                       # rear left
holder_a = holder_b.mirror("XZ")                                     # rear right, and (turned round) front left
add("pusher", "pusher_rail_holder_rear_L", holder_b, PRINT)
add("pusher", "pusher_rail_holder_rear_R", holder_a, PRINT)
add("pusher", "pusher_rail_holder_front_L",
    holder_a.rotate((0, 0, 0), (0, 0, 1), 180).translate((FRONT_HOLDER_X - L / 2, 0, 0)), PRINT)
fabricated["pusher_rail_holder_A"] = holder_a
fabricated["pusher_rail_holder_B"] = holder_b

add("pusher", "pusher_drive_bracket", drive_bracket().mirror("XZ"), PRINT, fab=True)
add("pusher", "pusher_lead_screw", cyl_x(8, SCREW_X1 - SCREW_LEN, SCREW_X1, -RAIL_Y, SCREW_Z), STEEL)
add("pusher", "pusher_bearing_block_KFL08",
    (box2(X_KFL, X_KFL + 5, RAIL_Y - 13.5, RAIL_Y + 13.5, SCREW_Z - 24, SCREW_Z + 24)
     .union(cyl_x(24, X_KFL + 5, X_KFL + 13, RAIL_Y, SCREW_Z))
     .cut(cyl_x(8, X_KFL - 1, X_KFL + 14, RAIL_Y, SCREW_Z))).mirror("XZ"), ALU)
add("pusher", "pusher_coupler",
    (cyl_x(19, X_KFL + 14, X_KFL + 39, RAIL_Y, SCREW_Z)
     .cut(cyl_x(8, X_KFL + 13, X_KFL + 26, RAIL_Y, SCREW_Z))
     .cut(cyl_x(6, X_KFL + 26, X_KFL + 40, RAIL_Y, SCREW_Z))).mirror("XZ"), ALU)
add("pusher", "pusher_motor_shaft", cyl_x(6, X_KFL + 26.5, X_PLATE + 4, -RAIL_Y, SCREW_Z), STEEL)
add("pusher", "pusher_gearmotor",
    (cyl_x(37, X_PLATE + 4, X_PLATE + 30, RAIL_Y + MOTOR_DY, SCREW_Z)
     .union(cyl_x(35, X_PLATE + 30, X_PLATE + 62, RAIL_Y + MOTOR_DY, SCREW_Z))).mirror("XZ"), MOTOR)
add("pusher", "pusher_switch_home",                                  # on the front-left rod holder; the left carriage's finger presses it
    limit_switch(FRONT_HOLDER_X - 14.2, FRONT_HOLDER_X - 4, RAIL_Y - 9.4, RAIL_Y + 10.4, RAIL_Z + 13, RAIL_Z + 19.4,
                 (FRONT_HOLDER_X - 15, FRONT_HOLDER_X - 14.2, RAIL_Y - 1, RAIL_Y + 2, RAIL_Z + 14.7, RAIL_Z + 17.7)),
    BLACK)
add("pusher", "pusher_switch_end",                                   # on the rear-right rod holder; the right carriage presses it
    limit_switch(-L / 2 + 4, -L / 2 + 14.2, RAIL_Y - 9.4, RAIL_Y + 10.4, RAIL_Z + 13, RAIL_Z + 19.4,
                 (-L / 2 + 14.2, -L / 2 + 15, RAIL_Y - 1, RAIL_Y + 2, RAIL_Z + 14.7, RAIL_Z + 17.7)).mirror("XZ"),
    BLACK)

MOVING = [name for name, _, _ in pusher_moving()]
for name, wp, color in pusher_moving():
    add("pusher", name, wp, color, fab=name.startswith(("pusher_blade", "pusher_carriage")))


# ============================== EXPORT ==============================
def build_assembly(push_face=PUSH_HOME):
    """The whole robot. push_face = X of the blade's pushing face (PUSH_HOME = parked ... PUSH_END = fully out)."""
    moved = {} if push_face == PUSH_HOME else {name: wp for name, wp, _ in pusher_moving(push_face)}
    assy = cq.Assembly(name="cargo_robot_v3")
    for gname, items in groups.items():
        sub = cq.Assembly(name=gname)
        for name, wp, color in items:
            sub.add(moved.get(name, wp), name=name, color=cq.Color(*color))
        assy.add(sub, name=gname)
    return assy


BOM = [
    # group, item, qty, make/buy
    ("Compute", "Raspberry Pi 5 8GB + Active Cooler + 64GB A2 microSD (x2)", 1, "buy"),
    ("Drive", "JGB37-520 12V 170 RPM gear motor with Hall encoder (6 wires)", 2, "buy"),
    ("Drive", "Steel mounting bracket for 37 mm gear motor (+ its M3 screws)", 2, "buy"),
    ("Drive", f"Wheel {WHEEL_D} mm x {WHEEL_W} mm with 6 mm D-shaft hub", 2, "buy"),
    ("Drive", f"Swivel caster {CASTER_WHEEL_D} mm, M8 threaded stem, >= 20 kg rating", 2, "buy"),
    ("Drive", "BTS7960 (IBT-2) 43 A motor driver", 2, "buy"),
    ("Sensors", "RPLIDAR A1M8 (with USB adapter)", 1, "buy"),
    ("Sensors", "USB (UVC) camera board 32x32 mm, wide-angle M12 lens", 1, "buy"),
    ("Sensors", "Camera mount (camera_mount.stl)", 1, "3D print PETG"),
    ("Payload", "Straight-bar load cell 10 kg, 80x12.7x12.7 mm", 4, "buy"),
    ("Payload", "HX711 module", 4, "buy"),
    ("Payload", "Load-cell spacer (load_cell_spacer.stl)", 8, "3D print PETG 100 %"),
    ("Payload", f"Cargo plate {CARGO[0]}x{CARGO[1]}x{CARGO_T} (cargo_plate.dxf), left bare so the cargo can slide", 1,
     "laser-cut acrylic"),
    ("Pusher", "JGB37-520 12 V gear motor, about 110 RPM, 6 mm D-shaft (no encoder needed)", 1, "buy"),
    ("Pusher", "BTS7960 (IBT-2) 43 A motor driver", 1, "buy"),
    ("Pusher", f"T8 lead screw, 8 mm lead, 350 mm, with brass flange nut - cut to {SCREW_LEN} mm", 1, "buy"),
    ("Pusher", f"Smooth steel rod 8 mm x {RAIL_LEN} mm (linear shaft), not cut", 2, "buy"),
    ("Pusher", "LM8UU linear ball bearing", 4, "buy"),
    ("Pusher", "KFL08 flange bearing block, 8 mm bore", 1, "buy"),
    ("Pusher", "Flexible shaft coupler 6 mm to 8 mm (D19 x L25)", 1, "buy"),
    ("Pusher", "Lever microswitch, subminiature 20x10x6.4 mm (KW11 / SS-5GL type)", 2, "buy"),
    ("Pusher", f"Aluminium angle {BLADE[0]}x{BLADE[0]}x{BLADE[1]} mm, cut to {BLADE_LEN} mm (pusher_blade.step)", 1,
     "cut from 1 m bar"),
    ("Pusher", "Carriage (pusher_carriage_L.stl, pusher_carriage_R.stl)", 2, "3D print PETG"),
    ("Pusher", "Rod holder (pusher_rail_holder_A.stl x 2, pusher_rail_holder_B.stl x 1)", 3, "3D print PETG"),
    ("Pusher", "Drive bracket (pusher_drive_bracket.stl)", 1, "3D print PETG"),
    ("Power", "3S LiPo 11.1 V 5000 mAh XT60 + balance charger + LiPo bag + voltage alarm", 1, "buy"),
    ("Power", "XL4015 5 A buck converter + USB-C pigtail", 1, "buy"),
    ("Power", "Inline fuse holder + 10 A fuses, WAGO 221 connectors, XT60 pairs", 1, "buy"),
    ("Power", "Emergency stop 22 mm panel-mount, latching, NC contact block", 1, "buy"),
    ("Power", "Toggle switch 12 mm panel-mount, 12 V DC >= 15 A", 1, "buy"),
    ("Chassis", f"Bottom plate {L}x{W}x{BASE_T} (bottom_plate.dxf)", 1, "laser-cut 6 mm plywood"),
    ("Chassis", f"Top deck {L}x{W}x{DECK_T} (top_deck.dxf)", 1, "laser-cut 5 mm acrylic"),
    ("Chassis", f"M6 threaded rod, cut to {ROD_Z1 - ROD_Z0:.0f} mm", len(ROD_PTS), "cut from 1 m rod"),
    ("Chassis", "M6 nut + M6 washer", 4 * len(ROD_PTS), "buy"),
    ("Chassis", "M8 nut + M8 washer (casters)", 4, "buy"),
    ("Hardware", "M4x12 screws + nylock nuts (motor brackets), M4/M5x16 (load cells), M3/M2.5 screws + "
                 "standoffs (Pi, camera), VHB tape, velcro strap, zip ties", 1, "buy"),
    ("Hardware", "Pusher: M4x20 + nut + washer x 10 (rod holders, blade, drive bracket, bearing block), M4x25 + nut "
                 "+ washer x 1 (drive bracket, under the motor), M3x16 x 4 (T8 nut), M3x8 socket-head x 4 (motor), "
                 "M3x8 x 4 (rod pinch screws), M2x10 x 4 (limit switches)", 1, "buy"),
]


def main():
    out = Path(__file__).parent / "output"
    for sub in ("parts", "dxf", "views"):
        (out / sub).mkdir(parents=True, exist_ok=True)

    assy = build_assembly()
    step_path = out / "robot_assembly.step"
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
                     ("cargo_plate", Z_CARGO + CARGO_T / 2)):
        cq.exporters.export(fabricated[name].translate((0, 0, -zc)).section(), str(out / "dxf" / f"{name}.dxf"))
    print("wrote DXFs")

    comp = assy.toCompound()
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
