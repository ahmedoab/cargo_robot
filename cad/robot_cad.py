"""
Parametric CAD model - Phase II cargo robot (AGV), DESIGN v4: 3D-printed body, ESP32 only.

Run:   python robot_cad.py          (needs: pip install cadquery)
Writes
  ../print/<MATERIAL>/*.stl     every printed part, already turned to its print orientation (flat side down)
  output/robot_assembly.step    full coloured assembly -> SolidWorks / Fusion 360 / Onshape / FreeCAD / AutoCAD
  output/parts_step/*.step      every printed part as STEP, in its place in the robot (for editing)
  output/print_list.csv         printed parts: file, material, quantity, size, plastic used
  output/bom.csv                bought parts
  output/views/*.svg            line drawings

What changed from v3:
  - no Raspberry Pi, no lidar: one ESP32 does line following (8-channel IR array), obstacle detection
    (3 ultrasonic sensors), the load cell (speed by weight) and the pusher. A laptop does the AI on the conveyor.
  - the body is printed: a tub made of 4 tiles bolted together, so every part fits a 220 x 220 mm printer
  - the pusher now pushes SIDEWAYS (to the robot's right, -Y), so the robot stops beside the conveyor
    instead of reversing into it: a NEMA17 stepper moves the blade with a GT2 belt
  - one 5 kg bar load cell carries the whole cargo bed (a single-point platform scale)
  - ball casters front and rear

Axes: X = forward, Y = left, Z = up. Origin = floor under the middle of the drive axle. Units: mm.
Bought parts are simplified envelopes with the right outer sizes (good for layout and clearance checks).
"""
import csv
import math
from pathlib import Path

import cadquery as cq

# ============================== PARAMETERS ==============================
PRINTER = (210, 210, 240)        # largest part allowed (a 220 x 220 x 250 printer, with a margin)

# ---- body: a tub, split into 4 tiles at X = 0 and Y = 0 ----
L, W = 400, 300                  # outside of the tub (X, Y)
WALL, FLOOR = 3, 4               # wall and floor thickness
CORNER_R = 12                    # rounded vertical corners
Z_F0 = 22                        # underside of the floor = ground clearance
Z_F1 = Z_F0 + FLOOR              # top of the floor
Z_RIM = 195                      # top of the walls
LIP = 6                          # stiffening lip round the inside of the rim
FLANGE = (5, 12)                 # seam flanges: thickness, depth (where two tiles meet)

# ---- drive: 2 x JGB37-520 gear motors, 100 mm wheels, in the middle ----
WHEEL_D, WHEEL_W = 100, 30
AXLE_Z = WHEEL_D / 2
MOTOR_UP = 7                     # JGB37 gearbox: the output shaft is 7 mm off the gearbox centre; turned so the motor sits higher
MOTOR_Z = AXLE_Z + MOTOR_UP      # motor centre line
FACE_Y = W / 2 - WALL            # gearbox face bolts to the inside of the motor plate
WHEEL_Y = W / 2 + 3 + WHEEL_W / 2   # 3 mm between the motor plate and the wheel -> track 336 mm
WIN_X, WIN_Z, WIN_PEAK = 40, 100, 140   # window in the side wall that the motor plate fills (half-width, top, gable peak)

# ---- casters: ball casters front and rear, in printed pods ----
CASTER_X = 150
CASTER_H = 30                   # caster height, floor to the top of its flange (1-inch metal ball caster)
CASTER_FLANGE = 44               # the pod takes flanges up to this square size
CASTER_HOLES = 31                # caster mounting holes: this far apart (slots fit 26-36 mm)

# ---- load cell + cargo bed ----
CELL = (80, 12.7, 12.7)          # 5 kg bar load cell
CELL_ZONE = (5, 23)              # screw holes lie this far from each end (fits the common cells)
CELL_Y = 25                      # the cell runs along X at Y = 25 (keeps it off the bed's seam)
BED = (-140, 125, -140, 140)     # cargo bed x0, x1, y0, y1
BED_TOP = 212                    # top of the cargo bed above the floor (the conveyor belt goes 4 mm below)
BED_SKIN, BED_RIB = 3, 9         # bed: top skin and ribs underneath
BRIDGE_X, BRIDGE_W, BRIDGE_H = 26, 30, 20   # beam across the tub that carries the cell's fixed end
SPACER_T = 5                     # spacers above and below the cell
RAIL_IN = (-117.5, 102.5)        # package guide rails on the bed: their inner faces (X)
RAIL_T, RAIL_H = 10, 20

# ---- pusher: blade pushes the package off the right side (-Y) onto the conveyor ----
ROD_X = (142, 188)               # two 8 mm rods along Y in the front strip
ROD_Z = 232
ROD_LEN = W                      # 300 mm rods, ends flush with the sides
HOLDER_T = 10                    # rod holders at each end: Y from +-140 to +-150
CARR_LEN = 54                    # carriage length along Y (two LM8UU per rod)
BLADE_AHEAD = 6                  # the blade's pushing face is this far beyond the carriage's -Y end
PUSH_HOME, PUSH_END = 65, -145   # blade's pushing face: parked / fully out (5 mm inside the robot's side)
BLADE_T, BLADE_H = 5, 120
BLADE_X = (-112, 97)             # blade ends (it runs between the guide rails)
BLADE_GAP = 5                    # blade's lower edge above the bed
BELT_X, BELT_Z = 165, 210        # GT2 belt loop: centre line and height
PULLEY_Y, IDLER_Y = 125.5, -129  # motor pulley (+Y) and idler (-Y): the belt clamp must stay between them
CLAMP_AT = (20.5, 44.5)          # belt clamp, measured from the carriage's -Y end
NEMA = (42.3, 40)                # NEMA17: square size, body length

# ---- sensors, controls ----
US_Y = (-95, 0, 95)              # 3 ultrasonic sensors in the front wall (right, centre, left)
US_Z = 82
TOF_X, TOF_Z = -60, 175          # side-facing distance sensor (gap to the conveyor), right wall
LINE_X = 191                     # 8-channel line sensor array under the front
ESTOP_POS = (-170, 95)
SWITCH_POS = (-170, 45)
BATTERY = (50, 155, 35)          # 3S 5000 mAh LiPo, lying across the robot (X, Y, Z)
BATT_X = -86                     # battery centre X; it slides out of the left side, behind the wheel

# ---- derived ----
BED_X = (BED[0] + BED[1]) / 2    # the bed's seam (along Y)
Z_BED0 = BED_TOP - BED_SKIN - BED_RIB            # underside of the bed's ribs = 200
CELL_FIX_X = BRIDGE_X                           # middle of the cell's fixed-end screw zone
CELL_X0 = CELL_FIX_X + (CELL_ZONE[0] + CELL_ZONE[1]) / 2 - CELL[0]   # rear end of the cell
CELL_LOAD_X = CELL_X0 + (CELL_ZONE[0] + CELL_ZONE[1]) / 2            # middle of the loaded-end screw zone
Z_BRIDGE = 170
Z_CELL = Z_BRIDGE + SPACER_T                     # underside of the cell
Z_HUB = Z_CELL + CELL[2] + SPACER_T              # underside of the bed's hub (the cell's loaded end, via a spacer)
Z_BLADE = BED_TOP + BLADE_GAP
CARR_Z = (ROD_Z - 12, 250)                       # carriage bottom / top (level with the arm's top)
ARM_Z = (236, 250)                               # arm from the carriage to the blade (above the front guide rail)
CARR_Y = (PUSH_END + BLADE_AHEAD, PUSH_HOME + BLADE_AHEAD)   # carriage's -Y end: fully out / parked

# ============================== COLOURS ==============================
PLA_C = (0.20, 0.55, 0.95, 1)
PETG_C = (1.0, 0.55, 0.1, 1)
ABS_C = (0.85, 0.15, 0.15, 1)
TPU_C = (0.15, 0.15, 0.15, 1)
STEEL = (0.62, 0.63, 0.66, 1)
DARK = (0.30, 0.30, 0.33, 1)
BRASS = (0.80, 0.65, 0.30, 1)
RUBBER = (0.08, 0.08, 0.08, 1)
PCB = (0.10, 0.45, 0.20, 1)
PCB_BLUE = (0.10, 0.25, 0.60, 1)
PCB_RED = (0.70, 0.10, 0.10, 1)
BLACK = (0.15, 0.15, 0.15, 1)
RED = (0.85, 0.10, 0.10, 1)
YELLOW = (0.95, 0.80, 0.10, 1)
GREEN = (0.10, 0.75, 0.25, 1)
BATT = (0.20, 0.30, 0.80, 1)
MOTOR = (0.35, 0.35, 0.38, 1)
ALU = (0.80, 0.82, 0.86, 1)
MATERIAL_COLOUR = {"PLA": PLA_C, "PETG": PETG_C, "ABS": ABS_C, "TPU": TPU_C}


# ============================== HELPERS ==============================
def box2(x0, x1, y0, y1, z0, z1):
    """Box given by two opposite corners."""
    (x0, x1), (y0, y1), (z0, z1) = sorted((x0, x1)), sorted((y0, y1)), sorted((z0, z1))
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def rbox(x0, x1, y0, y1, z0, z1, r):
    """Box with rounded vertical edges."""
    return box2(x0, x1, y0, y1, z0, z1).edges("|Z").fillet(r)


def cyl_z(d, z0, z1, x=0, y=0):
    return cq.Workplane("XY").circle(d / 2).extrude(z1 - z0).translate((x, y, z0))


def cyl_y(d, y0, y1, x=0, z=0):
    """Cylinder along +Y from y0 to y1 (y1 > y0)."""
    return cq.Workplane("XZ").circle(d / 2).extrude(-(y1 - y0)).translate((x, y0, z))


def cyl_x(d, x0, x1, y=0, z=0):
    """Cylinder along +X from x0 to x1 (x1 > x0)."""
    return cq.Workplane("YZ").circle(d / 2).extrude(x1 - x0).translate((x0, y, z))


def holes_z(pts, d, z0, z1):
    """Vertical holes of diameter d at XY points, from z0 to z1."""
    return cq.Workplane("XY").pushPoints(pts).circle(d / 2).extrude(z1 - z0).translate((0, 0, z0))


def union_all(shapes):
    out = shapes[0]
    for s in shapes[1:]:
        out = out.union(s)
    return out


def hex_prism_z(af, z0, z1, x, y):
    """Hexagon (nut trap) with the given width across flats."""
    return cq.Workplane("XY").polygon(6, af / math.cos(math.pi / 6)).extrude(z1 - z0).translate((x, y, z0))


M3, M4, M5 = 3.4, 4.5, 5.5       # clearance holes
M3_TAP = 2.8                     # for M3 self-tapping screws straight into the plastic

# ============================== REGISTRY ==============================
groups = {}          # group name -> list of (name, workplane, colour)
printed = {}         # file name -> dict(shape, material, qty, orient, note)


def add(group, name, wp, color):
    groups.setdefault(group, []).append((name, wp, color))


def add_print(group, name, wp, material, qty=1, orient=(), note="", file=None, show=True):
    """A printed part. orient = rotations [(axis, degrees), ...] that lay it flat for printing.
    file: STL name if several placed parts share one file (only the first one is exported)."""
    if show:
        add(group, name, wp, MATERIAL_COLOUR[material])
    key = file or name
    if key not in printed:
        printed[key] = dict(shape=wp, material=material, qty=qty, orient=orient, note=note)


# ============================== BODY: THE TUB (4 printed tiles) ==============================
XI, YI = L / 2 - WALL, W / 2 - WALL          # inside faces of the walls
POD = CASTER_FLANGE + 6                      # caster pod: the square that passes through the floor

# bolt positions shared by the tub and the parts fixed to it
MOTOR_EAR_XZ = [(x, z) for x in (-49, 49) for z in (45, 85)]                     # through the side walls (M3)
MOTOR_FOOT_XY = [(x, s * y) for x in (-38, 38) for y in (102, 138) for s in (1, -1)]   # through the floor (M4)
POD_XY = [(cx + dx, dy) for cx in (CASTER_X, -CASTER_X) for dx in (-26, 26) for dy in (-26, 26)]
BRIDGE_XZ = [(BRIDGE_X - BRIDGE_W / 2 + 6, Z_BRIDGE - 10), (BRIDGE_X + BRIDGE_W / 2 + 4, Z_BRIDGE - 10)]                          # through the side walls (M4)
HOLDER_XZ = [(134, 178), (184, 178)]                                            # through the side walls (M4)
HOLDER_FRONT_YZ = [(125, 178), (-125, 178)]                                     # through the front wall (M4)
BATT_TRAY_XY = [(BATT_X + dx, s * 91) for dx in (-18, 18) for s in (1, -1)]
BATT_DOOR_XZ = [(BATT_X + dx, Z_F1 + 51) for dx in (-20, 20)]
LINE_XY = [(LINE_X, -55), (LINE_X, 55)]
BUMPER_YZ = [(y, 44) for y in (-80, -30, 30, 80)]
REAR_PANEL_XY = [(-L / 2 + WALL + 3, s * y) for y in (40, 110) for s in (1, -1)] + \
                [(x, s * (W / 2 - WALL - 3)) for x in (-165, -185) for s in (1, -1)]
FRONT_COVER_XY = [(L / 2 - WALL - 3, y) for y in (-70, -25, 25, 70)]


def gable_window(y0, y1):
    """The opening in a side wall that the motor plate fills: straight sides and a 45-degree roof,
    so the tile prints without support."""
    pts = [(-WIN_X, Z_F1), (WIN_X, Z_F1), (WIN_X, WIN_Z), (0, WIN_PEAK), (-WIN_X, WIN_Z)]
    return cq.Workplane("XZ").polyline(pts).close().extrude(-(y1 - y0)).translate((0, y0, 0))


def ring(x0, x1, y0, y1, z0, z1, r, w):
    """Rounded-rectangle ring, w wide, just inside the given outline."""
    return rbox(x0, x1, y0, y1, z0, z1, r).cut(rbox(x0 + w, x1 - w, y0 + w, y1 - w, z0 - 1, z1 + 1, max(r - w, 0.5)))


def tub_holes():
    """Every opening and bolt hole in the tub, as one cutting tool."""
    h = []
    for s in (1, -1):                                                  # motor windows, both sides
        h.append(gable_window(*sorted((s * (W / 2 + 1), s * (YI - 1)))))
    for x in (CASTER_X, -CASTER_X):                                    # caster pods go through the floor
        h.append(box2(x - POD / 2, x + POD / 2, -POD / 2, POD / 2, Z_F0 - 1, Z_F1 + 1))
    for y in US_Y:                                                     # ultrasonic sensors, front wall
        for dy in (-13, 13):
            h.append(cyl_x(16.5, XI - 1, L / 2 + 1, y + dy, US_Z))
        for dy in (-27, 27):
            h.append(cyl_x(M3, XI - 1, L / 2 + 1, y + dy, US_Z))
    h.append(box2(TOF_X - 7, TOF_X + 7, -W / 2 - 1, -YI + 1, TOF_Z - 5, TOF_Z + 5))        # distance sensor window
    for dx in (-16, 16):
        h.append(cyl_y(M3, -W / 2 - 1, -YI + 1, TOF_X + dx, TOF_Z))
    h.append(box2(BATT_X - 27, BATT_X + 27, YI - 1, W / 2 + 1, Z_F1 + 1, Z_F1 + 46))       # battery hatch, left wall
    for x, z in BATT_DOOR_XZ:
        h.append(cyl_y(M3, YI - 1, W / 2 + 1, x, z))
    h.append(box2(-L / 2 - 1, -XI + 1, 50, 70, 33, 47))                                     # ESP32 USB port, rear wall
    h.append(cyl_z(10, Z_F0 - 1, Z_F1 + 1, 170, 45))                                       # line sensor cable
    for x in range(50, 111, 12):                                                           # vents over the motor driver
        h.append(box2(x - 2, x + 2, -W / 2 - 1, -YI + 1, 45, 90))
    for x, z in MOTOR_EAR_XZ:
        h.append(cyl_y(M3, -W / 2 - 1, W / 2 + 1, x, z))
    for x, z in BRIDGE_XZ + HOLDER_XZ:
        h.append(cyl_y(M4, -W / 2 - 1, W / 2 + 1, x, z))
    for y, z in HOLDER_FRONT_YZ:
        h.append(cyl_x(M4, XI - 1, L / 2 + 1, y, z))
    for y, z in BUMPER_YZ:
        h.append(cyl_x(M3, XI - 1, L / 2 + 1, y, z))
    h.append(holes_z(MOTOR_FOOT_XY + POD_XY + BATT_TRAY_XY, M4, Z_F0 - 1, Z_F1 + 1))
    h.append(holes_z(LINE_XY, M3, Z_F0 - 1, Z_F1 + 1))
    h.append(holes_z(REAR_PANEL_XY + FRONT_COVER_XY, M3_TAP, Z_RIM - 6, Z_RIM + 1))     # screws into the lip
    grid = [(x, y) for x in list(range(-185, -123, 10)) + list(range(55, 111, 10)) for y in range(-135, 136, 10)
            if abs(y) > 8 and not (abs(abs(x) - CASTER_X) < 36 and abs(y) < 36) and not (x < 50 and abs(y) > 90 and x > 0)]
    h.append(holes_z(grid, M3_TAP, Z_F0 - 1, Z_F1 + 1))                                    # electronics: 10 mm grid
    return union_all(h)


def tub():
    """The whole tub in one piece (it is split into 4 tiles below)."""
    t = rbox(-L / 2, L / 2, -W / 2, W / 2, Z_F0, Z_RIM, CORNER_R).cut(
        rbox(-XI, XI, -YI, YI, Z_F1, Z_RIM + 1, CORNER_R - WALL))
    for k in (2, 4, LIP):                       # lip round the inside of the rim, stepped at 45 degrees
        t = t.union(ring(-XI, XI, -YI, YI, Z_RIM - LIP - 2 + k, Z_RIM, CORNER_R - WALL, k))
    for y0, y1 in ((88, YI), (-YI, -99.5)):     # no lip at the front corners: the pusher's rod holders sit there
        t = t.cut(box2(126, XI, y0, y1, Z_RIM - LIP - 1, Z_RIM + 1))
    return t.cut(tub_holes())


def seam_flanges(sx, sy):
    """Flanges along the two inner edges of the tile in quadrant (sx, sy); bolts join them to the next tile."""
    f, d = FLANGE
    s = lambda a, b: sorted((a, b))
    fl = union_all([
        box2(*s(0, sx * f), *s(0, sy * 70), Z_F1, Z_F1 + d),                          # floor, X = 0 seam
        box2(*s(0, sx * (114 if sx > 0 else 30)), *s(0, sy * f), Z_F1, Z_F1 + d),     # floor, Y = 0 seam
        box2(*s(sx * (XI - d), sx * XI), *s(0, sy * f), Z_F1, Z_RIM),                 # front / rear wall
        box2(*s(0, sx * f), *s(sy * (YI - d), sy * YI), WIN_PEAK + 2, Z_RIM),         # side wall, above the window
    ])
    if sx > 0:                                  # leave room for the middle ultrasonic sensor
        fl = fl.cut(box2(XI - d - 1, XI, -f - 1, f + 1, US_Z - 24, US_Z + 24))
    return fl


def seam_bolts():
    h = [cyl_x(M4, -8, 8, y, Z_F1 + 6) for y in (-50, -20, 20, 50)]                  # floor, X = 0 seam
    h += [cyl_y(M4, -8, 8, x, Z_F1 + 6) for x in (-18, 25, 60, 95)]                  # floor, Y = 0 seam
    h += [cyl_y(M4, -8, 8, XI - 6, z) for z in (45, 140, 175)]                       # front wall
    h += [cyl_y(M4, -8, 8, -XI + 6, z) for z in (45, 110, 170)]                      # rear wall
    h += [cyl_x(M4, -8, 8, sy * (YI - 6), z) for sy in (1, -1) for z in (158, 178)]   # side walls
    return union_all(h)


TUB = tub()
SEAM_BOLTS = seam_bolts()
TILE_NAME = {(1, 1): "front_left", (1, -1): "front_right", (-1, 1): "rear_left", (-1, -1): "rear_right"}


def tub_tile(sx, sy):
    quadrant = box2(min(0, sx * 215), max(0, sx * 215), min(0, sy * 165), max(0, sy * 165), Z_F0 - 5, Z_RIM + 5)
    return TUB.intersect(quadrant).union(seam_flanges(sx, sy)).cut(SEAM_BOLTS)


for (sx, sy), tag in TILE_NAME.items():
    add_print("body", f"tub_{tag}", tub_tile(sx, sy), "PETG",
              note="print floor down. 4 walls, 25 % gyroid infill")


# ============================== DRIVE: motor mounts, gear motors, wheels ==============================
# Left-hand parts are modelled; the right-hand ones are their mirror images.
GEARBOX_D, MOTOR_D = 37, 35
CLAMP_Y = (123, 141)             # the clamp holds the gearbox over this length
CLAMP_X = 31                     # half-width of the clamp


def motor_mount():
    """ABS. Fills the window in the side wall (its 3 mm face plate takes the place of the wall, so the short
    motor shaft still reaches into the wheel hub), clamps the gearbox and bolts to both tiles: to the floor
    through the foot and to the walls through the ears. It does not rely on the gearbox's screw pattern."""
    c = 0.3                                                                   # fit in the window
    pts = [(-WIN_X + c, Z_F1), (WIN_X - c, Z_F1), (WIN_X - c, WIN_Z - c), (0, WIN_PEAK - 2 * c),
           (-WIN_X + c, WIN_Z - c)]
    face = cq.Workplane("XZ").polyline(pts).close().extrude(-WALL).translate((0, FACE_Y, 0))
    back = box2(-58, 58, YI - 6, YI, Z_F1, 95)                                 # behind the walls (the ears)
    foot = box2(-45, 45, 95, YI, Z_F1, Z_F1 + 4)
    cradle = (box2(-CLAMP_X, CLAMP_X, CLAMP_Y[0], CLAMP_Y[1], Z_F1, MOTOR_Z)
              .union(box2(-22, 22, 85, CLAMP_Y[0], Z_F1, MOTOR_Z - 8)))         # under the motor's rear end
    gussets = union_all([box2(x - 2.5, x + 2.5, 100, YI - 6, Z_F1, Z_F1 + 30) for x in (-42, 42)])
    m = union_all([face, back, foot, cradle, gussets])
    m = m.cut(cyl_y(GEARBOX_D + 0.6, FACE_Y - 28, YI + 0.01, 0, MOTOR_Z))      # gearbox (its face rests on the plate)
    m = m.cut(cyl_y(MOTOR_D + 0.8, 80, CLAMP_Y[0], 0, MOTOR_Z))                # motor body
    m = m.cut(cyl_y(14, FACE_Y - 1, W / 2 + 1, 0, AXLE_Z))                     # shaft boss
    for x in (-26, 26):                                                        # clamp bolts (M4) + nut traps
        m = m.cut(cyl_z(M4, Z_F1 + 6, MOTOR_Z + 1, x, (CLAMP_Y[0] + CLAMP_Y[1]) / 2))
        m = m.cut(box2(x - 4.2, x + 4.2, CLAMP_Y[0] - 1, (CLAMP_Y[0] + CLAMP_Y[1]) / 2 + 4, Z_F1 + 8, Z_F1 + 11.5))
    m = m.cut(box2(-25, 25, 92, 96, Z_F1 + 4, MOTOR_Z - 6))                    # zip-tie slot round the motor
    m = m.cut(holes_z([(x, y) for x, y in MOTOR_FOOT_XY if y > 0], M4, Z_F1 - 1, Z_F1 + 5))
    for x, z in MOTOR_EAR_XZ:
        m = m.cut(cyl_y(M3, YI - 7, YI + 1, x, z))
    return m


def motor_clamp_cap():
    """ABS. Top half of the gearbox clamp."""
    yc = (CLAMP_Y[0] + CLAMP_Y[1]) / 2
    cap = box2(-CLAMP_X, CLAMP_X, CLAMP_Y[0], CLAMP_Y[1], MOTOR_Z, MOTOR_Z + GEARBOX_D / 2 + 5)
    cap = cap.edges("|Y and >Z").fillet(8)
    cap = cap.cut(cyl_y(GEARBOX_D + 0.6, CLAMP_Y[0] - 1, CLAMP_Y[1] + 1, 0, MOTOR_Z))
    return cap.cut(holes_z([(-26, yc), (26, yc)], M4, MOTOR_Z - 1, MOTOR_Z + 30))


def gear_motor():
    """JGB37-520 envelope: gearbox, motor, encoder, the eccentric output shaft and its boss."""
    return (cyl_y(GEARBOX_D, FACE_Y - 26, FACE_Y, 0, MOTOR_Z)
            .union(cyl_y(MOTOR_D, FACE_Y - 58, FACE_Y - 26, 0, MOTOR_Z))
            .union(cyl_y(33, FACE_Y - 70, FACE_Y - 58, 0, MOTOR_Z))
            .union(cyl_y(12, FACE_Y, FACE_Y + 4, 0, AXLE_Z))
            .union(cyl_y(6, FACE_Y + 4, WHEEL_Y, 0, AXLE_Z)))


def wheel():
    y0, y1 = WHEEL_Y - WHEEL_W / 2, WHEEL_Y + WHEEL_W / 2
    rim_d = WHEEL_D - 24
    tire = cyl_y(WHEEL_D, y0, y1, 0, AXLE_Z).cut(cyl_y(rim_d, y0 - 1, y1 + 1, 0, AXLE_Z))
    rim = cyl_y(rim_d, y0 + 3, y1 - 3, 0, AXLE_Z).cut(cyl_y(6, y0, y1, 0, AXLE_Z))
    for i in range(6):
        a = math.radians(60 * i)
        rim = rim.cut(cyl_y(14, y0, y1, 22 * math.cos(a), AXLE_Z + 22 * math.sin(a)))
    return tire, rim


MOUNT, CAP = motor_mount(), motor_clamp_cap()
for side, s in (("left", 1), ("right", -1)):
    mirror = (lambda w: w) if s > 0 else (lambda w: w.mirror("XZ"))
    add_print("drive", f"motor_mount_{side}", mirror(MOUNT), "ABS", orient=[("x", -90 * s)],
              note="print with the face plate down. ABS: the motor gets warm. 5 walls, 40 % infill")
    add_print("drive", f"motor_clamp_cap_{side}", mirror(CAP), "ABS", qty=2, file="motor_clamp_cap",
              orient=[("x", 90)], note="print on its end")
    add("drive", f"gear_motor_{side}", mirror(gear_motor()), MOTOR)
    tire, rim = wheel()
    add("drive", f"tire_{side}", mirror(tire), RUBBER)
    add("drive", f"rim_{side}", mirror(rim), BLACK)


# ============================== CASTERS: ball casters in printed pods ==============================
def caster_pod(x):
    """PETG. Pushed up through the square hole in the floor from below, its flange under the floor. It sits on
    the Y = 0 seam, so it also joins the two tiles there. The caster bolts up under its roof, with shims between
    them: nominal is one 2 mm shim, so the caster can be set 2 mm higher or lower."""
    roof = CASTER_H + 2
    p = (box2(x - POD / 2 + 0.2, x + POD / 2 - 0.2, -POD / 2 + 0.2, POD / 2 - 0.2, Z_F0, roof + 4)
         .union(box2(x - 32, x + 32, -32, 32, Z_F0 - 4, Z_F0)))                 # flange under the floor
    p = p.cut(box2(x - CASTER_FLANGE / 2, x + CASTER_FLANGE / 2, -CASTER_FLANGE / 2, CASTER_FLANGE / 2,
                   Z_F0 - 5, roof))                                             # pocket for the caster
    for sx in (-1, 1):                                                          # caster holes: slots along X
        p = p.cut(cq.Workplane("XY").slot2D(10 + M4, M4).extrude(10).translate((x + sx * 15.5, 0, roof - 1)))
    return p.cut(holes_z([(px, py) for px, py in POD_XY if abs(px - x) < 40], M4, Z_F0 - 5, Z_F0 + 1))


def caster_shim(t):
    s = box2(-CASTER_FLANGE / 2 + 0.5, CASTER_FLANGE / 2 - 0.5, -CASTER_FLANGE / 2 + 0.5, CASTER_FLANGE / 2 - 0.5, 0, t)
    for sx in (-1, 1):
        s = s.cut(cq.Workplane("XY").slot2D(10 + M4, M4).extrude(t + 2).translate((sx * 15.5, 0, -1)))
    return s


def ball_caster(x):
    """1-inch metal ball caster: ball, housing, flange (its top at CASTER_H)."""
    return (cyl_z(25.4, 0, 25.4, x, 0).union(cyl_z(34, 9, CASTER_H - 3, x, 0))
            .union(box2(x - 20, x + 20, -20, 20, CASTER_H - 3, CASTER_H)))


for tag, x in (("front", CASTER_X), ("rear", -CASTER_X)):
    add_print("drive", f"caster_pod_{tag}", caster_pod(x), "PETG", qty=2, file="caster_pod",
              note="print flange down")
    add("drive", f"ball_caster_{tag}", ball_caster(x), STEEL)
    add("drive", f"caster_shim_{tag}", caster_shim(2).translate((x, 0, CASTER_H)), PETG_C)
printed["caster_shim_1mm"] = dict(shape=caster_shim(1), material="PETG", qty=4, orient=[],
                                 note="under the caster, to level the robot")
printed["caster_shim_2mm"] = dict(shape=caster_shim(2), material="PETG", qty=4, orient=[], note="as above")


# ============================== PAYLOAD: load cell, cargo bed, guide rails ==============================
# The cargo bed rests on ONE bar load cell (a single-point platform scale). The cell's fixed end is bolted on
# a bridge across the tub; its loaded end carries the bed. Nothing else touches the bed, so the cell weighs
# the package, and the pusher blade passes 5 mm above the bed.
ZONE = CELL_ZONE[1] - CELL_ZONE[0]
BX0, BX1 = BRIDGE_X - BRIDGE_W / 2, BRIDGE_X + BRIDGE_W / 2
STOP_Y = 100                     # overload stops under the bed, at Y = +-100
RAIL_X = [(RAIL_IN[0] - RAIL_T, RAIL_IN[0]), (RAIL_IN[1], RAIL_IN[1] + RAIL_T)]   # rear and front guide rails
RAIL_HOLE_Y = (25, 115)          # screws from under the bed into each half-rail


def slot_x(cx, cy, z0, z1, length, width):
    """Round-ended slot along X: 'length' = distance between the end-circle centres."""
    return cq.Workplane("XY").slot2D(length + width, width).extrude(z1 - z0).translate((cx, cy, z0))


def bridge_half():
    """PETG, left half (+Y). A U-beam across the tub, open underneath, bolted to the side wall and to the
    other half. It carries the load cell's fixed end and two overload stops."""
    z0, z1 = Z_BRIDGE - BRIDGE_H, Z_BRIDGE
    b = union_all([
        box2(BX0, BX1, 0, YI - 6, z1 - 5, z1),                                   # top
        box2(BX0, BX0 + 4, 0, YI - 6, z0, z1), box2(BX1 - 4, BX1, 0, YI - 6, z0, z1),   # two webs
        box2(BX0, BX1 + 10, YI - 6, YI, z0, z1 + 2),                             # end plate, against the wall
        box2(BX0, BX1, 0, 6, z0, z1),                                            # centre flange
        box2(BRIDGE_X - 5, BRIDGE_X + 5, STOP_Y - 5, STOP_Y + 5, z1, Z_BED0 - 1.5),   # overload stop
    ])
    b = b.cut(slot_x(CELL_FIX_X, CELL_Y, z1 - 6, z1 + 1, ZONE, 5.5))           # cell's fixed-end screws
    for x, z in BRIDGE_XZ:
        b = b.cut(cyl_y(M4, YI - 7, YI + 1, x, z))
    for x in (BX0 + 9, BX1 - 9):
        b = b.cut(cyl_y(M4, -1, 7, x, Z_BRIDGE - 10))
    return b


def cell_spacer(x):
    """PETG, 100 % infill. Lifts the cell off the bridge (fixed end) and the bed off the cell (loaded end)."""
    return box2(x - 14, x + 14, CELL_Y - CELL[1] / 2, CELL_Y + CELL[1] / 2, 0, SPACER_T).cut(
        slot_x(x, CELL_Y, -1, SPACER_T + 1, ZONE, 5.5))


def bed_tile(sx, sy):
    """PETG. One quarter of the cargo bed: a 3 mm skin with ribs underneath, bolted to its neighbours along
    the seams. Print it upside down, so the face the package slides on is the smooth face from the print bed."""
    xm = BED_X
    x0, x1 = (xm, BED[1]) if sx > 0 else (BED[0], xm)
    y0, y1 = (0, BED[3]) if sy > 0 else (BED[2], 0)
    zs = BED_TOP - BED_SKIN
    t = box2(x0, x1, y0, y1, zs, BED_TOP)
    ribs = [box2(x0, x1, y0, y0 + 3, Z_BED0, zs), box2(x0, x1, y1 - 3, y1, Z_BED0, zs),
            box2(x0, x0 + 3, y0, y1, Z_BED0, zs), box2(x1 - 3, x1, y0, y1, Z_BED0, zs)]
    ribs += [box2(x0, x1, y - 1.5, y + 1.5, Z_BED0, zs) for y in (35, 70, 105, -35, -70, -105) if y0 < y < y1]
    ribs += [box2(x - 1.5, x + 1.5, y0, y1, Z_BED0, zs) for x in (-95, -50, 30, 75) if x0 < x < x1]
    seam_x = x0 if sx > 0 else x1 - 4                                          # thicker ribs on the seams
    seam_y = y0 if sy > 0 else y1 - 4
    ribs += [box2(seam_x, seam_x + 4, y0, y1, Z_BED0, zs), box2(x0, x1, seam_y, seam_y + 4, Z_BED0, zs)]
    if (sx, sy) == (-1, 1):                                                    # hub on the cell's loaded end
        ribs.append(box2(CELL_LOAD_X - 15, CELL_LOAD_X + 15, CELL_Y - 12, CELL_Y + 12, Z_HUB, zs))
    if sx > 0:                                                                 # pads over the overload stops
        ribs.append(box2(BRIDGE_X - 7, BRIDGE_X + 7, sy * STOP_Y - 7, sy * STOP_Y + 7, Z_BED0, zs))
    t = t.union(union_all(ribs))
    t = t.edges(">Z").chamfer(0.8)
    if sy < 0:                                                                 # the package slides off this edge
        t = t.cut(box2(x0 - 1, x1 + 1, y0 - 1, y0 + 2, BED_TOP - 2, BED_TOP + 1)).union(
            box2(x0, x1, y0, y0 + 2, BED_TOP - 2, BED_TOP).edges("<Y and >Z").fillet(1.9))
    if (sx, sy) == (-1, 1):                                                    # cell screws, heads sunk in the hub
        t = t.cut(slot_x(CELL_LOAD_X, CELL_Y, Z_HUB - 1, BED_TOP + 1, ZONE, 4.5))
        t = t.cut(slot_x(CELL_LOAD_X, CELL_Y, BED_TOP - 4.5, BED_TOP + 1, ZONE, 8.5))
    for (rx0, rx1) in RAIL_X:                                                  # guide-rail screws
        if x0 < rx0 < x1:
            pts = [((rx0 + rx1) / 2, sy * y) for y in RAIL_HOLE_Y]
            t = t.cut(holes_z(pts, M3, Z_BED0 - 1, BED_TOP + 1))
    for x in ((-110, -60, -25) if sx < 0 else (15, 55, 100)):                  # seam bolts across Y = 0 (M3)
        t = t.cut(cyl_y(M3, -10, 10, x, Z_BED0 + 4.5))
    for y in (30, 90, -30, -90):                                              # seam bolts across the X seam
        t = t.cut(cyl_x(M3, xm - 10, xm + 10, y, Z_BED0 + 4.5))
    return t


def guide_rail():
    """PETG. Half of a package guide rail (each side of the bed has 2): it keeps the package square while the
    blade pushes it. Screwed on from under the bed."""
    r = box2(0, RAIL_T, 0, BED[3], 0, RAIL_H).edges("|Y and >Z").fillet(3)
    return r.cut(holes_z([(RAIL_T / 2, y) for y in RAIL_HOLE_Y], M3_TAP, -1, 12))


BRIDGE = bridge_half()
add_print("payload", "bridge_left", BRIDGE, "PETG",
          note="print standing on its webs (open side down). Carries the load cell")
add_print("payload", "bridge_right", BRIDGE.mirror("XZ"), "PETG",
          note="print standing on its webs. The mirror image of bridge_left")
add("payload", "load_cell_5kg", box2(CELL_X0, CELL_X0 + CELL[0], CELL_Y - CELL[1] / 2, CELL_Y + CELL[1] / 2,
                                     Z_CELL, Z_CELL + CELL[2]), ALU)
add_print("payload", "cell_spacer_fixed", cell_spacer(CELL_FIX_X).translate((0, 0, Z_BRIDGE)), "PETG", qty=2,
          file="load_cell_spacer", note="100 % infill")
add_print("payload", "cell_spacer_loaded", cell_spacer(CELL_LOAD_X).translate((0, 0, Z_CELL + CELL[2])), "PETG",
          file="load_cell_spacer")
for (sx, sy), tag in TILE_NAME.items():
    add_print("payload", f"bed_{tag}", bed_tile(sx, sy), "PETG", orient=[("x", 180)],
              note="print TOP FACE DOWN on a smooth plate (the package slides on that face). 3 walls, 20 % infill")
RAIL = guide_rail()
for (rx0, _), tag in zip(RAIL_X, ("rear", "front")):
    for sy in (1, -1):
        piece = RAIL if sy > 0 else RAIL.mirror("XZ")
        add_print("payload", f"guide_rail_{tag}_{'left' if sy > 0 else 'right'}", piece.translate((rx0, 0, BED_TOP)),
                  "PETG", qty=4, file="guide_rail", note="print standing on its flat bottom")


# ============================== PUSHER: blade on a belt-driven carriage ==============================
# Two 8 mm rods run across the front strip of the robot. A carriage with 4 LM8UU bearings rides on them,
# pulled by a GT2 belt. A NEMA17 stepper hangs under the left rod holder (shaft up) and the idler is on the
# right one, so nothing sticks out of the robot's 300 mm width. The blade hangs from an arm on the carriage,
# reaches back over the bed and pushes the package off the right side (-Y). Limit switches sit in pockets.
HB = (128, 200)                  # rod holders: X extent
SW_POCKET = (155, 175, 225, 235)  # limit-switch pocket in each holder: x0, x1, z0, z1
PULLEY_Z = (199, 214)            # 20T pulley: hub 199-206, belt between flanges 207-213


def switch_face(side):
    """Y of the face a limit switch is let into: the HOME switch (+Y) on a post, so the carriage parks with the
    blade at PUSH_HOME; the END switch (-Y) on the right holder's face."""
    return PULLEY_Y + 0.5 if side > 0 else -(W / 2 - HOLDER_T)


def rod_holder(motor):
    """Left-hand rod holder (+Y). motor=True: ABS, with the plate the NEMA17 hangs from (shaft up)."""
    yi, yo = W / 2 - HOLDER_T, W / 2
    h = union_all([
        box2(HB[0], XI, yi, yo, Z_RIM, CARR_Z[1] - 4),                        # block on the wall top: holds the rods
        box2(HB[0], XI, YI - 6, YI, 165, Z_RIM),                              # skirt inside the side wall
        box2(XI - 6, XI, 100, YI, 165, Z_RIM),                                # skirt inside the front wall
    ])
    if motor:
        h = h.cut(box2(141, 189, YI - 7, YI + 1, 160, Z_RIM))                 # the stepper sits against the wall here
        h = h.union(box2(140, XI - 6, 101, YI, Z_RIM - 4, Z_RIM))             # motor plate
        h = h.union(box2(155, 175, PULLEY_Y + 0.5, yi, 222, 238))             # post for the HOME switch
        h = h.cut(cyl_z(23, Z_RIM - 5, Z_RIM + 1, BELT_X, PULLEY_Y))
        h = h.cut(holes_z([(BELT_X + dx, PULLEY_Y + dy) for dx in (-15.5, 15.5) for dy in (-15.5, 15.5)],
                          M3, Z_RIM - 5, Z_RIM + 1))
    for x in ROD_X:
        h = h.cut(cyl_y(8.1, yi - 1, yo + 1, x, ROD_Z))                       # rods
        h = h.cut(cyl_z(M3_TAP, ROD_Z, CARR_Z[1], x, (yi + yo) / 2))          # M3 pinch screw on each rod
    h = h.cut(box2(-10, 10, -10, 10, 160, Z_RIM).rotate((0, 0, 0), (0, 0, 1), 45).translate((XI, YI, 0)))  # tub corner
    x0, x1, z0, z1 = SW_POCKET                                                # limit switch, flush with the face
    ys = switch_face(1 if motor else -1) * (1 if motor else -1)
    h = h.cut(box2(x0, x1, ys - 1, ys + 6.6, z0, z1))
    for dx in (-4.75, 4.75):
        h = h.cut(cyl_y(2.2, ys - 1, ys + 12, (x0 + x1) / 2 + dx, (z0 + z1) / 2))
    for x, z in HOLDER_XZ:
        h = h.cut(cyl_y(M4, YI - 7, YI + 1, x, z))
    for y, z in HOLDER_FRONT_YZ:
        if y > 0:
            h = h.cut(cyl_x(M4, XI - 7, XI + 1, y, z))
    return h


def idler_arm():
    """The right holder's arm under the belt's idler; the slot sets the belt tension."""
    a = box2(152, XI, -YI + 6, -100, Z_RIM, Z_RIM + 6)
    return a.cut(cq.Workplane("XY").slot2D(16 + M5, M5, angle=90).extrude(8).translate((BELT_X, IDLER_Y, Z_RIM - 1)))


def carriage(face):
    """PETG. Carriage + arm for the blade's pushing face at Y = face. The 4 LM8UU snap into pockets from
    below (and get a zip tie each); the belt is clamped underneath; the blade bolts to the arm's front."""
    y0 = face + BLADE_AHEAD                                                   # the carriage's -Y end
    y1 = y0 + CARR_LEN
    c = union_all([
        box2(HB[0] + 2, HB[1] - 0.5, y0, y1, CARR_Z[0], CARR_Z[1]),           # body round the rods
        box2(60, HB[0] - 2, face + BLADE_T, y1, ARM_Z[0], ARM_Z[1]),          # arm over the bed
        box2(HB[0] - 8, HB[0] + 3, y0, y1, ARM_Z[0], ARM_Z[1]),               # joins the arm to the body
        box2(152, 165.5, y0 + CLAMP_AT[0], y0 + CLAMP_AT[1], BELT_Z - 6, CARR_Z[0] + 1),   # belt clamp
    ])
    for x in ROD_X:
        c = c.cut(cyl_y(10, y0 - 1, y1 + 1, x, ROD_Z))
        for yb in (y0 + 3, y0 + 27):                                          # bearing pockets, open underneath
            c = c.cut(cyl_y(15.2, yb, yb + 24, x, ROD_Z))
            c = c.cut(box2(x - 5.4, x + 5.4, yb, yb + 24, CARR_Z[0] - 1, ROD_Z))
            c = c.cut(box2(x - 13, x + 13, yb + 10, yb + 14, CARR_Z[1] - 3.5, CARR_Z[1] + 1))   # zip-tie channel
    c = c.cut(box2(BELT_X - 7.6, BELT_X - 5.6, y0 - 1, y1 + 1, BELT_Z - 4, BELT_Z + 4))  # belt slot
    for yy in (y0 + CLAMP_AT[0] + 6, y0 + CLAMP_AT[1] - 6):                   # clamp screws squeeze the slot
        c = c.cut(cyl_x(M3, 150, 167, yy, BELT_Z))
    for x in (68, 90):                                                        # blade bolts, M3, nut on the arm's back
        c = c.cut(cyl_y(M3, face, y1 + 1, x, (ARM_Z[0] + ARM_Z[1]) / 2))
    return c


def blade(face):
    """PETG. The pushing blade: a plate with a stiffening flange along the top. It reaches back over the bed
    from the arm and passes between the guide rails, 5 mm above the bed."""
    b = union_all([
        box2(BLADE_X[0], BLADE_X[1], face, face + BLADE_T, Z_BLADE, Z_BLADE + BLADE_H),
        box2(BLADE_X[0], BLADE_X[1], face + BLADE_T, face + BLADE_T + 30, Z_BLADE + BLADE_H - 5, Z_BLADE + BLADE_H),
        box2(BLADE_X[0], 58, face + BLADE_T, face + BLADE_T + 12, Z_BLADE + 30, Z_BLADE + 34),    # middle rib
    ])
    for x in (68, 90):
        b = b.cut(cyl_y(M3, face - 1, face + BLADE_T + 1, x, (ARM_Z[0] + ARM_Z[1]) / 2))
    return b


def lm8uu(x, yc):
    return cyl_y(15, yc - 12, yc + 12, x, ROD_Z).cut(cyl_y(8, yc - 13, yc + 13, x, ROD_Z))


def pusher_moving(face=PUSH_HOME):
    """(name, shape, colour) of everything that travels with the blade."""
    y0 = face + BLADE_AHEAD
    parts = [("pusher_carriage", carriage(face), PETG_C), ("pusher_blade", blade(face), PETG_C)]
    for x in ROD_X:
        for i, yb in enumerate((y0 + 15, y0 + 39)):
            parts.append((f"pusher_lm8uu_{x}_{i}", lm8uu(x, yb), STEEL))
    return parts


def limit_switch(side):
    """20 x 6.4 x 10 mm lever microswitch in its pocket, lever out towards the carriage."""
    x0, x1, z0, z1 = SW_POCKET
    yi = switch_face(side)
    body = box2(x0 + 0.2, x1 - 0.2, *sorted((yi, yi + side * 6.4)), z0 + 0.2, z1 - 0.2)
    lever = box2(x0 + 2, x1 - 4, *sorted((yi, yi - side * 0.9)), z0 + 3, z0 + 6)
    return body.union(lever)


LEFT_HOLDER = rod_holder(motor=True)
RIGHT_HOLDER = rod_holder(motor=False).mirror("XZ").union(idler_arm())
add_print("pusher", "pusher_motor_holder", LEFT_HOLDER, "ABS", orient=[("y", 90)],
          note="left rod holder + stepper plate. ABS: the stepper runs hot. Print on its front face. 5 walls, 40 %")
add_print("pusher", "pusher_idler_holder", RIGHT_HOLDER, "PETG", orient=[("y", 90)],
          note="right rod holder + belt idler arm. Print on its front face. 5 walls, 40 %")
for x in ROD_X:
    add("pusher", f"pusher_rod_{x}", cyl_y(8, -W / 2, W / 2, x, ROD_Z), STEEL)
add("pusher", "nema17", box2(BELT_X - NEMA[0] / 2, BELT_X + NEMA[0] / 2, PULLEY_Y - NEMA[0] / 2, PULLEY_Y + NEMA[0] / 2,
                             Z_RIM - 4 - NEMA[1], Z_RIM - 4).edges("|Z").chamfer(4)
    .union(cyl_z(22, Z_RIM - 4, Z_RIM - 2, BELT_X, PULLEY_Y)).union(cyl_z(5, Z_RIM - 2, PULLEY_Z[1], BELT_X, PULLEY_Y)), DARK)
add("pusher", "gt2_pulley", cyl_z(16, PULLEY_Z[0], BELT_Z - 3, BELT_X, PULLEY_Y)
    .union(cyl_z(12.2, BELT_Z - 3, BELT_Z + 3, BELT_X, PULLEY_Y))
    .union(cyl_z(16, BELT_Z + 3, PULLEY_Z[1], BELT_X, PULLEY_Y)).cut(cyl_z(5, 190, 230, BELT_X, PULLEY_Y)), ALU)
add("pusher", "gt2_idler", cyl_z(18, BELT_Z - 4.5, BELT_Z - 3, BELT_X, IDLER_Y)
    .union(cyl_z(12.2, BELT_Z - 3, BELT_Z + 3, BELT_X, IDLER_Y))
    .union(cyl_z(18, BELT_Z + 3, BELT_Z + 4.5, BELT_X, IDLER_Y)).cut(cyl_z(5, 200, 220, BELT_X, IDLER_Y)), ALU)
add("pusher", "idler_bolt_m5", cyl_z(5, Z_RIM + 6, BELT_Z + 6, BELT_X, IDLER_Y)
    .union(cyl_z(8.5, Z_RIM - 4, Z_RIM, BELT_X, IDLER_Y)), STEEL)
add("pusher", "idler_washers", cyl_z(10, Z_RIM + 6, BELT_Z - 4.5, BELT_X, IDLER_Y).cut(cyl_z(5.3, 190, 210, BELT_X, IDLER_Y)), STEEL)
BELT_SPAN = PULLEY_Y - IDLER_Y
add("pusher", "gt2_belt", cq.Workplane("XY").slot2D(BELT_SPAN + 15, 15, angle=90).extrude(6)
    .cut(cq.Workplane("XY").slot2D(BELT_SPAN + 12.2, 12.2, angle=90).extrude(6))
    .translate((BELT_X, (PULLEY_Y + IDLER_Y) / 2, BELT_Z - 3)), RUBBER)
add("pusher", "limit_switch_home", limit_switch(1), BLACK)
add("pusher", "limit_switch_end", limit_switch(-1), BLACK)
MOVING = [n for n, _, _ in pusher_moving()]
for name, wp, colour in pusher_moving():
    add("pusher", name, wp, colour)
printed["pusher_carriage"] = dict(shape=carriage(PUSH_HOME), material="PETG", qty=1, orient=[("x", 180)],
                                  note="print upside down (top on the bed). 5 walls, 40 % infill")
printed["pusher_blade"] = dict(shape=blade(PUSH_HOME), material="PETG", qty=1, orient=[("x", 90)],
                               note="print lying on its pushing face. 4 walls, 30 % infill")


# ============================== SENSORS ==============================
US_PCB_X = (L / 2 - 12 - 1.6, L / 2 - 12)   # HC-SR04 board: its transducers (12 mm) end flush with the front face


def us_clip(y):
    """PLA. Holds one HC-SR04 behind the front wall, its transducers through the wall. The board slides up into
    the two slots until it stops; 2 screws from outside the robot."""
    z0, z1 = US_Z - 14, US_Z + 14
    c = union_all([box2(XI - 14, XI, y + s * 21, y + s * 30, z0, z1) for s in (-1, 1)]
                  + [box2(XI - 14, XI, y - 30, y + 30, US_Z + 10.3, z1)])
    for s in (-1, 1):
        c = c.cut(box2(US_PCB_X[0] - 0.2, US_PCB_X[1] + 0.2, *sorted((y + s * 21 - s, y + s * 23)), z0 - 1, US_Z + 10.3))
        c = c.cut(cyl_x(M3_TAP, XI - 10, XI + 1, y + s * 27, US_Z))
    return c


def tof_bracket():
    """PLA. Clamps the VL53L0X board against the right wall, behind its window."""
    b = box2(TOF_X - 22, TOF_X + 22, -YI, -YI + 6, TOF_Z - 10, TOF_Z + 10)
    b = b.cut(box2(TOF_X - 12.7, TOF_X + 12.7, -YI - 1, -YI + 1.8, TOF_Z - 6.7, TOF_Z + 6.7))      # the board
    b = b.cut(box2(TOF_X - 9, TOF_X + 9, -YI - 1, -YI + 7, TOF_Z - 6.5, TOF_Z - 3))                # its pins
    for dx in (-16, 16):
        b = b.cut(cyl_y(M3_TAP, -YI - 1, -YI + 7, TOF_X + dx, TOF_Z))
    return b


def line_bar():
    """PLA. Bar under the front of the floor; the 8-channel line sensor board screws under it (slot fits any hole
    spacing). Add M3 standoffs so the sensors end up 5-8 mm above the floor."""
    b = box2(LINE_X - 8, LINE_X + 8, -62, 62, Z_F0 - 5, Z_F0)
    b = b.cut(holes_z(LINE_XY, M3, Z_F0 - 6, Z_F0 + 1))
    return b.cut(cq.Workplane("XY").slot2D(90 + M3, M3, angle=90).extrude(7).translate((LINE_X, 0, Z_F0 - 6)))


for i, y in enumerate(US_Y):
    add_print("sensors", f"ultrasonic_clip_{i}", us_clip(y), "PLA", qty=3, file="ultrasonic_clip",
              note="print standing as shown")
    add("sensors", f"hc_sr04_{i}", box2(*US_PCB_X, y - 22.5, y + 22.5, US_Z - 10, US_Z + 10)
        .union(cyl_x(16, US_PCB_X[1], L / 2, y - 13, US_Z)).union(cyl_x(16, US_PCB_X[1], L / 2, y + 13, US_Z)), PCB_BLUE)
add_print("sensors", "tof_bracket", tof_bracket(), "PLA", orient=[("x", 90)], note="print on its flat back")
add("sensors", "vl53l0x", box2(TOF_X - 12.5, TOF_X + 12.5, -YI, -YI + 1.6, TOF_Z - 6.5, TOF_Z + 6.5), PCB)
add_print("sensors", "line_sensor_bar", line_bar(), "PLA", note="print flat")
add("sensors", "line_sensor_8ch", box2(LINE_X - 11, LINE_X + 11, -50, 50, 13.4, 15)
    .union(union_all([box2(LINE_X - 3, LINE_X + 3, y - 2.5, y + 2.5, 7, 13.4) for y in range(-42, 43, 12)])), PCB)


# ============================== BUMPER, PANELS, BATTERY ==============================
def bumper():
    """TPU. Soft strip across the front, below the ultrasonic sensors. Bolts sit in counterbores."""
    b = box2(L / 2, L / 2 + 10, -100, 100, 30, 58).edges("|Z and >X").fillet(4).edges("|Y and >X").fillet(3)
    for y, z in BUMPER_YZ:
        b = b.cut(cyl_x(M3, L / 2 - 1, L / 2 + 11, y, z)).cut(cyl_x(7, L / 2 + 5.5, L / 2 + 11, y, z))
    return b


REAR_PANEL_X = (-XI + 0.2, BED[0] - 3)
ESTOP_D, SWITCH_D, LED_D, BUZZER_D, BUTTON_D = 22.5, 12.2, 5.2, 12.2, 16.2
RIGHT_PANEL_HOLES = [((-158, -30), LED_D), ((-158, -50), LED_D), ((-178, -40), BUZZER_D), ((-178, -95), BUTTON_D)]
JOINER_XY = [(x, s * 10) for x in (-175, -158) for s in (1, -1)]


def rear_panel_left():
    """PLA. Left half of the rear panel: E-stop and master switch. It sits on the lip at the top of the tub."""
    p = box2(REAR_PANEL_X[0], REAR_PANEL_X[1], 0, YI - 0.2, Z_RIM, Z_RIM + 5).edges("|Z and <X and >Y").fillet(9)
    p = p.cut(holes_z([(x, y) for x, y in REAR_PANEL_XY if y > 0] + [(x, y) for x, y in JOINER_XY if y > 0],
                      M3, Z_RIM - 1, Z_RIM + 6))
    p = p.cut(cyl_z(ESTOP_D, Z_RIM - 1, Z_RIM + 6, *ESTOP_POS)).cut(cyl_z(SWITCH_D, Z_RIM - 1, Z_RIM + 6, *SWITCH_POS))
    return p


def rear_panel_right():
    """PLA. Right half: green and red LEDs, buzzer, and a GO button (optional)."""
    p = rear_panel_left().mirror("XZ")
    p = p.union(cyl_z(ESTOP_D + 0.5, Z_RIM, Z_RIM + 5, ESTOP_POS[0], -ESTOP_POS[1]))
    p = p.union(cyl_z(SWITCH_D + 0.5, Z_RIM, Z_RIM + 5, SWITCH_POS[0], -SWITCH_POS[1]))
    for (x, y), d in RIGHT_PANEL_HOLES:
        p = p.cut(cyl_z(d, Z_RIM - 1, Z_RIM + 6, x, y))
    return p


def panel_joiner():
    """PLA. Joins the two rear panel halves from underneath."""
    return box2(-183, -150, -16, 16, Z_RIM - 3, Z_RIM).cut(holes_z(JOINER_XY, M3_TAP, Z_RIM - 4, Z_RIM + 1))


def front_cover():
    """PLA. Covers the front strip between the rod holders, under the pusher carriage."""
    c = box2(HB[0], XI - 0.2, -99.5, 87.5, Z_RIM, Z_RIM + 3).union(box2(HB[0], HB[0] + 3, -99.5, 87.5, Z_RIM - 6, Z_RIM))
    return c.cut(holes_z(FRONT_COVER_XY, M3, Z_RIM - 1, Z_RIM + 4))


def battery_tray():
    """PETG. Holds the battery on the floor, across the Y = 0 seam (so it also joins the two rear tiles).
    The battery slides out through the hatch in the left wall; a velcro strap goes through the slots."""
    x0, x1 = BATT_X - 28.5, BATT_X + 28.5
    t = union_all([
        box2(x0, x1, -96, 96, Z_F1, Z_F1 + 3),
        box2(x0, x0 + 3, -83, 80, Z_F1, Z_F1 + 18), box2(x1 - 3, x1, -83, 80, Z_F1, Z_F1 + 18),
        box2(x0, x1, -83, -80, Z_F1, Z_F1 + 18),                                 # end stop
    ])
    for y in (-40, 40):
        t = t.cut(box2(BATT_X - 22, BATT_X + 22, y - 2, y + 2, Z_F1 - 1, Z_F1 + 4))
    return t.cut(holes_z(BATT_TRAY_XY, M4, Z_F1 - 1, Z_F1 + 4))


def battery_door():
    """PLA. Closes the battery hatch in the left wall; 2 M3 screws."""
    d = box2(BATT_X - 33, BATT_X + 33, W / 2, W / 2 + 3, Z_F1 - 1, Z_F1 + 56)
    d = d.union(box2(BATT_X - 26.6, BATT_X + 26.6, YI + 0.2, W / 2, Z_F1 + 1.4, Z_F1 + 45.6)
                .cut(box2(BATT_X - 24.6, BATT_X + 24.6, YI, W / 2, Z_F1 + 3.4, Z_F1 + 43.6)))
    for x, z in BATT_DOOR_XZ:
        d = d.cut(cyl_y(M3, YI, W / 2 + 4, x, z))
    return d


def grommet():
    """TPU. Lines the cable hole in the floor."""
    return cyl_z(16, 0, 1.5).union(cyl_z(9.8, -FLOOR, 0)).cut(cyl_z(6, -FLOOR - 1, 2.5))


add_print("body", "front_bumper", bumper(), "TPU", orient=[("y", -90)], note="print on its flat back. 3 walls, 15 % infill")
add_print("controls", "rear_panel_left", rear_panel_left(), "PLA", note="print flat, top face down is fine too")
add_print("controls", "rear_panel_right", rear_panel_right(), "PLA", note="print flat")
add_print("controls", "rear_panel_joiner", panel_joiner(), "PLA", note="print flat")
add_print("controls", "front_cover", front_cover(), "PLA", orient=[("x", 180)], note="print top face down")
add_print("electronics", "battery_tray", battery_tray(), "PETG", note="print flat")
add_print("electronics", "battery_door", battery_door(), "PLA", orient=[("x", -90)], note="print on its outer face")
for y in (-40, 40):
    add_print("electronics", f"battery_pad_{y}", box2(BATT_X - 20, BATT_X + 20, y - 10, y + 10, Z_F1 + 3, Z_F1 + 5),
              "TPU", qty=2, file="battery_pad", note="print flat; under the battery")
add_print("electronics", "cable_grommet", grommet().translate((170, 45, Z_F1)), "TPU", qty=2, orient=[("x", 180)],
          note="one in the floor's cable hole, one spare")

# ---------- bought parts (envelopes) ----------
z = Z_F1
add("electronics", "battery_3S_5000mAh", box2(BATT_X - 25, BATT_X + 25, -77.5, 77.5, z + 5, z + 5 + BATTERY[2]), BATT)
add("electronics", "logic_perfboard", box2(-190, -125, 35, 105, z + 8, z + 9.6), PCB)
add("electronics", "esp32_devkit", box2(-188, -133, 40, 68, z + 9.6, z + 22), PCB_BLUE)
add("electronics", "mcp23017", box2(-160, -130, 76, 98, z + 9.6, z + 14), PCB_RED)
add("electronics", "hx711", box2(-185, -151, 112, 133, z + 8, z + 12), PCB)
add("electronics", "buck_12V_5V", box2(-190, -139, -70, -44, z, z + 14), PCB_BLUE)
add("electronics", "fuse_holder", box2(-190, -150, -100, -85, z, z + 15), RED)
add("electronics", "wago_bus", box2(-190, -130, -135, -110, z, z + 16), BLACK)
add("electronics", "motor_driver_mdd10a", box2(50, 113, -135, -50, z + 6, z + 26), PCB_RED)
add("electronics", "stepper_perfboard", box2(55, 105, 50, 90, z + 6, z + 7.6), PCB)
add("electronics", "tmc2209", box2(70, 90, 60, 75, z + 7.6, z + 20), PCB_RED)
ex, ey = ESTOP_POS
add("controls", "estop_collar", cyl_z(30, Z_RIM + 5, Z_RIM + 11, ex, ey).cut(cyl_z(22, Z_RIM + 5, Z_RIM + 11, ex, ey)), YELLOW)
add("controls", "estop_mushroom", cyl_z(40, Z_RIM + 11, Z_RIM + 26, ex, ey), RED)
add("controls", "estop_body", cyl_z(22, Z_RIM - 4, Z_RIM + 5, ex, ey).union(box2(ex - 15, ex + 15, ey - 20, ey + 20,
                                                                                    Z_RIM - 44, Z_RIM - 4)), BLACK)
sx_, sy_ = SWITCH_POS
add("controls", "master_switch", box2(sx_ - 9, sx_ + 9, sy_ - 14, sy_ + 14, Z_RIM - 25, Z_RIM)
    .union(cyl_z(6, Z_RIM + 5, Z_RIM + 20, sx_, sy_)), BLACK)
for (x, y), d in RIGHT_PANEL_HOLES:
    colour = GREEN if (x, y) == (-158, -30) else RED if d == LED_D else BLACK
    add("controls", f"panel_item_{x}_{y}", cyl_z(d - 0.2, Z_RIM - 8, Z_RIM + 8, x, y), colour)

# a 180 x 180 x 140 mm test package, against the parked blade (for the pictures only)
PACKAGE = box2(BED_X - 90, BED_X + 90, PUSH_HOME - 5 - 180, PUSH_HOME - 5, BED_TOP, BED_TOP + 140)


# ============================== EXPORT ==============================
def solid(wp):
    return cq.Compound.makeCompound(wp.vals())


def build_assembly(push_face=PUSH_HOME, package=False):
    """The whole robot. push_face = Y of the blade's pushing face (PUSH_HOME = parked ... PUSH_END = fully out)."""
    moved = {} if push_face == PUSH_HOME else {n: wp for n, wp, _ in pusher_moving(push_face)}
    assy = cq.Assembly(name="cargo_robot_v4")
    for gname, items in groups.items():
        sub = cq.Assembly(name=gname)
        for name, wp, color in items:
            sub.add(moved.get(name, wp), name=name, color=cq.Color(*color))
        assy.add(sub, name=gname)
    if package:
        assy.add(PACKAGE, name="package", color=cq.Color(0.72, 0.53, 0.30, 1))
    return assy


def print_pose(wp, orient):
    """Turn a part to its print orientation and put it on the build plate, centred."""
    axes = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}
    for axis, deg in orient:
        wp = wp.rotate((0, 0, 0), axes[axis], deg)
    bb = solid(wp).BoundingBox()
    return wp.translate((-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))


DENSITY = {"PLA": 1.24, "PETG": 1.27, "ABS": 1.04, "TPU": 1.21}      # g/cm3


def grams(shape, material, walls=1.6, infill=0.25):
    """Rough printed mass: a solid skin `walls` mm thick plus sparse infill inside."""
    v, a = shape.Volume() / 1000, shape.Area() / 100                  # cm3, cm2
    skin = min(v, a * walls / 10)
    return DENSITY[material] * (skin + (v - skin) * infill)


def stl_name(name, qty):
    return f"{name}_x{qty}.stl" if qty > 1 else f"{name}.stl"


BOM = [
    # group, item, qty, notes
    ("Control", "ESP32 DevKit, 38-pin (ESP32-WROOM-32)", 1, "the AGV brain; buy a spare"),
    ("Control", "MCP23017 I2C expander module", 1, "8 line sensors, 2 limit switches, LEDs"),
    ("Control", "Perfboard 70 x 90 mm + female headers", 2, "logic board (ESP32 + MCP23017) and stepper board"),
    ("Drive", "JGB37-520 12 V gear motor with Hall encoder, 60-100 rpm, 6 mm D shaft", 2, ""),
    ("Drive", f"Wheel {WHEEL_D} x {WHEEL_W} mm with 6 mm D-shaft hub", 2, ""),
    ("Drive", "Dual motor driver Cytron MDD10A (or 2 x BTS7960)", 1, "PWM + DIR per motor"),
    ("Drive", "Metal ball caster, 1 inch ball, about 30 mm tall, 2 mounting holes 26-36 mm apart", 2,
     "other heights: change CASTER_H and re-run"),
    ("Sensors", "8-channel IR line sensor array (TCRT5000 type) with digital outputs", 1, ""),
    ("Sensors", "HC-SR04 ultrasonic sensor", 3, "+1 spare. 1k/2k divider on each ECHO"),
    ("Sensors", "VL53L0X time-of-flight distance sensor module", 1, "side-facing: gap to the conveyor"),
    ("Payload", "Bar load cell 5 kg (80 x 12.7 x 12.7 mm)", 1, ""),
    ("Payload", "HX711 load cell amplifier", 1, ""),
    ("Pusher", "NEMA17 stepper, 40 N.cm or more, 34-48 mm long, 5 mm shaft", 1, ""),
    ("Pusher", "TMC2209 stepper driver (or A4988) + 100 uF capacitor", 1, ""),
    ("Pusher", "GT2 20-tooth pulley, 5 mm bore", 1, ""),
    ("Pusher", "GT2 20-tooth idler pulley, 5 mm bore (toothless or toothed)", 1, ""),
    ("Pusher", "GT2 belt, 6 mm wide, open, 1 m", 1, "uses about 0.55 m"),
    ("Pusher", "Smooth rod 8 mm x 300 mm", 2, ""),
    ("Pusher", "LM8UU linear bearing", 4, ""),
    ("Pusher", "Lever microswitch 20 x 10 x 6.4 mm (KW11 / SS-5GL type)", 2, "home and end"),
    ("Power", "3S LiPo 11.1 V 5000 mAh (about 155 x 50 x 35 mm), XT60 + balance charger + LiPo bag", 1, ""),
    ("Power", "Buck converter 12 V -> 5 V, 5 A (XL4015)", 1, "ESP32 and sensors"),
    ("Power", "Inline fuse holder + 10 A fuses, WAGO 221 connectors, XT60 pairs", 1, ""),
    ("Power", "Emergency stop 22 mm, latching, NC contact", 1, ""),
    ("Power", "Toggle switch 12 mm panel mount, 12 V DC 15 A", 1, ""),
    ("Panel", "LED 5 mm green + red, active buzzer 12 mm, push button 16 mm (optional)", 1, ""),
    ("Hardware", "M4 x 16 bolts + nuts (tile seams, pods, motor mounts, battery tray)", 40, ""),
    ("Hardware", "M4 x 30 bolts + nuts (motor clamps), M4 x 20 (bridge, rod holders)", 12, ""),
    ("Hardware", "M3 x 10/16 bolts + nuts, M3 self-tapping screws x 12, M3 standoffs", 1, "one assortment box"),
    ("Hardware", "M5 x 25 bolt + nut + 4 washers (belt idler)", 1, ""),
    ("Hardware", "M2 x 10 screws (limit switches), zip ties, velcro strap 20 mm", 1, ""),
]


def main():
    here = Path(__file__).parent
    out, prints = here / "output", here.parent / "print"
    for sub in ("parts_step", "views"):
        (out / sub).mkdir(parents=True, exist_ok=True)
    for mat in DENSITY:
        (prints / mat).mkdir(parents=True, exist_ok=True)

    build_assembly().export(str(out / "robot_assembly.step"))
    print("wrote robot_assembly.step")

    rows = []
    for name, p in printed.items():
        posed = print_pose(p["shape"], p["orient"])
        cq.exporters.export(posed, str(prints / p["material"] / stl_name(name, p["qty"])),
                            tolerance=0.05, angularTolerance=0.1)
        cq.exporters.export(p["shape"], str(out / "parts_step" / f"{name}.step"))
        s = solid(posed)
        bb = s.BoundingBox()
        g = grams(s, p["material"])
        rows.append([f"print/{p['material']}/{stl_name(name, p['qty'])}", p["material"], p["qty"],
                     f"{bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f}", round(g), round(g * p["qty"]), p["note"]])
    rows.sort(key=lambda r: (["PETG", "ABS", "PLA", "TPU"].index(r[1]), r[0]))
    with open(out / "print_list.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["File", "Material", "Qty", "Size as printed (mm)", "Grams each (approx.)",
                    "Grams total (approx.)", "How to print"])
        w.writerows(rows)
    print(f"wrote {len(rows)} STL files to ../print and print_list.csv")

    with open(out / "bom.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Group", "Item", "Qty", "Notes"])
        w.writerows(BOM)
    print("wrote bom.csv")


if __name__ == "__main__":
    main()
