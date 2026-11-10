"""
Parametric CAD model - Phase II autonomous cargo robot, DESIGN v2 (simplified) with a tilting cargo bed.

Run:   python robot_cad.py          (needs: pip install cadquery)
Writes ./output/
  robot_assembly.step         full coloured assembly, bed flat -> SolidWorks / Fusion 360 / Onshape / FreeCAD
  robot_assembly_tipped.step  the same with the bed tipped to TIP_ANGLE
  parts/*.stl, *.step         ONLY the parts to 3D-print, each already lying in its printing position
  dxf/*.dxf                   the four plates, ready for the laser cutter (every hole already in them)
  views/*.svg                 line drawings
  bom.csv                     bill of materials

Design choices that keep it simple to build:
  - frame: 6 x M6 threaded rod + nuts, instead of 6 printed pillars with heat-set inserts
  - drive: the JGB37-520 110 RPM motor set (motor + steel bracket + coupling + 65 mm wheel), bought as one
  - casters: 2 printed ball casters (a 25 mm steel ball in a cup), each on an M8 bolt through the plate - the
    height is set with the nuts, so getting all 4 wheels onto the floor takes a spanner, not shims
  - one ESP32 runs everything, with no I/O expander: wheels, encoders, 3 IR line sensors, the ultrasonic sensor,
    the bed servos, the buttons and lights
  - tilting bed: two servos sit on the hinge line and turn the bed directly - no screws, levers or linkages.
    The servo shafts ARE the hinge. The bed rests on two posts when flat, so the servos only work while tipping.
  - servo mounts: the ears are clamped, and the mount slides +-4 mm, so it doesn't depend on servo details the
    maker doesn't publish (where the shaft sits along the body, where the ear notches are)
  - the cargo plate stands on 4 printed risers on the tilting plate: high enough to clear the horn adapters,
    and wide enough that its right edge overhangs the deck when tipped, so the package slides clear
  - the holes in the plates are laser-cut (servo mounts, risers, the 22 mm E-stop hole, the switch, button and
    LED holes) - nothing is drilled by hand except the motor brackets in the plywood (and the 8 countersinks
    on the cargo plate)
  - check_render.py checks every bolt's path through the parts it joins, collisions with the bed at every
    angle, and that each printed part fits a 200 x 200 mm printer with a flat face down
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

WHEEL_D, WHEEL_W = 65, 26        # drive wheels: the 65 mm wheels of the JGB37-520 motor set
AXLE_Z = WHEEL_D / 2
MOTOR_DROP = 22                  # motor bracket: motor axis -> plate underside. The real value depends on the bracket
                                 # and on how the motor is turned in it: the casters and the line sensors adjust
MOTOR_FACE_Y = W / 2 - 5         # outer face of the bracket's upright
BRACKET = (40, 40, 47, 1.5)      # the set's steel L-bracket: base X x Y, upright height, sheet thickness
WHEEL_Y = W / 2 + 28             # wheel centre-plane (about; it depends on the coupling)

CASTER_X = 160                   # 2 printed ball casters at X = +-160, each on an M8 bolt through the plate
BALL_D = 25.4                    # 1" steel ball (a 25 mm glass marble also fits)

ROD_PTS = [(x, s * (W / 2 - 15)) for x in (-190, 60, 190) for s in (-1, 1)]   # M6 threaded rods

ESTOP_POS = (150, 110)           # 22 mm E-stop, front-left of the deck
SWITCH_POS = (155, 45)           # 12 mm toggle switch (master), front of the deck
BUTTON_POS = (-165, 30)          # 12 mm start button, rear of the deck
LED_POS = {"green": (-165, 68), "red": (-165, 92)}

# ---- tilting bed ----
TIP_ANGLE = 40                   # bed angle when it drops the package (deg)
Y_AX = -122                      # hinge axis = the two servo shafts: Y position (bed tips towards -Y)
SERVO = dict(L=65, W=30, H=48,   # DS5160 60 kg.cm (maker's datasheet drawing): body L x W x H, shaft on the L x W face
             shaft=16,           # shaft centre -> near end of the body: NOT on the datasheet; the mount slides to suit
             ear_h=31,           # mounting ears: underside 31 mm above the case bottom (datasheet)
             ear_t=3,            # ear thickness (about)
             ear_ext=5,          # ears stick out 5 mm past each end: 75 mm overall (datasheet)
             ear_holes=17,       # the 2 open-ended notches in each ear are 17 mm apart (datasheet; not used by the mount)
             boss_d=14, boss_h=1)
HORN_D, HORN_T, HORN_HUB = 38, 3, 5   # aluminium DISC horn (not the 4-arm horn: its arms would hit the deck)
HORN_SLOTS = (9.5, 15)           # the adapter's 4 radial slots fit horn screw circles of 19-30 mm
CLAMP_DY = 40.5                  # servo ear clamps: screw this far from the middle of the servo body (along L)
MOUNT_SLIDE = 4                  # servo mount foot slots: +- this much along Y, to line the shaft up with the hinge
ADAPTER_WALL = 6                 # printed horn adapter: wall the horn screws onto
TILT_X = 118                    # tilting plate spans X = -118..118
TILT_Y1 = 125                    # ... and Y = Y_AX..125
RISER = (80, 16, 22.7)           # printed riser bar (L x W x H): 4 of them carry the cargo plate above the
                                 # horn adapters, so its right edge overhangs the deck when the bed tips
RISER_X = (-55, 55)              # centres of the two riser columns
RISER_Y = 110
RISER_HOLES = (8, 20)            # a riser's 2 screw holes at each end lie this far from that end
M4_NUT_SLOT = (7.4, 3.6)         # side slot an M4 nut slides into (width, height)
CARGO = (260, 280)               # cargo plate, centred on X = 0, Y = 0
CARGO_X = 0
REST_PTS = [(-40, 95), (40, 95)] # printed rest posts under the bed's left side

# ---- electronics and sensors ----
ESP_POS = (110, 62)              # ESP32 DevKit on its screw-terminal breakout (USB end towards +Y)
IO_POS = (110, -45)              # I/O board: resistors and dividers on perfboard
BATTERY = (50, 155, 35)          # 3S 5000 mAh LiPo lying across the robot (X, Y, Z)
LINE_X = 98                      # where the 3 IR modules look at the floor (firmware: stop_mm)
LINE_Y = (18, 0, -18)            # sensor 1 (left), 2 (middle), 3 (right)
IR_PCB = (32, 14, 1.6)           # IR obstacle-avoidance module (FC-51 type, 3 pins): length x width x thickness
IR_TIP_Z = 8                     # the IR LEDs' tips, this high above the floor
BAR_X = LINE_X - 7               # front face of the sensor bar's upright (the boards sit in front of it)
LINE_BRKT_HOLES = [(BAR_X - 22, -24), (BAR_X - 22, 24)]   # behind the I/O board and the ESP32 (x < 75)
LINE_CABLE_SLOT = (BAR_X - 40, 0)
SONAR_HOLES = [(178, -20), (178, 20)]
SERVO_CABLE_SLOTS = [(150, -50), (-150, -50)]

# ---- derived Z levels ----
Z_BASE = AXLE_Z + MOTOR_DROP     # underside of bottom plate = 54.5
Z_LOWER = Z_BASE + BASE_T        # top of bottom plate = 60.5
Z_DECK = Z_LOWER + BAY_H         # underside of deck = 160.5
Z_DECK_TOP = Z_DECK + DECK_T     # 165.5
SERVO_FOOT = 7                   # printed servo mount: floor under the servo
Z_AX = Z_DECK_TOP + SERVO_FOOT + SERVO["W"] / 2   # servo shaft (hinge axis) = 187.5
Z_TILT = Z_AX - TILT_T / 2       # underside of the tilting plate = 184.5 (axis in the middle of the plate)
Z_TILT_TOP = Z_TILT + TILT_T     # 190.5
Z_CARGO = Z_TILT_TOP + RISER[2] # underside of cargo plate = 213.2
X_SERVO_TOP = TILT_X + ADAPTER_WALL + HORN_T + HORN_HUB + SERVO["boss_h"]   # servo shaft face = 133
AXIS = ((0, Y_AX, Z_AX), (1, Y_AX, Z_AX))    # tipping axis (along +X)


def riser_holes(bottom):
    """XY of the riser screws: bottom = the 8 that come up through the tilting plate, else the 8 down through the
    cargo plate (countersunk, so nothing sticks up where the package slides)."""
    pts = []
    for cx in RISER_X:
        outer = -1 if cx < CARGO_X else 1
        for d in RISER_HOLES:
            x = cx + outer * (RISER[0] / 2 - d) if bottom else cx - outer * (RISER[0] / 2 - d)
            pts += [(x, RISER_Y), (x, -RISER_Y)]
    return pts


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


def slot_y(cx, cy, z0, h, centres, width):
    """Round-ended slot along Y."""
    return cq.Workplane("XY").slot2D(centres + width, width, 90).extrude(h).translate((cx, cy, z0))


def mirror_x(wp):
    """Mirror image front <-> rear (X -> -X)."""
    return wp.mirror("YZ")


# ============================== MODEL ==============================
groups = {}          # group name -> list of (name, workplane, colour)
fabricated = {}      # printed parts: name -> workplane (exported as STL + STEP to output/parts/)
plates = {}          # laser-cut plates: name -> workplane (exported as DXF to output/dxf/)


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
    p = p.cut(box(12, 20, h, *LINE_CABLE_SLOT, zc))   # the 3 line-sensor cables come up here
    return p


SV = SERVO
X_EAR = X_SERVO_TOP + SV["H"] - SV["ear_h"]   # front servo: its ears' underside = the face of its printed mount
Y_SERVO_MID = Y_AX - SV["shaft"] + SV["L"] / 2  # middle of the servo body along its length


def servo_mount_holes(front=True):
    """The 3 M4 bolts that hold a servo mount to the deck (under the servo body, behind its upright)."""
    s = 1 if front else -1
    return [(s * (X_EAR + 10), Y_SERVO_MID - 25), (s * (X_EAR + 10), Y_SERVO_MID + 25),
            (s * (X_SERVO_TOP + SV["H"] - 7), Y_SERVO_MID)]


def top_deck():
    zc, z0, h = Z_DECK + DECK_T / 2, Z_DECK - 1, DECK_T + 2
    p = box(L, W, DECK_T, 0, 0, zc)
    p = drill(p, ROD_PTS, 6.5, z0, h)
    p = drill(p, [ESTOP_POS], 22.5, z0, h)
    p = drill(p, [SWITCH_POS, BUTTON_POS], 12.2, z0, h)
    p = drill(p, list(LED_POS.values()), 6.2, z0, h)            # 5 mm LEDs in panel clips
    p = drill(p, servo_mount_holes(True) + servo_mount_holes(False) + REST_PTS, 4.5, z0, h)
    for x, y in SERVO_CABLE_SLOTS:                              # servo cables go down into the bay
        p = p.cut(box(16, 12, h, x, y, zc))
    return p


plates["bottom_plate"], plates["top_deck"] = bottom_plate(), top_deck()
add("chassis", "bottom_plate", plates["bottom_plate"], PLYWOOD)
add("chassis", "top_deck", plates["top_deck"], ACRYLIC)

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


# ---------- Drive: the JGB37-520 motor set (motor, steel L-bracket, hex coupling, 65 mm wheel) ----------
def bracket_left():
    """The set's steel L-bracket: base screwed under the plate (holes drilled by hand), upright hangs down."""
    bx, by, bh, t = BRACKET
    vert = span(-bx / 2, bx / 2, MOTOR_FACE_Y - t, MOTOR_FACE_Y, Z_BASE - bh, Z_BASE)
    base = span(-bx / 2, bx / 2, MOTOR_FACE_Y - by, MOTOR_FACE_Y, Z_BASE - t, Z_BASE)
    b = vert.union(base).cut(cyl_y(14, MOTOR_FACE_Y - t - 1, MOTOR_FACE_Y + 1, 0, AXLE_Z))
    for k in range(6):                                   # 6 x M3 on a 31 mm circle into the gearbox face
        a = math.radians(30 + 60 * k)
        b = b.cut(cyl_y(3.2, MOTOR_FACE_Y - t - 1, MOTOR_FACE_Y + 1, 15.5 * math.cos(a), AXLE_Z + 15.5 * math.sin(a)))
    pts = [(x, MOTOR_FACE_Y - y) for x in (-15, 15) for y in (10, 34)]
    return drill(b, pts, 3.5, Z_BASE - t - 1, t + 2)


def gearmotor_left():
    f = MOTOR_FACE_Y - BRACKET[3]                        # gearbox face against the bracket's upright
    return (cyl_y(37, f - 24, f, 0, AXLE_Z)              # gearbox (24 mm long on the 110 RPM version)
            .union(cyl_y(35, f - 57, f - 24, 0, AXLE_Z))  # motor
            .union(cyl_y(33, f - 72, f - 57, 0, AXLE_Z)))  # encoder cap


def wheel(yc):
    y0, y1 = yc - WHEEL_W / 2, yc + WHEEL_W / 2
    rim_d = WHEEL_D - 16
    tire = cyl_y(WHEEL_D, y0, y1, 0, AXLE_Z).cut(cyl_y(rim_d, y0 - 1, y1 + 1, 0, AXLE_Z))
    rim = cyl_y(rim_d, y0 + 2, y1 - 2, 0, AXLE_Z).cut(cyl_y(6, y0, y1, 0, AXLE_Z))
    for i in range(5):
        a = math.radians(72 * i)
        rim = rim.cut(cyl_y(10, y0, y1, 15 * math.cos(a), AXLE_Z + 15 * math.sin(a)))
    return tire, rim


br = bracket_left()
add("drive", "motor_bracket_L", br, DARK_STEEL)
add("drive", "motor_bracket_R", br.mirror("XZ"), DARK_STEEL)
add("drive", "gearmotor_L", gearmotor_left(), MOTOR)
add("drive", "gearmotor_R", gearmotor_left().mirror("XZ"), MOTOR)
shaft = cyl_y(6, MOTOR_FACE_Y - BRACKET[3], MOTOR_FACE_Y - BRACKET[3] + 15.5, 0, AXLE_Z)       # 15.5 mm D-shaft
coupling = (cyl_y(14, MOTOR_FACE_Y + 2, WHEEL_Y - WHEEL_W / 2, 0, AXLE_Z)                      # hex coupling
            .cut(cyl_y(6.2, MOTOR_FACE_Y, WHEEL_Y, 0, AXLE_Z)))
for side, s in (("L", 1), ("R", -1)):
    add("drive", f"shaft_{side}", shaft if s > 0 else shaft.mirror("XZ"), STEEL)
    add("drive", f"coupling_{side}", coupling if s > 0 else coupling.mirror("XZ"), ALU)
    tire, rim = wheel(s * WHEEL_Y)
    add("drive", f"tire_{side}", tire, RUBBER)
    add("drive", f"rim_{side}", rim, YELLOW)


# ---------- Casters: 2 printed ball casters, each on an M8 bolt through the plate ----------
BALL_Z = BALL_D / 2                  # ball centre
RING_TOP = BALL_Z - 6                # the retaining ring's top face: 6 mm under the ball centre
RING_T = 2.4
CUP_D = 40
CUP_HEAD = (BALL_D + 0.6, BALL_D + 6.2)  # M8 bolt-head pocket: from just above the ball to its top
CUP_TOP = CUP_HEAD[1] + 4            # top face of the cup
CUP_SCREW_R = 16.6                   # 3 x M3 hold the ring on


def caster_cup(x):
    """Holds the ball from above on a 45 deg cone (a ring of contact: grease it). The M8 bolt's head sits in a hex
    pocket above the ball, its shank goes up through the plate: 2 nuts set the height, as with any threaded stem."""
    r = BALL_D / 2
    cav_r = r + 0.4                                  # 26.2 mm bore
    apex = BALL_Z + r * math.sqrt(2)                 # 45 deg cone that touches the ball
    cup = cyl_z(CUP_D, CUP_TOP - RING_TOP, x, 0, RING_TOP)
    cavity = (cq.Workplane("XZ").polyline([(0, RING_TOP - 1), (cav_r, RING_TOP - 1), (cav_r, apex - cav_r), (0, apex)])
              .close().revolve(360, (0, 0, 0), (0, 1, 0)).translate((x, 0, 0)))
    head = cq.Workplane("XY").polygon(6, 13.4 / math.cos(math.pi / 6)).extrude(CUP_HEAD[1] - CUP_HEAD[0])
    cup = cup.cut(cavity).cut(head.translate((x, 0, CUP_HEAD[0]))).cut(cyl_z(8.5, 10, x, 0, CUP_HEAD[1] - 1))
    pts = [(x + CUP_SCREW_R * math.cos(math.radians(90 + 120 * k)), CUP_SCREW_R * math.sin(math.radians(90 + 120 * k)))
           for k in range(3)]
    return drill(cup, pts, 2.8, RING_TOP - 1, 11)    # the ring's M3 screws cut their own thread


def caster_ring(x):
    """Stops the ball falling out when the robot is lifted (23.5 mm hole: it never touches the ball otherwise)."""
    ring = cyl_z(CUP_D, RING_T, x, 0, RING_TOP - RING_T).cut(cyl_z(23.5, RING_T, x, 0, RING_TOP - RING_T))
    for k in range(3):
        a = math.radians(90 + 120 * k)
        px, py = x + CUP_SCREW_R * math.cos(a), CUP_SCREW_R * math.sin(a)
        csk = cq.Solid.makeCone(3.2, 1.7, 1.5, cq.Vector(px, py, RING_TOP - RING_T), cq.Vector(0, 0, 1))
        ring = ring.cut(cyl_z(3.4, RING_T + 1, px, py, RING_TOP - RING_T - 0.5)).cut(cq.Workplane("XY").add(csk))
    return ring


def caster_hardware(x):
    """M8 x 50 bolt (head in the cup), washer + nut under the plate, washer + nut on top."""
    head_z = CUP_HEAD[1] - 5.3
    hw = (cq.Workplane("XY").polygon(6, 13 / math.cos(math.pi / 6)).extrude(5.3).translate((x, 0, head_z))
          .union(cyl_z(8, 50, x, 0, CUP_HEAD[1]))
          .union(hex_nut(13, 6.5, x, 0, Z_BASE - 8.1, 8)).union(washer(16, 1.6, x, 0, Z_BASE - 1.6, 8.4))
          .union(washer(16, 1.6, x, 0, Z_LOWER, 8.4)).union(hex_nut(13, 6.5, x, 0, Z_LOWER + 1.6, 8)))
    return hw


for tag, s in (("front", 1), ("rear", -1)):
    x = s * CASTER_X
    add("drive", f"caster_cup_{tag}", caster_cup(x), PRINT)
    add("drive", f"caster_ring_{tag}", caster_ring(x), PRINT)
    add("drive", f"caster_ball_{tag}", cq.Workplane("XY").sphere(BALL_D / 2).translate((x, 0, BALL_Z)), STEEL)
    add("drive", f"caster_bolt_{tag}", caster_hardware(x), DARK_STEEL)
fabricated["caster_cup"] = caster_cup(0).translate((0, 0, -RING_TOP))
fabricated["caster_ring"] = caster_ring(0).translate((0, 0, -(RING_TOP - RING_T)))


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
add("electronics", "buck_12V_5V", box(51, 26, 1.6, -60, 85, z + 1.0), PCB_BLUE)
add("electronics", "buck_inductor", cyl_z(20, 12, -60, 85, z + 1.8), BLACK)
add("electronics", "servo_buck_12V_7V4_pcb", box(60, 51, 1.6, -72, -95, z + 1.0), PCB_BLUE)
add("electronics", "servo_buck_heatsink", box(40, 20, 18, -72, -95, z + 1.8 + 9), BLACK)
add("electronics", "power_bus_wagos", box(60, 40, 20, -85, -20, z + 10), BLACK)
add("electronics", "inline_fuse_holder", box(40, 15, 15, 45, -110, z + 7.5), RED)
add("electronics", "buzzer_module", box(25, 15, 12, -110, 105, z + 6), BLACK)


# ---------- Line sensors: 3 IR modules on a printed bar under the bottom plate ----------
IR_LED_LEN = 5.5                     # the IR LED and receiver stick out this far past the board's end
IR_PCB_Z0 = IR_TIP_Z + IR_LED_LEN    # bottom edge of the boards
IR_HOLE_Z = IR_PCB_Z0 + IR_PCB[0] / 2  # the board's mounting hole: assumed mid-length; the slots allow +-6 mm
BAR_FLANGE_T = 5


def line_sensor_bar():
    """A flange screwed under the bottom plate and an upright web; each IR module stands on a 3 mm pad on the web's
    front, LEDs down, held by one M3 screw in a vertical slot (sets the height above the floor)."""
    zf = Z_BASE - BAR_FLANGE_T
    flange = span(BAR_X - 28, BAR_X, -32, 32, zf, Z_BASE)
    web = span(BAR_X - 4, BAR_X, -32, 32, IR_PCB_Z0 - 1.5, zf)
    b = flange.union(web)
    for y0 in (-32, 29):                                 # 2 gussets behind the web
        g = (cq.Workplane("XZ").polyline([(BAR_X - 4, zf), (BAR_X - 26, zf), (BAR_X - 4, IR_PCB_Z0 + 6)]).close()
             .extrude(-3).translate((0, y0, 0)))
        b = b.union(g)
    for y in LINE_Y:
        b = b.union(span(BAR_X, BAR_X + 3, y - 4.5, y + 4.5, IR_HOLE_Z - 9, IR_HOLE_Z + 9))     # pad
        slot = (cq.Workplane("YZ").slot2D(12 + 3.4, 3.4, 90).extrude(12).translate((BAR_X - 6, y, IR_HOLE_Z)))
        b = b.cut(slot)
    return drill(b, LINE_BRKT_HOLES, 3.4, zf - 1, BAR_FLANGE_T + 2)


add("sensors", "line_sensor_bar", line_sensor_bar(), PRINT, fab=True)
for i, y in enumerate(LINE_Y):
    x_pcb = BAR_X + 3
    add("sensors", f"ir_module_{i + 1}_pcb", span(x_pcb, x_pcb + IR_PCB[2], y - IR_PCB[1] / 2, y + IR_PCB[1] / 2,
                                                  IR_PCB_Z0, IR_PCB_Z0 + IR_PCB[0]), PCB_BLUE)
    for dy, col in ((-3, SILVER), (3, BLACK)):          # IR LED + receiver, looking down
        add("sensors", f"ir_module_{i + 1}_led{'a' if dy < 0 else 'b'}",
            cyl_z(5, IR_LED_LEN + 3, LINE_X, y + dy, IR_TIP_Z), col)
    add("sensors", f"ir_module_{i + 1}_pot", box(4.5, 6.5, 6.5, x_pcb + IR_PCB[2] + 2.25, y, IR_HOLE_Z + 7), BATT)
    add("sensors", f"ir_module_{i + 1}_plug", span(x_pcb + IR_PCB[2], x_pcb + IR_PCB[2] + 16, y - 4, y + 4,
                                                   IR_PCB_Z0 + IR_PCB[0] - 3, IR_PCB_Z0 + IR_PCB[0]), BLACK)


# ---------- Ultrasonic sensor (HC-SR04) at the front of the bay, looking straight ahead ----------
SONAR_X = 196.5                  # middle of the holder face (3 mm thick, its front at X = 198)
SONAR_Z = Z_LOWER + 40           # height of the transducer centres: leaves room under the board for its 4 plugs


def sonar_bracket():
    """Base on the bottom plate + one upright face; the HC-SR04 sits behind it on 2 small ledges, its 2 transducers
    through 2 18 mm windows (fits transducer spacings of 24-28 mm). Two M3 bolts hold its base to the bottom plate."""
    face = span(SONAR_X - 1.5, SONAR_X + 1.5, -26, 26, Z_LOWER + 4, SONAR_Z + 14)
    for y in (-13, 13):
        face = face.cut(cyl_x(18, SONAR_X - 3, SONAR_X + 3, y, SONAR_Z))
    face = face.cut(span(SONAR_X - 2, SONAR_X + 2, -7, 7, Z_LOWER + 4, SONAR_Z - 9))     # room for the 4 plugs
    b = span(170, 200, -32, 32, Z_LOWER, Z_LOWER + 4).union(face)
    for y0, y1 in ((-22.5, -7), (7, 22.5)):                          # ledges under the board; its pins pass between
        b = b.union(span(SONAR_X - 5.5, SONAR_X - 1.5, y0, y1, SONAR_Z - 13, SONAR_Z - 10))
    for y0 in (-27, 24):                                               # 2 ribs behind the face, beside the board
        rib = (cq.Workplane("XZ").polyline([(180, Z_LOWER + 4), (SONAR_X - 1.5, Z_LOWER + 4),
                                            (SONAR_X - 1.5, SONAR_Z + 4)]).close().extrude(-3).translate((0, y0, 0)))
        b = b.union(rib)
    return drill(b, SONAR_HOLES, 3.4, Z_LOWER - 1, 6)


add("sensors", "sonar_bracket", sonar_bracket(), PRINT, fab=True)
add("sensors", "hc_sr04_pcb", span(SONAR_X - 3.1, SONAR_X - 1.5, -22.5, 22.5, SONAR_Z - 10, SONAR_Z + 10), PCB_BLUE)
add("sensors", "hc_sr04_cans", cyl_x(16, SONAR_X - 1.5, SONAR_X + 10.5, -13, SONAR_Z)
    .union(cyl_x(16, SONAR_X - 1.5, SONAR_X + 10.5, 13, SONAR_Z)), SILVER)
add("sensors", "hc_sr04_plugs", span(SONAR_X - 3.8, SONAR_X - 0.8, -5.5, 5.5, SONAR_Z - 10 - 20, SONAR_Z - 10), BLACK)


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
MOUNT_T = 5                      # servo mount: thickness of the upright the ears rest on


def servo_body_front():
    """DS5160, shaft pointing -X (towards the bed), body along +Y from the shaft. Ears 31 mm above the case bottom,
    5 mm past each end, with 2 open notches each (17 mm apart) - all from the maker's drawing."""
    xt = X_SERVO_TOP
    y0, y1 = Y_AX - SV["shaft"], Y_AX - SV["shaft"] + SV["L"]
    zw = SV["W"] / 2
    body = span(xt, xt + SV["H"], y0, y1, Z_AX - zw, Z_AX + zw)
    xe = X_EAR - SV["ear_t"]
    ears = span(xe, X_EAR, y0 - SV["ear_ext"], y1 + SV["ear_ext"], Z_AX - zw, Z_AX + zw)
    for ye in (y0 - SV["ear_ext"], y1 + SV["ear_ext"]):
        for zh in (Z_AX - SV["ear_holes"] / 2, Z_AX + SV["ear_holes"] / 2):
            ears = ears.cut(span(xe - 1, X_EAR + 1, ye - 3, ye + 3, zh - 2.2, zh + 2.2))
    boss = cyl_x(SV["boss_d"], xt - SV["boss_h"], xt, Y_AX, Z_AX)
    return body.union(ears).union(boss)


def servo_mount_front():
    """Printed: an upright with a window the servo body passes through (its ears rest on the upright's face), and a
    foot under the body. The ears are held by 2 printed clamps, so the mount doesn't depend on where the ear notches
    are. The foot's 3 bolt slots let the mount slide +-4 mm along Y to put the servo shaft exactly on the hinge axis.
    Print it lying on the upright's face (the side the ears touch): no supports."""
    xp, xb = X_EAR, X_SERVO_TOP + SV["H"] + 2
    ym, zw = Y_SERVO_MID, SV["W"] / 2
    yh = CLAMP_DY + 7.5                                      # half-length of the upright
    z_top = Z_AX + zw + 8
    upright = span(xp, xp + MOUNT_T, ym - yh, ym + yh, Z_DECK_TOP, z_top)
    upright = upright.cut(span(xp - 1, xp + MOUNT_T + 1, ym - SV["L"] / 2 - 0.4, ym + SV["L"] / 2 + 0.4,
                               Z_AX - zw, Z_AX + zw + 0.4))   # window: the body + 0.4 mm all round
    foot = span(xp, xb, ym - 40, ym + 40, Z_DECK_TOP, Z_DECK_TOP + SERVO_FOOT)
    m = upright.union(foot)
    for ry in (ym - yh, ym + yh - 3):                        # 2 ribs behind the upright, at its ends
        rib = (cq.Workplane("XZ").polyline([(xp + MOUNT_T, Z_DECK_TOP), (xb, Z_DECK_TOP), (xp + MOUNT_T, z_top - 3)])
               .close().extrude(-3).translate((0, ry, 0)))
        m = m.union(rib)
    for sy in (-1, 1):                                       # the 2 ear-clamp screws (M4, nut behind the upright)
        m = m.cut(cyl_x(4.5, xp - 1, xp + MOUNT_T + 1, ym + sy * CLAMP_DY, Z_AX))
    zt = Z_DECK_TOP + SERVO_FOOT
    for hx, hy in servo_mount_holes(True):                  # 3 bolts up through the deck: slots + nut slots
        m = m.cut(slot_y(hx, hy, Z_DECK_TOP - 1, SERVO_FOOT + 2, 2 * MOUNT_SLIDE, 4.5))
        m = m.cut(span(hx - 3.7, hx + 3.7, hy - MOUNT_SLIDE - 4.5, hy + MOUNT_SLIDE + 4.5, zt - 3.6, zt + 1))
    return m


def servo_clamp(sy):
    """Front servo, ear at the -Y end (sy = -1) or +Y end (sy = +1): presses the ear onto the upright. Its step is
    0.5 mm lower than the ear is thick, so it always bears on the ear. The same printed part fits all 4 places."""
    ym = Y_SERVO_MID
    ye = ym + sy * (SV["L"] / 2 + SV["ear_ext"])            # end of the ear
    yi = ym + sy * (SV["L"] / 2 + 1)                         # tip: 1 mm past the body, on the ear
    yo = ym + sy * (CLAMP_DY + 6)                            # outer end
    xe = X_EAR - SV["ear_t"]
    top = span(xe - 5, xe, min(yi, yo), max(yi, yo), Z_AX - 6, Z_AX + 6)
    step = span(xe, X_EAR - 0.5, min(ye, yo), max(ye, yo), Z_AX - 6, Z_AX + 6)
    return top.union(step).cut(cyl_x(4.5, xe - 6, X_EAR + 1, ym + sy * CLAMP_DY, Z_AX))


def bed_rest(x, y):
    """Post the bed lies on when it is flat. An M4 x 10 comes up through the deck into a nut trapped in a pocket in
    the post's underside (put the nut in before you stand the post on the deck)."""
    post = cyl_z(20, 17, x, y, Z_DECK_TOP).cut(cyl_z(4.5, 12, x, y, Z_DECK_TOP))
    pocket = cq.Workplane("XY").polygon(6, 7.4 / math.cos(math.pi / 6)).extrude(3.6).translate((x, y, Z_DECK_TOP))
    return post.cut(pocket)


sb = servo_body_front()
add("tilt", "servo_front", sb, SERVO_BLUE)
add("tilt", "servo_rear", mirror_x(sb), SERVO_BLUE)
sm = servo_mount_front()
add("tilt", "servo_mount_front", sm, PRINT, fab=True)
add("tilt", "servo_mount_rear", mirror_x(sm), PRINT, fab=True)
for tag, sy in (("a", -1), ("b", 1)):
    c = servo_clamp(sy)
    add("tilt", f"servo_clamp_front_{tag}", c, PRINT)
    add("tilt", f"servo_clamp_rear_{tag}", mirror_x(c), PRINT)
fabricated["servo_clamp"] = servo_clamp(-1)
for i, (x, y) in enumerate(REST_PTS):
    add("tilt", f"bed_rest_{i}", bed_rest(x, y), PRINT)
    add("tilt", f"bed_rest_pad_{i}", cyl_z(18, 2, x, y, Z_DECK_TOP + 17), RUBBER)
fabricated["bed_rest"] = bed_rest(0, 0).translate((0, 0, -Z_DECK_TOP))


# ---------- Tilting bed: moving half (rotates about AXIS) ----------
ADAPTER_BOLTS = [(TILT_X - 10, Y_AX + 8), (TILT_X - 10, Y_AX + 21)]


def tilt_plate():
    zc, z0, h = Z_TILT + TILT_T / 2, Z_TILT - 1, TILT_T + 2
    p = span(-TILT_X, TILT_X, Y_AX, TILT_Y1, Z_TILT, Z_TILT_TOP)
    bolts = ADAPTER_BOLTS + [(-x, y) for x, y in ADAPTER_BOLTS]
    return drill(p, bolts + riser_holes(bottom=True), 4.5, z0, h)


def cargo_plate():
    zc, z0, h = Z_CARGO + CARGO_T / 2, Z_CARGO - 1, CARGO_T + 2
    p = box(CARGO[0], CARGO[1], CARGO_T, CARGO_X, 0, zc)
    return drill(p, riser_holes(bottom=False), 4.5, z0, h)       # countersink them by hand (90 deg, M4 flat head)


def riser(cx, cy, z0):
    """Printed bar between the tilting plate and the cargo plate. 2 x M4 come up from below through the
    tilting plate into its outer end, 2 countersunk M4 come down through the cargo plate into its inner end. Each
    screw meets an M4 nut slid into a slot that runs right across the bar."""
    L_, W_, H_ = RISER
    outer = -1 if cx < CARGO_X else 1
    r = box(L_, W_, H_, cx, cy, z0 + H_ / 2)
    nw, nh = M4_NUT_SLOT
    for d in RISER_HOLES:
        for x, zn in ((cx + outer * (L_ / 2 - d), z0 + 6),            # outer end: screws from below
                      (cx - outer * (L_ / 2 - d), z0 + H_ - 6)):      # inner end: screws from above
            r = r.cut(cyl_z(4.5, H_ + 2, x, cy, z0 - 1))
            r = r.cut(box(nw, W_ + 2, nh, x, cy, zn))
    return r


def horn_adapter_front():
    """Printed block on top of the tilting plate's front-right corner; the servo's disc horn screws to its round wall
    with 4 x M3 self-tapping screws in 4 radial slots, so any horn with a 19-28 mm screw circle fits."""
    x0, x1 = TILT_X, TILT_X + ADAPTER_WALL
    wall = cyl_x(36, x0, x1, Y_AX, Z_AX)                # round, so it never dips towards the deck as it turns
    flange = span(TILT_X - 20, TILT_X + 0.01, Y_AX, Y_AX + 28, Z_TILT_TOP, Z_TILT_TOP + 6)
    gusset = span(TILT_X - 8, TILT_X + 0.01, Y_AX, Y_AX + 10, Z_TILT_TOP + 6, Z_TILT_TOP + 12)
    a = wall.union(flange).union(gusset)
    r0, r1 = HORN_SLOTS
    rm = (r0 + r1) / 2
    for k in range(4):
        ang = 45 + 90 * k
        cy, cz = Y_AX + rm * math.cos(math.radians(ang)), Z_AX + rm * math.sin(math.radians(ang))
        a = a.cut(cq.Workplane("YZ").slot2D(r1 - r0 + 2.8, 2.8, ang).extrude(x1 + 1 - x0).translate((x0, cy, cz)))
    a = a.cut(cyl_x(8, x0 - 1, x1 + 1, Y_AX, Z_AX))     # clearance for the horn's centre screw
    return drill(a, ADAPTER_BOLTS, 4.5, Z_TILT_TOP - 1, 8)


def horn_front():
    x0 = TILT_X + ADAPTER_WALL
    disc = cyl_x(HORN_D, x0, x0 + HORN_T, Y_AX, Z_AX)
    hub = cyl_x(14, x0 + HORN_T, x0 + HORN_T + HORN_HUB, Y_AX, Z_AX)
    return disc.union(hub)


def bed_items():
    """Every part that tips with the bed, in the flat position."""
    items = [("tilt_plate", tilt_plate(), PLYWOOD)]
    n = 0
    for cx in RISER_X:
        for s in (-1, 1):
            items.append((f"bed_riser_{n}", riser(cx, s * RISER_Y, Z_TILT_TOP), PRINT))
            n += 1
    ha = horn_adapter_front()
    items += [("horn_adapter_front", ha, PRINT), ("horn_adapter_rear", mirror_x(ha), PRINT),
              ("servo_horn_front", horn_front(), ALU), ("servo_horn_rear", mirror_x(horn_front()), ALU)]
    items.append(("cargo_plate", cargo_plate(), ACRYLIC))
    return items


def tipped(wp, angle):
    """The workplane turned about the hinge axis by angle degrees (positive = left edge up)."""
    return wp.rotate(AXIS[0], AXIS[1], angle) if angle else wp


BED_KG = 0.9                     # everything that tips (plates, risers, adapters, horns), estimated
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
_bed = dict((n, w) for n, w, _ in BED)
for name in ("tilt_plate", "cargo_plate"):
    plates[name] = _bed[name]
for name in ("horn_adapter_front", "horn_adapter_rear"):
    fabricated[name] = _bed[name]
fabricated["bed_riser"] = riser(RISER[0] / 2, 0, 0)    # printed on its 80 x 16 face


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
    ("Drive", "JGB37-520 12 V 110 RPM gear motor SET: motor with Hall encoder (6 wires) + steel bracket + hex "
              "coupling + 65 mm wheel", 2, "buy"),
    ("Drive", "BTS7960 (IBT-2) 43 A motor driver", 2, "buy"),
    ("Drive", "Ball caster: cup (caster_cup.stl) + ring (caster_ring.stl)", 2, "3D print PETG"),
    ("Drive", f"Steel bearing ball {BALL_D:.1f} mm (1 inch), or a 25 mm glass marble", 2, "buy (get 3)"),
    ("Drive", "M8 x 50 bolt + 2 M8 nuts + 2 M8 washers (one set per caster)", 2, "buy"),
    ("Sensors", "IR obstacle-avoidance module, 3 pins (VCC, GND, OUT), one blue pot (FC-51 / HW-201 type)", 3,
     "buy (get 4)"),
    ("Sensors", "Line sensor bar (line_sensor_bar.stl)", 1, "3D print PETG"),
    ("Sensors", "HC-SR04 ultrasonic distance sensor", 1, "buy (get 2)"),
    ("Sensors", "Ultrasonic sensor bracket (sonar_bracket.stl)", 1, "3D print PETG"),
    ("Tilting bed", f"Cargo plate {CARGO[0]}x{CARGO[1]}x{CARGO_T} (cargo_plate.dxf)", 1, "laser-cut acrylic"),
    ("Tilting bed", "Bed riser (bed_riser.stl)", 4, "3D print PETG"),
    ("Tilting bed", f"Tilting plate {2 * TILT_X}x{TILT_Y1 - Y_AX}x{TILT_T} (tilt_plate.dxf)", 1, "laser-cut 6 mm plywood"),
    ("Tilting bed", "Servo DS5160, 60 kg.cm, metal gears, 180 deg, 6-8.4 V, 65 x 30 x 48 mm", 2, "buy"),
    ("Tilting bed", "Aluminium DISC servo horn for the servo's spline (25T on most DS5160), 38 mm or smaller "
                    "(not the 4-arm horn: its arms hit the deck)", 2, "buy"),
    ("Tilting bed", "Servo mount (servo_mount_front.stl + servo_mount_rear.stl)", 2, "3D print PETG"),
    ("Tilting bed", "Servo ear clamp (servo_clamp.stl)", 4, "3D print PETG"),
    ("Tilting bed", "Horn adapter (horn_adapter_front.stl + horn_adapter_rear.stl)", 2, "3D print PETG 100 %"),
    ("Tilting bed", "Bed rest post (bed_rest.stl) + 18 mm self-adhesive rubber pad 2 mm", 2, "3D print PETG"),
    ("Tilting bed", "Buck converter XL4016 8 A, set to 7.4 V for the servos", 1, "buy"),
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
    ("Hardware", "M4: x10 (bed rests, 2), x12 (servo mounts, 6), x16 (servo ear clamps, 4), x20 (horn adapters 4 + "
                 "risers from below 8), x20 COUNTERSUNK (risers through the cargo plate, 8); 32 M4 nuts; M4 washers",
     1, "buy"),
    ("Hardware", "M3: x10 (servo horns, 8), x8 countersunk (caster rings, 6), x12 + nut (line sensor bar 2, IR "
                 "modules 3, sonar bracket 2); screws for the motor brackets (holes drilled by hand)", 1, "buy"),
    ("Hardware", "Resistors 1k x1, 2k x1, 10k x5, 2.2k x1, 100 nF x1, small perfboard; VHB tape, velcro strap, zip "
                 "ties, a little grease (casters)", 1, "buy"),
]

PRINT_POSE = {                   # how each printed part lies on the printer bed (rotations about the origin)
    "servo_mount_front": [((0, 1, 0), -90)], "servo_mount_rear": [((0, 1, 0), 90)],   # on the ears' face
    "servo_clamp": [((0, 1, 0), -90)],                                                    # on its flat face
    "horn_adapter_front": [((0, 1, 0), 90)], "horn_adapter_rear": [((0, 1, 0), -90)],   # on the round wall
    "line_sensor_bar": [((1, 0, 0), 180)],                                                # on the flange's top
    "caster_cup": [((1, 0, 0), 180)],                                                     # upside down
}


def print_pose(name, wp):
    """The part turned to its printing position, centred on the bed and standing on Z = 0."""
    for axis, ang in PRINT_POSE.get(name, []):
        wp = wp.rotate((0, 0, 0), axis, ang)
    bb = cq.Compound.makeCompound(wp.vals()).BoundingBox()
    return wp.translate((-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))


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

    for old in list((out / "parts").iterdir()):
        if old.stem not in fabricated:                   # only the parts to print live in output/parts/
            old.unlink()
    for name, wp in fabricated.items():
        posed = print_pose(name, wp)
        cq.exporters.export(posed, str(out / "parts" / f"{name}.step"))
        cq.exporters.export(posed, str(out / "parts" / f"{name}.stl"), tolerance=0.05, angularTolerance=0.1)
    print("wrote", len(fabricated), "printed parts (in their printing position)")

    for name, zc in (("bottom_plate", Z_BASE + BASE_T / 2), ("top_deck", Z_DECK + DECK_T / 2),
                     ("tilt_plate", Z_TILT + TILT_T / 2), ("cargo_plate", Z_CARGO + CARGO_T / 2)):
        cq.exporters.export(plates[name].translate((0, 0, -zc)).section(), str(out / "dxf" / f"{name}.dxf"))
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
