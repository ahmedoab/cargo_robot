"""
Electrical schematics of the cargo robot (design v2) - AutoCAD DXF + PDF.

Run:   .venv\\Scripts\\python make_schematics.py
Writes ./autocad/
  CargoRobot_v2_schematics.dxf   3 A3 layout tabs (open in AutoCAD, Save As .dwg):
      E01 SYSTEM BLOCK DIAGRAM   E02 POWER DISTRIBUTION   E03 CONTROL & SENSOR WIRING
  CargoRobot_v2_schematics.pdf   the same 3 sheets (colour, A3)
  previews/E0x_*.png

The pin numbers come from robot_ws/src/cargo_bot/cargo_bot/pins.py and the wire IDs (P1..P16, S1..S30)
match the wire lists in BUILD_GUIDE.md (weeks 2 and 3).
"""
import sys
from pathlib import Path

import ezdxf
from ezdxf.enums import TextEntityAlignment as TA

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot_ws" / "src" / "cargo_bot"))
from cargo_bot.pins import PINS  # noqa: E402

OUT = Path(__file__).parent / "autocad"
NAME = "CargoRobot_v2_schematics"
FRAME = (20, 10, 410, 287)
TB = (230, 10, 410, 62)
N_SHEETS = 3
PROJECT = "AUTONOMOUS CARGO ROBOT - DESIGN v2"
SCHOOL = "ARAB ACADEMY FOR SCIENCE, TECHNOLOGY & MARITIME TRANSPORT - GRADUATION PROJECT II"
DATE = "29/09/2026"

doc = ezdxf.new("R2018", setup=True, units=ezdxf.units.MM)
# net colour (AutoCAD colour index) per layer
NETS = {
    "NET-12V": 1,        # red: +12 V main
    "NET-12V-MOTOR": 6,  # magenta: +12 V after the E-stop
    "NET-5V": 30,        # orange: 5.1 V
    "NET-3V3": 40,       # dark orange/yellow: 3.3 V logic supply
    "NET-GND": 8,        # grey: ground
    "NET-SIGNAL": 5,     # blue: GPIO signals
    "NET-USB": 3,        # green: USB
    "NET-MOTOR": 6,      # magenta: motor leads
}


def setup():
    doc.styles.add("ISO", font="isocpeur.ttf")
    for name, color in NETS.items():
        doc.layers.add(name, color=color, lineweight=35)
    for name, color, lw in (("SYMBOLS", 7, 35), ("TEXT", 7, 18), ("TITLEBLOCK", 7, 50), ("WIRE-COLOURS", 7, 35)):
        doc.layers.add(name, color=color, lineweight=lw)


def net_layer(net):
    n = net.upper()
    if n.startswith("+12V_MOTOR") or n.startswith("MOTOR_L") or n.startswith("MOTOR_R") or "M+" in n or "M-" in n:
        return "NET-12V-MOTOR"
    if n.startswith("+12V"):
        return "NET-12V"
    if n.startswith("+5V"):
        return "NET-5V"
    if n.startswith("3V3"):
        return "NET-3V3"
    if n.startswith("GND"):
        return "NET-GND"
    if n.startswith("USB"):
        return "NET-USB"
    return "NET-SIGNAL"


# ============================== drawing helpers ==============================
def text(psp, s, p, h=2.3, align=TA.LEFT, layer="TEXT", rot=0.0, color=None):
    attrs = {"layer": layer, "style": "ISO"}
    if color is not None:
        attrs["color"] = color
    t = psp.add_text(s, height=h, rotation=rot, dxfattribs=attrs)
    t.set_placement(p, align=align)
    return t


def line(psp, pts, layer="SYMBOLS", color=None):
    attrs = {"layer": layer}
    if color is not None:
        attrs["color"] = color
    psp.add_lwpolyline(pts, dxfattribs=attrs)


def rect(psp, x0, y0, x1, y1, layer="SYMBOLS"):
    psp.add_lwpolyline([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], close=True, dxfattribs={"layer": layer})


def dot(psp, p, layer):
    """Filled junction dot."""
    x, y = p
    psp.add_lwpolyline([(x - 0.6, y, 1), (x + 0.6, y, 1)], format="xyb", close=True,
                       dxfattribs={"layer": layer, "const_width": 1.2})


def arrow(psp, a, b, layer, label=None, label_pos=None, h=2.2, both=False):
    """Straight arrow a -> b (filled head at b)."""
    line(psp, [a, b], layer)
    for tip, tail in ((b, a),) + (((a, b),) if both else ()):
        dx, dy = tip[0] - tail[0], tip[1] - tail[1]
        ln = (dx * dx + dy * dy) ** 0.5
        ux, uy = dx / ln, dy / ln
        bx, by = tip[0] - 3.5 * ux, tip[1] - 3.5 * uy
        psp.add_solid([tip, (bx - 1.3 * uy, by + 1.3 * ux), (bx + 1.3 * uy, by - 1.3 * ux)], dxfattribs={"layer": layer})
    if label:
        mx, my = label_pos or ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2 + 1.5)
        text(psp, label, (mx, my), h=h, align=TA.BOTTOM_CENTER, layer=layer)


def ground(psp, p):
    """Ground symbol hanging below point p."""
    x, y = p
    line(psp, [(x, y), (x, y - 3)], "NET-GND")
    for i, w in enumerate((4.0, 2.6, 1.2)):
        line(psp, [(x - w / 2, y - 3 - i * 1.1), (x + w / 2, y - 3 - i * 1.1)], "NET-GND")


def block(psp, x0, y0, x1, y1, title, lines=(), h=2.6):
    rect(psp, x0, y0, x1, y1)
    text(psp, title, ((x0 + x1) / 2, y1 - 2.5), h=h, align=TA.TOP_CENTER)
    for i, s in enumerate(lines):
        text(psp, s, ((x0 + x1) / 2, y1 - 3.5 - h - (i + 1) * (h * 0.85 + 1.2)), h=h * 0.8, align=TA.BOTTOM_CENTER)


def resistor_v(psp, x, y_top, y_bot, ref, value, layer):
    """Vertical IEC resistor between y_top and y_bot."""
    mid = (y_top + y_bot) / 2
    line(psp, [(x, y_top), (x, mid + 5)], layer)
    rect(psp, x - 1.6, mid - 5, x + 1.6, mid + 5)
    line(psp, [(x, mid - 5), (x, y_bot)], layer)
    text(psp, f"{ref} {value}", (x + 3, mid), h=2.2, align=TA.MIDDLE_LEFT)


def fuse_h(psp, x0, x1, y, ref, value, layer):
    mid = (x0 + x1) / 2
    line(psp, [(x0, y), (mid - 6, y)], layer)
    rect(psp, mid - 6, y - 1.8, mid + 6, y + 1.8)
    line(psp, [(mid - 6, y), (mid + 6, y)], "SYMBOLS")
    line(psp, [(mid + 6, y), (x1, y)], layer)
    text(psp, ref, (mid, y + 3), h=2.3, align=TA.BOTTOM_CENTER)
    text(psp, value, (mid, y - 3), h=2.0, align=TA.TOP_CENTER)


def switch_h(psp, x0, x1, y, ref, value, layer, nc=False):
    """SPST switch (open lever) or NC push button (closed contact with actuator)."""
    mid = (x0 + x1) / 2
    line(psp, [(x0, y), (mid - 5, y)], layer)
    line(psp, [(mid + 5, y), (x1, y)], layer)
    dot(psp, (mid - 5, y), "SYMBOLS")
    dot(psp, (mid + 5, y), "SYMBOLS")
    if nc:
        line(psp, [(mid - 6, y - 1.5), (mid + 6, y - 1.5)], "SYMBOLS")            # bridging contact
        line(psp, [(mid, y - 1.5), (mid, y + 6)], "SYMBOLS")                      # plunger
        line(psp, [(mid - 4, y + 6), (mid + 4, y + 6)], "SYMBOLS")                # mushroom head
        line(psp, [(mid - 4, y + 6), (mid - 3, y + 8), (mid + 3, y + 8), (mid + 4, y + 6)], "SYMBOLS")
    else:
        line(psp, [(mid - 5, y), (mid + 4, y + 4)], "SYMBOLS")
    text(psp, ref, (mid, y + (10 if nc else 6)), h=2.3, align=TA.BOTTOM_CENTER)
    text(psp, value, (mid, y - 4), h=2.0, align=TA.TOP_CENTER)


def battery_v(psp, x, y_top, y_bot, ref, value):
    mid = (y_top + y_bot) / 2
    line(psp, [(x, y_top), (x, mid + 2)], "NET-12V")
    line(psp, [(x - 5, mid + 2), (x + 5, mid + 2)], "SYMBOLS")       # long plate = +
    line(psp, [(x - 2.5, mid - 1), (x + 2.5, mid - 1)], "SYMBOLS")   # short plate = -
    line(psp, [(x, mid - 1), (x, y_bot)], "NET-GND")
    text(psp, "+", (x - 6.5, mid + 2), h=2.5, align=TA.MIDDLE_CENTER)
    text(psp, ref, (x + 7, mid + 3), h=2.3)
    text(psp, value, (x + 7, mid - 1), h=2.0)


def motor(psp, c, ref, value):
    psp.add_circle(c, 6, dxfattribs={"layer": "SYMBOLS"})
    text(psp, "M", c, h=4, align=TA.MIDDLE_CENTER)
    text(psp, ref, (c[0] + 8, c[1] + 2), h=2.3)
    if value:
        text(psp, value, (c[0] + 8, c[1] - 2), h=1.8)


def title_block(psp, title, dwg_no, sheet_no):
    x0, y0, x1, y1 = FRAME
    rect(psp, x0, y0, x1, y1, "TITLEBLOCK")
    tx0, ty0, tx1, ty1 = TB
    rect(psp, tx0, ty0, tx1, ty1, "TITLEBLOCK")
    for y in (22, 34, 48):
        line(psp, [(tx0, y), (tx1, y)], "TITLEBLOCK")
    line(psp, [(350, 34), (350, 48)], "TITLEBLOCK")
    cells = [(230, "SCALE", "NTS"), (258, "SHEET", f"{sheet_no} / {N_SHEETS}"), (286, "UNITS", "-"),
             (310, "TYPE", "SCHEMATIC"), (346, "DATE", DATE), (376, "DRAWN", "")]
    for i, (x, label, value) in enumerate(cells):
        if i:
            line(psp, [(x, ty0), (x, 22)], "TITLEBLOCK")
        nxt = cells[i + 1][0] if i + 1 < len(cells) else tx1
        text(psp, label, (x + 1.2, 19.8), h=1.8)
        text(psp, value, ((x + nxt) / 2, 13.5), h=2.8, align=TA.MIDDLE_CENTER)
    text(psp, "PROJECT", (tx0 + 1.2, 59.5), h=1.8)
    text(psp, PROJECT, ((tx0 + tx1) / 2, 53.5), h=4.2, align=TA.MIDDLE_CENTER)
    text(psp, "TITLE", (tx0 + 1.2, 45.6), h=1.8)
    text(psp, title, ((tx0 + 350) / 2, 39.5), h=3.8, align=TA.MIDDLE_CENTER)
    text(psp, "DWG No.", (351.2, 45.6), h=1.8)
    text(psp, dwg_no, (380, 39.5), h=3.5, align=TA.MIDDLE_CENTER)
    text(psp, SCHOOL, ((tx0 + tx1) / 2, 28), h=2.0, align=TA.MIDDLE_CENTER)


def new_sheet(number, name, title):
    psp = doc.layouts.new(f"E{number:02d} {name}")
    psp.page_setup(size=(420, 297), margins=(0, 0, 0, 0), units="mm",
                   name="ISO_full_bleed_A3_(420.00_x_297.00_MM)", device="DWG To PDF.pc3")
    title_block(psp, title, f"CR-V2-E{number:02d}", number)
    return psp


def legend(psp, x, y):
    text(psp, "WIRE COLOURS IN THIS DRAWING", (x, y), h=2.4)
    items = [("NET-12V", "+12 V main (after fuse + switch)"), ("NET-12V-MOTOR", "+12 V motor (after E-stop), motor leads"),
             ("NET-5V", "+5.1 V (buck output)"), ("NET-3V3", "+3.3 V logic (from Pi pins 1 / 17)"),
             ("NET-GND", "ground"), ("NET-SIGNAL", "GPIO signals"), ("NET-USB", "USB")]
    for i, (layer, s) in enumerate(items):
        yy = y - 5 - i * 4.2
        line(psp, [(x, yy), (x + 10, yy)], layer)
        text(psp, s, (x + 13, yy), h=2.1, align=TA.MIDDLE_LEFT)


# ============================== E01 block diagram ==============================
def sheet_block_diagram():
    psp = new_sheet(1, "SYSTEM BLOCK DIAGRAM", "SYSTEM BLOCK DIAGRAM")
    B = {
        "bat": (28, 205, 78, 235, "BATTERY", ["3S LiPo 11.1 V", "5000 mAh, XT60"]),
        "prot": (100, 205, 150, 235, "F1 + SW1", ["10 A fuse", "master switch"]),
        "estop": (175, 240, 225, 268, "S1 E-STOP", ["latching, NC", "cuts motor 12 V"]),
        "buck": (175, 170, 225, 198, "U1 BUCK", ["XL4015", "12 V -> 5.1 V 5 A"]),
        "pi": (250, 140, 320, 230, "RASPBERRY PI 5", ["8 GB, Ubuntu 24.04", "ROS 2 Jazzy:", "base_driver",
                                                      "payload_node", "speed_limiter", "vision_node",
                                                      "SLAM + Nav2"]),
        "drv": (250, 240, 320, 268, "U2 + U3", ["2 x BTS7960", "motor drivers"]),
        "mot": (345, 240, 405, 268, "M1 + M2", ["JGB37-520 12 V", "+ Hall encoders"]),
        "hx": (345, 185, 405, 213, "4 x HX711", ["24-bit load-cell", "amplifiers"]),
        "cells": (345, 150, 405, 176, "4 x LOAD CELL", ["10 kg bar"]),
        "lidar": (345, 110, 405, 136, "RPLIDAR A1M8", ["360 deg 2D lidar"]),
        "cam": (345, 78, 405, 102, "USB CAMERA", ["UVC wide-angle"]),
        "laptop": (250, 78, 320, 104, "LAPTOP", ["RViz, Nav2 goals", "teleop"]),
    }
    for x0, y0, x1, y1, title, lines in B.values():
        block(psp, x0, y0, x1, y1, title, lines)
    arrow(psp, (78, 220), (100, 220), "NET-12V", "12 V")
    line(psp, [(150, 220), (162, 220), (162, 254), (175, 254)], "NET-12V")
    arrow(psp, (170, 254), (175, 254), "NET-12V")
    text(psp, "+12 V MAIN", (156, 238), h=2.2, align=TA.MIDDLE_CENTER, layer="NET-12V", rot=90)
    line(psp, [(162, 220), (162, 184), (175, 184)], "NET-12V")
    arrow(psp, (170, 184), (175, 184), "NET-12V")
    dot(psp, (162, 220), "NET-12V")
    arrow(psp, (225, 254), (250, 254), "NET-12V-MOTOR", "+12 V MOTOR")
    arrow(psp, (225, 184), (250, 184), "NET-5V", "5.1 V (USB-C)")
    arrow(psp, (320, 254), (345, 254), "NET-12V-MOTOR", "12 V PWM")
    arrow(psp, (285, 230), (285, 240), "NET-SIGNAL")
    text(psp, "PWM x4 + EN", (288, 234.5), h=2.0, align=TA.MIDDLE_LEFT, layer="NET-SIGNAL")
    line(psp, [(375, 240), (375, 234), (330, 234), (330, 225)], "NET-SIGNAL")
    arrow(psp, (330, 226), (320, 226), "NET-SIGNAL")
    text(psp, "encoders A/B x2", (352, 236), h=2.0, align=TA.BOTTOM_CENTER, layer="NET-SIGNAL")
    arrow(psp, (375, 176), (375, 185), "NET-SIGNAL")
    text(psp, "bridge mV", (377, 180.5), h=1.9, align=TA.MIDDLE_LEFT, layer="NET-SIGNAL")
    arrow(psp, (345, 199), (320, 199), "NET-SIGNAL", "DT x4 + SCK")
    arrow(psp, (345, 123), (320, 150), "NET-USB", "USB", label_pos=(337, 138))
    arrow(psp, (345, 90), (305, 140), "NET-USB", "USB", label_pos=(333, 111))
    arrow(psp, (285, 104), (285, 140), "NET-SIGNAL", both=True)
    text(psp, "Wi-Fi (ROS 2)", (288, 121), h=2.0, align=TA.MIDDLE_LEFT, layer="NET-SIGNAL")
    line(psp, [(200, 240), (200, 232), (238, 232), (238, 215), (250, 215)], "NET-SIGNAL")
    arrow(psp, (245, 215), (250, 215), "NET-SIGNAL")
    text(psp, "E-stop sense", (222, 229), h=1.9, align=TA.TOP_CENTER, layer="NET-SIGNAL")
    text(psp, "(GPIO26 via divider)", (222, 225.5), h=1.7, align=TA.TOP_CENTER, layer="NET-SIGNAL")
    legend(psp, 28, 150)
    text(psp, "Details: E02 power distribution, E03 control & sensor wiring. Pin numbers: cargo_bot/pins.py.",
         (28, 95), h=2.2)
    return psp


# ============================== E02 power distribution ==============================
def sheet_power():
    psp = new_sheet(2, "POWER DISTRIBUTION", "POWER DISTRIBUTION")
    y_main = 235
    # battery + fuse + master switch -> +12 V MAIN
    battery_v(psp, 40, y_main, 195, "BT1", "3S LiPo 11.1 V 5000 mAh (XT60)")
    ground(psp, (40, 195))
    fuse_h(psp, 40, 85, y_main, "F1", "10 A blade", "NET-12V")
    switch_h(psp, 85, 125, y_main, "SW1", "master, 15 A DC", "NET-12V")
    line(psp, [(125, y_main), (240, y_main)], "NET-12V")
    text(psp, "+12V_MAIN", (170, y_main + 1.5), h=2.5, align=TA.BOTTOM_CENTER, layer="NET-12V")
    text(psp, "P1", (44, y_main + 1.5), h=1.9, layer="NET-12V")
    text(psp, "P2", (88, y_main + 1.5), h=1.9, layer="NET-12V")
    text(psp, "P3", (135, y_main + 1.5), h=1.9, layer="NET-12V")
    # buck converter
    dot(psp, (150, y_main), "NET-12V")
    line(psp, [(150, y_main), (150, 205), (165, 205)], "NET-12V")
    text(psp, "P13", (151.5, 220), h=1.9, layer="NET-12V")
    rect(psp, 165, 180, 205, 212)
    text(psp, "U1 BUCK", (185, 209), h=2.4, align=TA.TOP_CENTER)
    text(psp, "XL4015", (185, 204.5), h=1.9, align=TA.TOP_CENTER)
    text(psp, "set 5.10 V", (185, 200.5), h=1.9, align=TA.TOP_CENTER)
    for s, y in (("IN+", 205), ("IN-", 186)):
        text(psp, s, (166, y), h=1.8, align=TA.MIDDLE_LEFT)
    for s, y in (("OUT+", 205), ("OUT-", 186)):
        text(psp, s, (204, y), h=1.8, align=TA.MIDDLE_RIGHT)
    line(psp, [(165, 186), (155, 186)], "NET-GND")
    ground(psp, (155, 186))
    text(psp, "P14", (157, 188), h=1.9, layer="NET-GND")
    line(psp, [(205, 205), (225, 205)], "NET-5V")
    line(psp, [(205, 186), (225, 186)], "NET-GND")
    text(psp, "+5V1", (212, 206.5), h=2.2, align=TA.BOTTOM_CENTER, layer="NET-5V")
    rect(psp, 225, 180, 262, 212)
    text(psp, "RASPBERRY PI 5", (243.5, 209), h=2.2, align=TA.TOP_CENTER)
    text(psp, "USB-C power in", (243.5, 204.5), h=1.8, align=TA.TOP_CENTER)
    text(psp, "(pigtail, P15)", (243.5, 200.5), h=1.8, align=TA.TOP_CENTER)
    text(psp, "5V", (226, 205), h=1.8, align=TA.MIDDLE_LEFT)
    text(psp, "GND", (226, 186), h=1.8, align=TA.MIDDLE_LEFT)
    text(psp, "GND is common with the logic GND of E03", (225, 176), h=1.7)
    # E-stop -> +12 V MOTOR
    switch_h(psp, 240, 280, y_main, "S1 E-STOP", "latching, NC, 10 A", "NET-12V", nc=True)
    line(psp, [(280, y_main), (300, y_main), (300, 118)], "NET-12V-MOTOR")
    text(psp, "+12V_MOTOR", (302, 226), h=2.5, align=TA.MIDDLE_LEFT, layer="NET-12V-MOTOR")
    text(psp, "P5 / P6", (260, y_main - 10), h=1.9, layer="NET-12V")
    # two motor drivers + motors
    for i, (y0, ref, side, mref) in enumerate(((190, "U2", "LEFT", "M1"), (130, "U3", "RIGHT", "M2"))):
        y1 = y0 + 30
        dot(psp, (300, y1 - 6), "NET-12V-MOTOR")
        line(psp, [(300, y1 - 6), (320, y1 - 6)], "NET-12V-MOTOR")
        text(psp, "P7" if i == 0 else "P8", (305, y1 - 4.5), h=1.9, layer="NET-12V-MOTOR")
        rect(psp, 320, y0, 360, y1)
        text(psp, f"{ref} BTS7960", (340, y1 - 2.5), h=2.2, align=TA.TOP_CENTER)
        text(psp, f"({side} motor)", (340, y1 - 6.5), h=1.8, align=TA.TOP_CENTER)
        text(psp, "B+", (321, y1 - 6), h=1.8, align=TA.MIDDLE_LEFT)
        text(psp, "B-", (321, y0 + 5), h=1.8, align=TA.MIDDLE_LEFT)
        text(psp, "M+", (359, y0 + 18), h=1.8, align=TA.MIDDLE_RIGHT)
        text(psp, "M-", (359, y0 + 5), h=1.8, align=TA.MIDDLE_RIGHT)
        text(psp, "logic: E03", (340, y0 + 12), h=1.7, align=TA.MIDDLE_CENTER)
        line(psp, [(320, y0 + 5), (312, y0 + 5)], "NET-GND")
        ground(psp, (312, y0 + 5))
        text(psp, "P9" if i == 0 else "P10", (313, y0 + 6.5), h=1.9, layer="NET-GND")
        mc = (388, y0 + 12)
        line(psp, [(360, mc[1] + 6), (388, mc[1] + 6)], "NET-MOTOR")                   # to the top of M
        line(psp, [(360, y0 + 5), (388, y0 + 5), (388, mc[1] - 6)], "NET-MOTOR")       # to the bottom of M
        motor(psp, mc, mref, "")
        text(psp, "red (M1)", (366, y0 + 19.5), h=1.6, layer="NET-MOTOR")
        text(psp, "white (M2)", (366, y0 + 6.5), h=1.6, layer="NET-MOTOR")
        text(psp, "P11" if i == 0 else "P12", (366, y0 + 12), h=1.9, layer="NET-MOTOR")
    text(psp, "JGB37-520 12 V", (371, 120), h=1.7)
    # E-stop sense divider
    dot(psp, (300, 118), "NET-12V-MOTOR")
    line(psp, [(300, 118), (270, 118)], "NET-12V-MOTOR")
    resistor_v(psp, 270, 118, 96, "R1", "10 k", "NET-12V-MOTOR")
    dot(psp, (270, 96), "NET-SIGNAL")
    line(psp, [(270, 96), (240, 96)], "NET-SIGNAL")
    text(psp, "ESTOP_SENSE -> Pi GPIO26 (pin 37)", (238, 96), h=2.2, align=TA.MIDDLE_RIGHT, layer="NET-SIGNAL")
    text(psp, "~3.1 V = motor power ON, 0 V = E-stop pressed", (238, 92), h=1.8, align=TA.MIDDLE_RIGHT,
         layer="NET-SIGNAL")
    resistor_v(psp, 270, 96, 76, "R2", "3.3 k", "NET-GND")
    ground(psp, (270, 76))
    text(psp, "S28", (272, 99), h=1.9, layer="NET-SIGNAL")
    # LiPo alarm + notes
    rect(psp, 28, 150, 92, 168)
    text(psp, "LiPo voltage alarm", (60, 165), h=2.1, align=TA.TOP_CENTER)
    text(psp, "on the balance lead (P16)", (60, 161), h=1.8, align=TA.TOP_CENTER)
    text(psp, "set to 3.5 V / cell", (60, 157), h=1.8, align=TA.TOP_CENTER)
    line(psp, [(52, 168), (48, 200)], "SYMBOLS", color=8)
    text(psp, "NOTES", (28, 138), h=2.8)
    for k, s in enumerate([
        "1. Wire IDs P1..P16 = power wire list, BUILD_GUIDE.md week 2.",
        "2. 16 AWG: P1-P10 (battery, fuse, switch, E-stop, drivers).",
        "3. 20 AWG: P11-P15 (motors, buck in/out, USB-C pigtail).",
        "4. Fuse F1 must be the first part after the battery +.",
        "5. The E-stop cuts motor power only; the Pi stays on",
        "   and reads the E-stop state on GPIO26 (divider R1/R2).",
        "6. Set the buck to 5.10-5.20 V BEFORE connecting the Pi.",
    ]):
        text(psp, s, (28, 133 - k * 4.3), h=2.1)
    legend(psp, 130, 150)
    return psp


# ============================== E03 control & sensor wiring ==============================
PI_PIN_NAMES = {
    1: "3V3", 2: "5V", 3: "GPIO2", 4: "5V", 5: "GPIO3", 6: "GND", 7: "GPIO4", 8: "GPIO14", 9: "GND", 10: "GPIO15",
    11: "GPIO17", 12: "GPIO18", 13: "GPIO27", 14: "GND", 15: "GPIO22", 16: "GPIO23", 17: "3V3", 18: "GPIO24",
    19: "GPIO10", 20: "GND", 21: "GPIO9", 22: "GPIO25", 23: "GPIO11", 24: "GPIO8", 25: "GND", 26: "GPIO7",
    27: "GPIO0", 28: "GPIO1", 29: "GPIO5", 30: "GND", 31: "GPIO6", 32: "GPIO12", 33: "GPIO13", 34: "GND",
    35: "GPIO19", 36: "GPIO16", 37: "GPIO26", 38: "GPIO20", 39: "GND", 40: "GPIO21",
}
BCM_TO_PIN = {int(v[4:]): k for k, v in PI_PIN_NAMES.items() if v.startswith("GPIO")}


def pi_nets():
    """Physical pin -> net name, built from pins.py."""
    nets = {1: "3V3_A", 17: "3V3_B", 6: "GND", 9: "GND"}
    named = {"enable": "MOTOR_EN", "left_fwd": "L_RPWM", "left_rev": "L_LPWM", "right_fwd": "R_RPWM",
             "right_rev": "R_LPWM", "left_enc_a": "ENC_L_A", "left_enc_b": "ENC_L_B", "right_enc_a": "ENC_R_A",
             "right_enc_b": "ENC_R_B", "hx711_sck": "HX_SCK", "estop_sense": "ESTOP_SENSE"}
    for key, net in named.items():
        nets[BCM_TO_PIN[PINS[key]]] = net
    for bcm, corner in zip(PINS["hx711_dout"], ("FL", "FR", "RL", "RR")):
        nets[BCM_TO_PIN[bcm]] = f"HX_{corner}_DT"
    return nets


def net_label(psp, p, net, align, h=2.3):
    text(psp, net, p, h=h, align=align, layer=net_layer(net))


def connector(psp, x0, y_top, width, title, pins, pitch=4.2, side="right", stub=10):
    """Box with pins on one side; pins = [(pin name, net, wire note)]. Returns nothing."""
    h = pitch * (len(pins) + 1) + 4
    y0 = y_top - h
    rect(psp, x0, y0, x0 + width, y_top)
    text(psp, title, (x0 + width / 2, y_top + 1.2), h=2.3, align=TA.BOTTOM_CENTER)
    for i, (name, net, note) in enumerate(pins):
        y = y_top - 4 - (i + 1) * pitch + pitch / 2
        if side == "right":
            text(psp, name, (x0 + width - 1, y), h=1.8, align=TA.MIDDLE_RIGHT)
            if note:
                text(psp, note, (x0 + 1, y), h=1.6, align=TA.MIDDLE_LEFT, layer="WIRE-COLOURS")
            if net:
                line(psp, [(x0 + width, y), (x0 + width + stub, y)], net_layer(net))
                net_label(psp, (x0 + width + stub + 1, y), net, TA.MIDDLE_LEFT)
        else:
            text(psp, name, (x0 + 1, y), h=1.8, align=TA.MIDDLE_LEFT)
            if note:
                text(psp, note, (x0 + width - 1, y), h=1.6, align=TA.MIDDLE_RIGHT, layer="WIRE-COLOURS")
            if net:
                line(psp, [(x0, y), (x0 - stub, y)], net_layer(net))
                net_label(psp, (x0 - stub - 1, y), net, TA.MIDDLE_RIGHT)


def sheet_control():
    psp = new_sheet(3, "CONTROL & SENSOR WIRING", "CONTROL & SENSOR WIRING")
    # ---- Raspberry Pi 40-pin header in the middle ----
    nets = pi_nets()
    x_odd, x_even, top, pitch = 191, 199, 252, 5.4
    rect(psp, 186, top - 19 * pitch - 3.5, 204, top + 3.5)
    text(psp, "RASPBERRY PI 5 - 40-PIN HEADER", (195, top + 9), h=2.5, align=TA.BOTTOM_CENTER)
    text(psp, "(via GPIO screw-terminal HAT)", (195, top + 5), h=1.7, align=TA.BOTTOM_CENTER)
    for pin in range(1, 41):
        row = (pin - 1) // 2
        y = top - row * pitch
        x = x_odd if pin % 2 else x_even
        rect(psp, x - 2, y - 2, x + 2, y + 2)
        text(psp, str(pin), (x, y), h=1.5, align=TA.MIDDLE_CENTER)
        name = PI_PIN_NAMES[pin]
        net = nets.get(pin)
        if pin % 2:          # left column
            if net:
                line(psp, [(186, y), (150, y)], net_layer(net))
                text(psp, name, (184, y + 0.5), h=1.8, align=TA.BOTTOM_RIGHT, color=8)
                net_label(psp, (148, y), net, TA.MIDDLE_RIGHT)
            else:
                text(psp, name, (184, y), h=1.8, align=TA.MIDDLE_RIGHT, color=8)
        else:                # right column
            if net:
                line(psp, [(204, y), (238, y)], net_layer(net))
                text(psp, name, (206, y + 0.5), h=1.8, align=TA.BOTTOM_LEFT, color=8)
                net_label(psp, (240, y), net, TA.MIDDLE_LEFT)
            else:
                text(psp, name, (206, y), h=1.8, align=TA.MIDDLE_LEFT, color=8)
    text(psp, "Do NOT use pins 2 / 4 (5 V) for anything: every", (153, 141), h=1.8)
    text(psp, "signal here is 3.3 V. Unused GND pins may be used.", (153, 138), h=1.8)

    # ---- motor encoder cables (left) ----
    for side, top_y, drv in (("LEFT", 262, "BTS_L"), ("RIGHT", 222, "BTS_R")):
        s = side[0]
        connector(psp, 28, top_y, 38, f"M{1 if s == 'L' else 2} {side} MOTOR CABLE", [
            ("M1", f"MOTOR_{s}+ ({drv} M+, E02)", "red"),
            ("M2", f"MOTOR_{s}- ({drv} M-, E02)", "white"),
            ("VCC", "3V3_A", "blue"),
            ("GND", "GND", "black"),
            ("A", f"ENC_{s}_A", "yellow"),
            ("B", f"ENC_{s}_B", "green"),
        ], stub=6)
    text(psp, "Encoder VCC = 3.3 V ONLY (5 V would", (28, 182), h=1.8)
    text(psp, "put 5 V on the Pi's GPIO pins).", (28, 179), h=1.8)

    # ---- BTS7960 logic headers (right) ----
    for side, top_y, ref in (("LEFT", 262, "U2"), ("RIGHT", 212, "U3")):
        s = side[0]
        connector(psp, 330, top_y, 36, f"{ref} BTS7960 ({side} motor)", [
            ("RPWM", f"{s}_RPWM", ""),
            ("LPWM", f"{s}_LPWM", ""),
            ("R_EN", "MOTOR_EN", ""),
            ("L_EN", "MOTOR_EN", ""),
            ("R_IS", "", "not connected"),
            ("L_IS", "", "not connected"),
            ("VCC", "3V3_B", ""),
            ("GND", "GND", ""),
        ], side="left", stub=8)
    text(psp, "BTS7960 VCC from 3.3 V makes its", (330, 166), h=1.8)
    text(psp, "inputs 3.3 V-logic compatible.", (330, 163), h=1.8)
    text(psp, "Power side (B+/B-/M+/M-): see E02.", (330, 160), h=1.8)

    # ---- HX711 boards + load cells (bottom left) ----
    for i, corner in enumerate(("FL", "FR", "RL", "RR")):
        x0 = 28 + i * 52
        rect(psp, x0, 94, x0 + 42, 110)
        text(psp, f"HX711 {corner}", (x0 + 21, 102), h=2.3, align=TA.MIDDLE_CENTER)
        for k, (pin, net) in enumerate((("VCC", "3V3_B"), ("GND", "GND"), ("DT", f"HX_{corner}_DT"), ("SCK", "HX_SCK"))):
            x = x0 + 6 + k * 10
            text(psp, pin, (x, 108.5), h=1.5, align=TA.TOP_CENTER)
            line(psp, [(x, 110), (x, 115)], net_layer(net))
            text(psp, net, (x + 0.8, 116), h=1.9, align=TA.MIDDLE_LEFT, layer=net_layer(net), rot=90)
        for k, (pin, color, cname) in enumerate((("E+", 1, "red"), ("E-", 7, "blk"), ("A-", 8, "wht"), ("A+", 3, "grn"))):
            x = x0 + 6 + k * 10
            text(psp, pin, (x, 95.5), h=1.5, align=TA.BOTTOM_CENTER)
            line(psp, [(x, 94), (x, 86)], "WIRE-COLOURS", color=color)
            text(psp, cname, (x + 0.8, 90), h=1.4, align=TA.MIDDLE_LEFT, layer="WIRE-COLOURS")
        rect(psp, x0, 78, x0 + 42, 86)
        text(psp, f"LOAD CELL {corner} 10 kg", (x0 + 21, 82), h=1.9, align=TA.MIDDLE_CENTER)
    text(psp, "FL/FR/RL/RR = front-left / front-right / rear-left / rear-right, looking down, front away from you.",
         (28, 73), h=1.8)
    text(psp, "A cell that reads negative: swap its A+/A- (green/white) or set its sign to -1 in robot.yaml.",
         (28, 69.5), h=1.8)

    # ---- USB devices + E-stop sense (bottom right) ----
    rect(psp, 262, 96, 292, 136)
    text(psp, "RASPBERRY PI 5", (277, 133), h=2.0, align=TA.TOP_CENTER)
    text(psp, "USB ports", (277, 129.5), h=1.7, align=TA.TOP_CENTER)
    for y, port, dev in ((122, "USB 2.0", "RPLIDAR A1M8 + USB adapter (CP2102) -> /dev/rplidar"),
                         (112, "USB 3.0", "USB camera, UVC -> /dev/video0"),
                         (102, "USB-C", "power in: buck 5.1 V (E02)")):
        text(psp, port, (291, y), h=1.8, align=TA.MIDDLE_RIGHT)
        line(psp, [(292, y), (305, y)], "NET-USB" if port != "USB-C" else "NET-5V")
        text(psp, dev, (307, y), h=1.9, align=TA.MIDDLE_LEFT, layer="NET-USB" if port != "USB-C" else "NET-5V")
    text(psp, "ESTOP_SENSE comes from the R1/R2 divider on E02 (junction below R1).", (262, 90), h=1.9,
         layer="NET-SIGNAL")
    text(psp, "Same net name = same wire. Wire IDs S1..S30: BUILD_GUIDE.md week 3.", (262, 85.5), h=1.9)
    return psp


# ============================== main ==============================
def main():
    OUT.mkdir(exist_ok=True)
    (OUT / "previews").mkdir(exist_ok=True)
    setup()
    sheet_block_diagram()
    sheet_power()
    sheet_control()
    doc.layouts.delete("Layout1")
    doc.layouts.set_active_layout("E01 SYSTEM BLOCK DIAGRAM")
    doc.header["$TILEMODE"] = 0
    path = OUT / f"{NAME}.dxf"
    doc.saveas(path)
    print("wrote", path, "| audit errors:", len(doc.audit().errors))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from ezdxf.addons.drawing import Frontend, RenderContext
    from ezdxf.addons.drawing.config import BackgroundPolicy, ColorPolicy, Configuration
    from ezdxf.addons.drawing.matplotlib import MatplotlibBackend
    from matplotlib.backends.backend_pdf import PdfPages

    cfg = Configuration(color_policy=ColorPolicy.COLOR, background_policy=BackgroundPolicy.WHITE)
    with PdfPages(OUT / f"{NAME}.pdf") as pdf:
        for layout in doc.layouts:
            if not layout.is_any_paperspace:
                continue
            fig = plt.figure(figsize=(420 / 25.4, 297 / 25.4))
            ax = fig.add_axes([0, 0, 1, 1])
            ctx = RenderContext(doc)
            ctx.set_current_layout(layout)
            Frontend(ctx, MatplotlibBackend(ax), config=cfg).draw_layout(layout, finalize=True)
            ax.set_xlim(0, 420)
            ax.set_ylim(0, 297)
            ax.set_aspect("equal")
            ax.axis("off")
            pdf.savefig(fig)
            png = OUT / "previews" / (layout.name.replace(" ", "_").replace("&", "and") + ".png")
            fig.savefig(png, dpi=130)
            plt.close(fig)
            print("sheet", layout.name)
    print("wrote", OUT / f"{NAME}.pdf")


if __name__ == "__main__":
    main()
