"""
Electrical schematics of the cargo robot (design v2, ESP32, tilting bed) - AutoCAD DXF + PDF.

Run:   .venv\\Scripts\\python make_schematics.py
Writes ./autocad/
  CargoRobot_v2_schematics.dxf   4 A3 layout tabs (open in AutoCAD, Save As .dwg):
      E01 SYSTEM BLOCK DIAGRAM   E02 POWER DISTRIBUTION   E03 ESP32 WIRING   E04 I2C, LINE SENSOR AND PANEL
  CargoRobot_v2_schematics.pdf   the same 4 sheets (colour, A3)
  previews/E0x_*.png

The pin numbers come from firmware/cargo_robot/config.h (through wiring.py) and the wire IDs (P1..P19, S1..S38) match the
wire lists in BUILD_GUIDE.md (weeks 2 and 3).
"""
from pathlib import Path

import ezdxf
from ezdxf.enums import TextEntityAlignment as TA

from wiring import P

OUT = Path(__file__).parent / "autocad"
NAME = "CargoRobot_v2_schematics"
FRAME = (20, 10, 410, 287)
TB = (230, 10, 410, 62)
N_SHEETS = 4
PROJECT = "AUTONOMOUS CARGO ROBOT - DESIGN v2"
SCHOOL = "ARAB ACADEMY FOR SCIENCE, TECHNOLOGY & MARITIME TRANSPORT - GRADUATION PROJECT II"
DATE = "22/10/2026"

doc = ezdxf.new("R2018", setup=True, units=ezdxf.units.MM)
# net colour (AutoCAD colour index) per layer
NETS = {
    "NET-12V": 1,        # red: +12 V main
    "NET-12V-MOTOR": 6,  # magenta: +12 V after the E-stop
    "NET-7V4": 150,      # blue-green: servo supply
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
    if n.startswith("+7V4"):
        return "NET-7V4"
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
             ("NET-7V4", "+7.4 V servo supply"), ("NET-5V", "+5.1 V (buck output)"),
             ("NET-3V3", "+3.3 V logic (ESP32 3V3 pin)"), ("NET-GND", "ground"), ("NET-SIGNAL", "signals"),
             ("NET-USB", "USB")]
    for i, (layer, s) in enumerate(items):
        yy = y - 5 - i * 4.2
        line(psp, [(x, yy), (x + 10, yy)], layer)
        text(psp, s, (x + 13, yy), h=2.1, align=TA.MIDDLE_LEFT)


def net_label(psp, p, net, align, h=2.3):
    text(psp, net, p, h=h, align=align, layer=net_layer(net))


def connector(psp, x0, y_top, width, title, pins, pitch=4.2, side="right", stub=10):
    """Box with pins on one side; pins = [(pin name, net, wire note)]."""
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
    return y0


# ============================== E01 block diagram ==============================
def sheet_block_diagram():
    psp = new_sheet(1, "SYSTEM BLOCK DIAGRAM", "SYSTEM BLOCK DIAGRAM")
    B = {
        "bat": (28, 245, 78, 272, "BATTERY", ["3S LiPo 11.1 V", "5000 mAh, XT60"]),
        "prot": (95, 245, 145, 272, "F1 + SW1", ["10 A fuse", "master switch"]),
        "estop": (165, 245, 215, 272, "S1 E-STOP", ["latching, NC", "cuts motor 12 V"]),
        "drv": (240, 245, 300, 272, "U2 + U3", ["2 x BTS7960", "motor drivers"]),
        "mot": (330, 245, 400, 272, "M1 + M2", ["JGB37-520 12 V", "+ Hall encoders"]),
        "buck": (165, 200, 215, 225, "U1 BUCK 5.1 V", ["XL4015", "ESP32 + sensors"]),
        "sbuck": (165, 160, 215, 185, "U5 BUCK 7.4 V", ["10 A", "bed servos"]),
        "esp": (240, 95, 300, 225, "ESP32", ["DevKitC 38-pin", "", "line following", "speed by weight",
                                              "obstacles", "tilting bed", "mission + safety"]),
        "hx": (330, 205, 400, 230, "4 x HX711", ["+ 4 x 10 kg", "load cells"]),
        "servo": (330, 165, 400, 190, "SV1 + SV2", ["60 kg.cm servos", "tilt the bed"]),
        "sonar": (330, 125, 400, 150, "HC-SR04", ["ultrasonic, front", "ECHO via 1k/2k"]),
        "mcp": (165, 95, 215, 135, "U4 MCP23017", ["I2C 0x20", "I/O expander"]),
        "tof": (95, 140, 145, 162, "VL53L0X", ["side distance", "I2C 0x29"]),
        "line": (95, 105, 145, 127, "LINE SENSOR", ["8 x TCRT5000"]),
        "ui": (95, 70, 145, 92, "PANEL", ["button, 2 LEDs", "buzzer"]),
        "laptop": (28, 70, 78, 92, "LAPTOP", ["Arduino IDE", "(USB, programming)"]),
    }
    for x0, y0, x1, y1, title, lines in B.values():
        block(psp, x0, y0, x1, y1, title, lines)
    arrow(psp, (78, 258), (95, 258), "NET-12V", "12 V")
    arrow(psp, (145, 258), (165, 258), "NET-12V")
    line(psp, [(155, 258), (155, 212), (165, 212)], "NET-12V")
    arrow(psp, (160, 212), (165, 212), "NET-12V")
    dot(psp, (155, 258), "NET-12V")
    text(psp, "+12 V MAIN", (152, 235), h=2.0, align=TA.MIDDLE_CENTER, layer="NET-12V", rot=90)
    arrow(psp, (215, 258), (240, 258), "NET-12V-MOTOR", "+12 V MOTOR")
    line(psp, [(228, 258), (228, 172), (215, 172)], "NET-12V-MOTOR")
    arrow(psp, (220, 172), (215, 172), "NET-12V-MOTOR")
    dot(psp, (228, 258), "NET-12V-MOTOR")
    arrow(psp, (300, 258), (330, 258), "NET-12V-MOTOR", "12 V PWM")
    arrow(psp, (215, 212), (240, 212), "NET-5V", "5.1 V")
    line(psp, [(215, 166), (320, 166), (320, 172), (330, 172)], "NET-7V4")
    arrow(psp, (325, 172), (330, 172), "NET-7V4")
    text(psp, "+7.4 V", (318, 163), h=1.9, align=TA.TOP_RIGHT, layer="NET-7V4")
    arrow(psp, (270, 225), (270, 245), "NET-SIGNAL")
    text(psp, "PWM x4", (273, 235), h=2.0, align=TA.MIDDLE_LEFT, layer="NET-SIGNAL")
    line(psp, [(365, 245), (365, 238), (310, 238), (310, 222), (300, 222)], "NET-SIGNAL")
    arrow(psp, (305, 222), (300, 222), "NET-SIGNAL")
    text(psp, "encoders A/B x2", (338, 239.5), h=2.0, align=TA.BOTTOM_CENTER, layer="NET-SIGNAL")
    arrow(psp, (330, 214), (300, 214), "NET-SIGNAL", "DT x4 + SCK", label_pos=(315, 208))
    arrow(psp, (300, 180), (330, 180), "NET-SIGNAL", "2 x signal")
    arrow(psp, (330, 137), (300, 137), "NET-SIGNAL", "TRIG + ECHO", both=True, label_pos=(315, 139))
    arrow(psp, (215, 115), (240, 115), "NET-SIGNAL", "I2C", both=True)
    line(psp, [(228, 115), (228, 151), (145, 151)], "NET-SIGNAL")
    dot(psp, (228, 115), "NET-SIGNAL")
    arrow(psp, (145, 116), (165, 116), "NET-SIGNAL", "8 x OUT")
    arrow(psp, (165, 102), (145, 81), "NET-SIGNAL", both=True)
    text(psp, "button, LEDs, buzzer", (157, 88), h=1.8, align=TA.MIDDLE_LEFT, layer="NET-SIGNAL")
    arrow(psp, (215, 128), (240, 245), "NET-SIGNAL")
    text(psp, "MOTOR_EN", (226, 190), h=1.8, align=TA.MIDDLE_LEFT, layer="NET-SIGNAL", rot=73)
    line(psp, [(190, 245), (190, 238), (248, 238), (248, 225)], "NET-SIGNAL")
    arrow(psp, (248, 230), (248, 225), "NET-SIGNAL")
    text(psp, "battery V / E-stop sense", (200, 240), h=1.8, align=TA.BOTTOM_LEFT, layer="NET-SIGNAL")
    arrow(psp, (78, 81), (240, 98), "NET-USB", "USB", label_pos=(110, 97))
    legend(psp, 28, 228)
    text(psp, "Details: E02 power, E03 ESP32 wiring, E04 I2C, line sensor and panel.", (28, 180), h=2.1)
    text(psp, "Pin numbers: firmware/cargo_robot/config.h.", (28, 176), h=2.1)
    return psp


# ============================== E02 power distribution ==============================
def sheet_power():
    psp = new_sheet(2, "POWER DISTRIBUTION", "POWER DISTRIBUTION")
    y_main = 235
    battery_v(psp, 40, y_main, 195, "BT1", "3S LiPo 11.1 V 5000 mAh (XT60)")
    ground(psp, (40, 195))
    fuse_h(psp, 40, 85, y_main, "F1", "10 A blade", "NET-12V")
    switch_h(psp, 85, 125, y_main, "SW1", "master, 15 A DC", "NET-12V")
    line(psp, [(125, y_main), (240, y_main)], "NET-12V")
    text(psp, "+12V_MAIN", (170, y_main + 1.5), h=2.5, align=TA.BOTTOM_CENTER, layer="NET-12V")
    text(psp, "P1", (44, y_main + 1.5), h=1.9, layer="NET-12V")
    text(psp, "P2", (88, y_main + 1.5), h=1.9, layer="NET-12V")
    text(psp, "P3", (135, y_main + 1.5), h=1.9, layer="NET-12V")
    # 5.1 V buck converter -> 5 V bus
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
    text(psp, "+5V", (212, 206.5), h=2.2, align=TA.BOTTOM_CENTER, layer="NET-5V")
    text(psp, "P15", (210, 200), h=1.9, layer="NET-5V")
    rect(psp, 225, 176, 280, 212)
    text(psp, "5 V BUS (WAGO)", (252.5, 209), h=2.2, align=TA.TOP_CENTER)
    for k, s in enumerate(("ESP32 5V / VIN pin", "HC-SR04 VCC", "(3.3 V for logic comes", " from the ESP32 3V3 pin)")):
        text(psp, s, (252.5, 204 - k * 4), h=1.8, align=TA.TOP_CENTER)
    text(psp, "5V", (226, 205), h=1.8, align=TA.MIDDLE_LEFT)
    text(psp, "GND", (226, 186), h=1.8, align=TA.MIDDLE_LEFT)
    # E-stop -> +12 V MOTOR
    switch_h(psp, 240, 280, y_main, "S1 E-STOP", "latching, NC, 10 A", "NET-12V", nc=True)
    line(psp, [(280, y_main), (300, y_main), (300, 100)], "NET-12V-MOTOR")
    text(psp, "+12V_MOTOR", (302, 226), h=2.5, align=TA.MIDDLE_LEFT, layer="NET-12V-MOTOR")
    text(psp, "P5 / P6", (260, y_main - 10), h=1.9, layer="NET-12V")
    # two motor drivers + motors
    for i, (y0, ref, side, mref) in enumerate(((190, "U2", "LEFT", "M1"), (140, "U3", "RIGHT", "M2"))):
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
        line(psp, [(360, mc[1] + 6), (388, mc[1] + 6)], "NET-MOTOR")
        line(psp, [(360, y0 + 5), (388, y0 + 5), (388, mc[1] - 6)], "NET-MOTOR")
        motor(psp, mc, mref, "")
        text(psp, "red (M1)", (366, y0 + 19.5), h=1.6, layer="NET-MOTOR")
        text(psp, "white (M2)", (366, y0 + 6.5), h=1.6, layer="NET-MOTOR")
        text(psp, "P11" if i == 0 else "P12", (366, y0 + 12), h=1.9, layer="NET-MOTOR")
    text(psp, "JGB37-520 12 V", (371, 135), h=1.7)
    # servo buck + 2 servos
    dot(psp, (300, 112), "NET-12V-MOTOR")
    line(psp, [(300, 112), (320, 112)], "NET-12V-MOTOR")
    text(psp, "P17", (305, 113.5), h=1.9, layer="NET-12V-MOTOR")
    rect(psp, 320, 92, 360, 124)
    text(psp, "U5 BUCK 10 A", (340, 121), h=2.2, align=TA.TOP_CENTER)
    text(psp, "set 7.4 V", (340, 116.5), h=1.9, align=TA.TOP_CENTER)
    text(psp, "IN+", (321, 112), h=1.8, align=TA.MIDDLE_LEFT)
    text(psp, "IN-", (321, 97), h=1.8, align=TA.MIDDLE_LEFT)
    line(psp, [(320, 97), (312, 97)], "NET-GND")
    ground(psp, (312, 97))
    text(psp, "P18", (309, 99), h=1.9, layer="NET-GND")
    line(psp, [(360, 112), (375, 112)], "NET-7V4")
    text(psp, "+7V4 (P19)", (362, 113.5), h=1.8, layer="NET-7V4")
    for k, (yy, ref, pin) in enumerate(((112, "SV1 FRONT", "GPIO 2"), (96, "SV2 REAR", "GPIO 12"))):
        if k:
            line(psp, [(368, 112), (368, yy), (375, yy)], "NET-7V4")
            dot(psp, (368, 112), "NET-7V4")
        rect(psp, 375, yy - 6, 405, yy + 3)
        text(psp, ref, (390, yy + 1), h=1.9, align=TA.TOP_CENTER)
        text(psp, f"signal: {pin}", (390, yy - 3), h=1.6, align=TA.TOP_CENTER)
    text(psp, "servo wires: red = +7.4 V, brown = GND, orange = signal", (320, 86), h=1.7)
    # motor-bus sense divider -> ESP32 GPIO 33
    dot(psp, (300, 100), "NET-12V-MOTOR")
    line(psp, [(300, 100), (270, 100)], "NET-12V-MOTOR")
    resistor_v(psp, 270, 100, 80, "R1", "10 k", "NET-12V-MOTOR")
    dot(psp, (270, 80), "NET-SIGNAL")
    line(psp, [(270, 80), (240, 80)], "NET-SIGNAL")
    text(psp, f"BUS_SENSE -> ESP32 GPIO {P['PIN_BUS_SENSE']}", (238, 80), h=2.2, align=TA.MIDDLE_RIGHT,
         layer="NET-SIGNAL")
    text(psp, "~2.3 V = 12.6 V battery, 0 V = E-stop pressed", (238, 76), h=1.8, align=TA.MIDDLE_RIGHT,
         layer="NET-SIGNAL")
    resistor_v(psp, 270, 80, 66, "R2", "2.2 k", "NET-GND")
    ground(psp, (270, 66))
    text(psp, "S38", (272, 83), h=1.9, layer="NET-SIGNAL")
    # LiPo alarm + notes
    rect(psp, 28, 150, 92, 168)
    text(psp, "LiPo voltage alarm", (60, 165), h=2.1, align=TA.TOP_CENTER)
    text(psp, "on the balance lead (P16)", (60, 161), h=1.8, align=TA.TOP_CENTER)
    text(psp, "set to 3.5 V / cell", (60, 157), h=1.8, align=TA.TOP_CENTER)
    line(psp, [(52, 168), (48, 200)], "SYMBOLS", color=8)
    text(psp, "NOTES", (28, 140), h=2.8)
    for k, s in enumerate([
        "1. Wire IDs P1..P19 = power wire list, BUILD_GUIDE.md week 2.",
        "2. 16 AWG: P1-P10, P17-P19 (battery, fuse, switch, E-stop, drivers, servos).",
        "3. 20 AWG: P11-P15 (motors, 5 V buck in/out).",
        "4. Fuse F1 must be the first part after the battery +.",
        "5. The E-stop cuts the drive motors AND the servos; the ESP32",
        f"   stays on and reads the bus voltage on GPIO {P['PIN_BUS_SENSE']} (R1/R2).",
        "6. Set both bucks (5.10 V and 7.4 V) BEFORE connecting anything.",
    ]):
        text(psp, s, (28, 135 - k * 4.3), h=2.1)
    legend(psp, 130, 160)
    return psp


# ============================== E03 ESP32 wiring ==============================
DEVKIT_LEFT = ["3V3", "EN", "36", "39", "34", "35", "32", "33", "25", "26", "27", "14", "12", "GND", "13", "9", "10",
               "11", "5V"]
DEVKIT_RIGHT = ["GND", "23", "22", "1", "3", "21", "GND", "19", "18", "5", "17", "16", "4", "0", "2", "15", "8", "7",
                "6"]
PIN_LABEL = {"36": "GPIO36 (VP)", "39": "GPIO39 (VN)", "1": "TX0", "3": "RX0", "9": "SD2", "10": "SD3",
             "11": "CMD", "8": "SD1", "7": "SD0", "6": "CLK", "EN": "EN (reset)"}


def esp_nets():
    """GPIO number -> net name, from config.h."""
    names = {"PIN_L_RPWM": "L_RPWM", "PIN_L_LPWM": "L_LPWM", "PIN_R_RPWM": "R_RPWM", "PIN_R_LPWM": "R_LPWM",
             "PIN_ENC_L_A": "ENC_L_A", "PIN_ENC_L_B": "ENC_L_B", "PIN_ENC_R_A": "ENC_R_A", "PIN_ENC_R_B": "ENC_R_B",
             "PIN_HX_SCK": "HX_SCK", "PIN_SONAR_TRIG": "SONAR_TRIG", "PIN_SERVO_FRONT": "SERVO_F_SIG",
             "PIN_SERVO_REAR": "SERVO_R_SIG", "PIN_SDA": "SDA", "PIN_SCL": "SCL", "PIN_BUS_SENSE": "BUS_SENSE"}
    nets = {"3V3": "3V3", "GND": "GND", "5V": "+5V"}
    for key, net in names.items():
        nets[str(P[key])] = net
    for gpio, corner in zip(P["PIN_HX_DT"], ("FL", "FR", "RL", "RR")):
        nets[str(gpio)] = f"HX_{corner}_DT"
    nets[str(P["PIN_SONAR_ECHO"])] = "SONAR_ECHO"
    return nets


def sheet_esp32():
    psp = new_sheet(3, "ESP32 WIRING", "ESP32 WIRING")
    nets = esp_nets()
    top, pitch, xl, xr = 262, 5.4, 186, 210
    rect(psp, xl, top - 18 * pitch - 4, xr, top + 4)
    text(psp, "ESP32 DevKitC 38-pin", (198, top + 9), h=2.5, align=TA.BOTTOM_CENTER)
    text(psp, "(on its screw-terminal breakout; USB end at the bottom)", (198, top + 5.5), h=1.6,
         align=TA.BOTTOM_CENTER)
    for col, pins in ((0, DEVKIT_LEFT), (1, DEVKIT_RIGHT)):
        for row, pin in enumerate(pins):
            y = top - row * pitch
            label = PIN_LABEL.get(pin, pin if not pin.isdigit() else f"GPIO{pin}")
            net = nets.get(pin)
            if pin in ("9", "10", "11", "8", "7", "6"):
                net = None                           # flash pins: never use
            if col == 0:
                text(psp, label, (xl + 1, y), h=1.6, align=TA.MIDDLE_LEFT, color=8)
                if net:
                    line(psp, [(xl, y), (150, y)], net_layer(net))
                    net_label(psp, (148, y), net, TA.MIDDLE_RIGHT)
            else:
                text(psp, label, (xr - 1, y), h=1.6, align=TA.MIDDLE_RIGHT, color=8)
                if net:
                    line(psp, [(xr, y), (246, y)], net_layer(net))
                    net_label(psp, (248, y), net, TA.MIDDLE_LEFT)
    text(psp, "SD0-SD3, CMD, CLK: flash - never use.  GPIO 0 and 15: free.", (150, 154), h=1.8)
    text(psp, "GPIO 12 must be LOW at start-up: only the rear servo signal on it.", (150, 150.5), h=1.8)
    # motor + encoder cables (left)
    for side, top_y, drv in (("LEFT", 262, "U2"), ("RIGHT", 222, "U3")):
        s = side[0]
        connector(psp, 28, top_y, 38, f"M{1 if s == 'L' else 2} {side} MOTOR CABLE", [
            ("M1", f"MOTOR_{s}+ ({drv} M+, E02)", "red"),
            ("M2", f"MOTOR_{s}- ({drv} M-, E02)", "white"),
            ("VCC", "3V3", "blue"),
            ("GND", "GND", "black"),
            ("A", f"ENC_{s}_A", "yellow"),
            ("B", f"ENC_{s}_B", "green"),
        ], stub=6)
    text(psp, "Encoder VCC = 3.3 V ONLY. GPIO 34/35/36/39", (28, 182), h=1.8)
    text(psp, "have no pull-ups: 10k from each ENC line to 3V3.", (28, 178.5), h=1.8)
    # BTS7960 logic headers (right)
    for side, top_y, ref in (("LEFT", 262, "U2"), ("RIGHT", 212, "U3")):
        s = side[0]
        connector(psp, 340, top_y, 36, f"{ref} BTS7960 ({side})", [
            ("RPWM", f"{s}_RPWM", ""),
            ("LPWM", f"{s}_LPWM", ""),
            ("R_EN", "MOTOR_EN", ""),
            ("L_EN", "MOTOR_EN", ""),
            ("R_IS", "", "not connected"),
            ("L_IS", "", "not connected"),
            ("VCC", "3V3", ""),
            ("GND", "GND", ""),
        ], side="left", stub=8)
    text(psp, "MOTOR_EN comes from the MCP23017 (E04)", (300, 166), h=1.8)
    text(psp, "with a 10k pull-down: motors off at power-up.", (300, 162.5), h=1.8)
    # servo signals
    connector(psp, 340, 152, 36, "BED SERVOS (signal)", [
        ("SV1 FRONT", "SERVO_F_SIG", "orange"),
        ("SV2 REAR", "SERVO_R_SIG", "orange"),
    ], side="left", stub=8)
    text(psp, "red/brown: +7V4 bus (E02)", (340, 132), h=1.7)
    # HX711 boards + load cells (bottom left)
    for i, corner in enumerate(("FL", "FR", "RL", "RR")):
        x0 = 28 + i * 47
        rect(psp, x0, 100, x0 + 40, 116)
        text(psp, f"HX711 {corner}", (x0 + 20, 108), h=2.2, align=TA.MIDDLE_CENTER)
        for k, (pin, net) in enumerate((("VCC", "3V3"), ("GND", "GND"), ("DT", f"HX_{corner}_DT"), ("SCK", "HX_SCK"))):
            x = x0 + 5 + k * 10
            text(psp, pin, (x, 114.5), h=1.4, align=TA.TOP_CENTER)
            line(psp, [(x, 116), (x, 121)], net_layer(net))
            text(psp, net, (x + 0.8, 122), h=1.8, align=TA.MIDDLE_LEFT, layer=net_layer(net), rot=90)
        for k, (pin, color, cname) in enumerate((("E+", 1, "red"), ("E-", 7, "blk"), ("A-", 8, "wht"), ("A+", 3, "grn"))):
            x = x0 + 5 + k * 10
            text(psp, pin, (x, 101.5), h=1.4, align=TA.BOTTOM_CENTER)
            line(psp, [(x, 100), (x, 92)], "WIRE-COLOURS", color=color)
            text(psp, cname, (x + 0.8, 96), h=1.3, align=TA.MIDDLE_LEFT, layer="WIRE-COLOURS")
        rect(psp, x0, 84, x0 + 40, 92)
        text(psp, f"LOAD CELL {corner} 10 kg", (x0 + 20, 88), h=1.8, align=TA.MIDDLE_CENTER)
    text(psp, "On the tilting plate. FL/FR/RL/RR = front-left ... rear-right, looking down, front away from you.",
         (28, 79), h=1.8)
    text(psp, "A cell that reads negative: swap its A+/A- (green/white), or keep sign -1 (\"cal\" sets it).",
         (28, 75.5), h=1.8)
    # ultrasonic sensor (bottom right)
    x0 = 296
    rect(psp, x0, 100, x0 + 30, 116)
    text(psp, "HC-SR04 (front)", (x0 + 15, 108), h=1.9, align=TA.MIDDLE_CENTER)
    for k, (pin, net) in enumerate((("VCC", "+5V"), ("TRIG", "SONAR_TRIG"), ("ECHO", ""), ("GND", "GND"))):
        x = x0 + 4 + k * 7.3
        text(psp, pin, (x, 101.5), h=1.3, align=TA.BOTTOM_CENTER)
        line(psp, [(x, 100), (x, 95)], net_layer(net) if net else "NET-SIGNAL")
        if net:
            text(psp, net, (x + 0.6, 94), h=1.6, align=TA.MIDDLE_RIGHT, layer=net_layer(net), rot=90)
    xe = x0 + 4 + 2 * 7.3
    resistor_v(psp, xe, 95, 81, "", "1k", "NET-SIGNAL")
    dot(psp, (xe, 81), "NET-SIGNAL")
    line(psp, [(xe, 81), (xe + 18, 81)], "NET-SIGNAL")
    net_label(psp, (xe + 19, 81), "SONAR_ECHO", TA.MIDDLE_LEFT, h=1.9)
    resistor_v(psp, xe, 81, 67, "", "2k", "NET-GND")
    ground(psp, (xe, 67))
    text(psp, "ECHO is 5 V: the 1k / 2k divider makes it 3.3 V.", (296, 121), h=1.8)
    text(psp, "Same net name = same wire. Wire IDs S1..S38: BUILD_GUIDE.md week 3.", (28, 70), h=1.9)
    return psp


# ============================== E04 I2C, line sensor, panel ==============================
def sheet_io():
    psp = new_sheet(4, "I2C, LINE SENSOR AND PANEL", "I2C, LINE SENSOR AND PANEL")
    port_b = {P["XP_MOTOR_EN"]: ("MOTOR_EN", ""), P["XP_BUZZER"]: ("BUZZER", ""),
              P["XP_LED_GREEN"]: ("LED_G", ""), P["XP_LED_RED"]: ("LED_R", ""), P["XP_BUTTON"]: ("BUTTON", "")}
    left = [(f"GPA{i}", f"LINE_{i + 1}", "") for i in range(8)]
    left += [("VDD", "3V3", ""), ("VSS", "GND", ""), ("A0 A1 A2", "GND", "addr 0x20"), ("RESET", "3V3", "")]
    right = [(f"GPB{i}", port_b.get(8 + i, ("", ""))[0], port_b.get(8 + i, ("", "spare"))[1] or "") for i in range(8)]
    right += [("SCL", "SCL", ""), ("SDA", "SDA", "")]
    connector(psp, 175, 262, 34, "U4 MCP23017 (left side)", left, pitch=5, side="left", stub=12)
    connector(psp, 215, 262, 34, "U4 (right side)", right, pitch=5, side="right", stub=12)
    # line sensor board
    connector(psp, 28, 262, 34, "LINE SENSOR (8 x TCRT5000)", [("VCC", "3V3", ""), ("GND", "GND", "")] +
              [(f"OUT{i}", f"LINE_{i}", "left" if i == 1 else ("right" if i == 8 else "")) for i in range(1, 9)],
              pitch=5, stub=10)
    text(psp, "OUT1 = leftmost sensor (looking forward).", (28, 200), h=1.8)
    text(psp, "Set each sensor's pot so its LED goes on over", (28, 196.5), h=1.8)
    text(psp, "black tape and off over the floor.", (28, 193), h=1.8)
    # VL53L0X
    connector(psp, 330, 262, 34, "VL53L0X (right side)", [("VIN", "3V3", ""), ("GND", "GND", ""),
                                                          ("SCL", "SCL", ""), ("SDA", "SDA", ""),
                                                          ("XSHUT", "", "n.c."), ("GPIO1", "", "n.c.")],
              pitch=5, side="left", stub=10)
    text(psp, f"ESP32: SDA = GPIO {P['PIN_SDA']}, SCL = GPIO {P['PIN_SCL']}", (300, 218), h=2.0)
    text(psp, "4.7k pull-ups: SDA and SCL to 3V3 (once, on the I/O board)", (300, 214), h=1.8)
    # panel: button, LEDs, buzzer, motor-enable pull-down
    y = 170
    text(psp, "PANEL AND PULL-RESISTORS (on the I/O perfboard)", (28, y + 18), h=2.4)
    switch_h(psp, 40, 80, y, "S2 START", "12 mm, NO", "NET-SIGNAL")
    net_label(psp, (38, y), "BUTTON", TA.MIDDLE_RIGHT)
    ground(psp, (80, y))
    for k, (net, ref, col) in enumerate((("LED_G", "D1 green", 3), ("LED_R", "D2 red", 1))):
        x = 110 + k * 45
        net_label(psp, (x, y + 4), net, TA.BOTTOM_CENTER)
        resistor_v(psp, x, y + 2, y - 14, "", "330", "NET-SIGNAL")
        psp.add_lwpolyline([(x - 3, y - 14), (x + 3, y - 14), (x, y - 19), (x - 3, y - 14)],
                           dxfattribs={"layer": "SYMBOLS", "color": col})
        line(psp, [(x - 3, y - 19), (x + 3, y - 19)], "SYMBOLS")
        text(psp, ref, (x + 4, y - 17), h=1.8)
        line(psp, [(x, y - 19), (x, y - 23)], "NET-GND")
        ground(psp, (x, y - 23))
    rect(psp, 200, y - 20, 240, y + 2)
    text(psp, "BUZZER MODULE", (220, y - 1), h=1.9, align=TA.TOP_CENTER)
    for k, (pin, net) in enumerate((("VCC", "3V3"), ("I/O", "BUZZER"), ("GND", "GND"))):
        yy = y - 6 - k * 5
        text(psp, pin, (201, yy), h=1.6, align=TA.MIDDLE_LEFT)
        line(psp, [(200, yy), (192, yy)], net_layer(net))
        net_label(psp, (191, yy), net, TA.MIDDLE_RIGHT, h=1.9)
    x = 275
    net_label(psp, (x, y + 4), "MOTOR_EN", TA.BOTTOM_CENTER)
    resistor_v(psp, x, y + 2, y - 18, "R3", "10 k", "NET-GND")
    ground(psp, (x, y - 18))
    text(psp, "keeps the drivers OFF until", (x + 10, y - 22), h=1.7)
    text(psp, "the ESP32 has started", (x + 10, y - 25), h=1.7)
    for k, net in enumerate(("SDA", "SCL")):
        xx = 340 + k * 25
        net_label(psp, (xx, y - 20), net, TA.TOP_CENTER)
        resistor_v(psp, xx, y + 2, y - 18, "", "4.7 k", "NET-SIGNAL")
        net_label(psp, (xx, y + 4), "3V3", TA.BOTTOM_CENTER)
    text(psp, "NOTES", (28, 120), h=2.8)
    for k, s in enumerate([
        "1. All logic on this sheet runs on 3.3 V from the ESP32's 3V3 pin. Never feed 5 V to the MCP23017 or VL53L0X.",
        "2. Line sensor: 1 = output HIGH over black tape (setting line_black). Check with the \"line\" command.",
        "3. Same net name = same wire. Pin numbers: firmware/cargo_robot/config.h.",
    ]):
        text(psp, s, (28, 114 - k * 4.5), h=2.0)
    legend(psp, 28, 96)
    return psp


# ============================== main ==============================
def main():
    OUT.mkdir(exist_ok=True)
    (OUT / "previews").mkdir(exist_ok=True)
    setup()
    sheet_block_diagram()
    sheet_power()
    sheet_esp32()
    sheet_io()
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
            png = OUT / "previews" / (layout.name.replace(" ", "_").replace("&", "and").replace(",", "") + ".png")
            fig.savefig(png, dpi=130)
            plt.close(fig)
            print("sheet", layout.name)
    print("wrote", OUT / f"{NAME}.pdf")


if __name__ == "__main__":
    main()
