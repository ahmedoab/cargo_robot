"""
Assembly and connection documents of the cargo robot (design v2, tilting bed), as PDFs.

Run (after robot_cad.py, make_autocad.py and make_schematics.py):
    .venv\\Scripts\\python make_docs.py           (needs: pip install reportlab pypdf)
Writes ./autocad/
  CargoRobot_v2_assembly.pdf     every component (bought, laser-cut, printed), tools, and the assembly step by
                                 step with a picture of each stage; ends with drawing sheets 01, 02 and 07
  CargoRobot_v2_connections.pdf  every wire: power list, signal list, the I/O board, the connections of each part;
                                 ends with the 4 schematic sheets
and brings these parts of ../BUILD_GUIDE.md up to date: the power and signal wire tables (from wiring.py) and
Appendix B (the firmware source).
"""
import tempfile
from pathlib import Path

import cadquery as cq
from cadquery.vis import show
from pypdf import PdfReader, PdfWriter
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (Image, KeepTogether, PageBreak, Paragraph, SimpleDocTemplate, Spacer, Table,
                                TableStyle)

import robot_cad as rc
import wiring

HERE = Path(__file__).parent
OUT = HERE / "autocad"
GUIDE = HERE.parent / "BUILD_GUIDE.md"
FIRMWARE = HERE.parent / "firmware" / "cargo_robot"
PROJECT = "Autonomous cargo robot - design v2"
DATE = "22 October 2026"

ss = getSampleStyleSheet()
BODY = ParagraphStyle("body", parent=ss["Normal"], fontSize=9.5, leading=12.5)
SMALL = ParagraphStyle("small", parent=BODY, fontSize=8, leading=10)
CELL = ParagraphStyle("cell", parent=BODY, fontSize=8, leading=9.6)
CELL_B = ParagraphStyle("cellb", parent=CELL, fontName="Helvetica-Bold")
H1 = ParagraphStyle("h1", parent=ss["Heading1"], fontSize=16, spaceBefore=4, spaceAfter=8)
H2 = ParagraphStyle("h2", parent=ss["Heading2"], fontSize=12, spaceBefore=10, spaceAfter=4)
TITLE = ParagraphStyle("title", parent=ss["Title"], fontSize=24, leading=30)
CENTER = ParagraphStyle("center", parent=BODY, alignment=TA_CENTER)
HEAD_BG = colors.HexColor("#dde4ee")


def p(text, style=BODY):
    return Paragraph(text, style)


def table(rows, widths, header=True, zebra=True):
    """rows of strings -> a grid table with wrapped cells."""
    data = [[p(str(c), CELL_B if (header and i == 0) else CELL) for c in r] for i, r in enumerate(rows)]
    t = Table(data, colWidths=[w * mm for w in widths], repeatRows=1 if header else 0)
    style = [("GRID", (0, 0), (-1, -1), 0.4, colors.grey), ("VALIGN", (0, 0), (-1, -1), "TOP"),
             ("TOPPADDING", (0, 0), (-1, -1), 2), ("BOTTOMPADDING", (0, 0), (-1, -1), 2)]
    if header:
        style.append(("BACKGROUND", (0, 0), (-1, 0), HEAD_BG))
    if zebra:
        for i in range(2 if header else 1, len(rows), 2):
            style.append(("BACKGROUND", (0, i), (-1, i), colors.HexColor("#f5f7fa")))
    t.setStyle(TableStyle(style))
    return t


def picture(path, width_mm):
    img = Image(str(path))
    ratio = img.imageHeight / img.imageWidth
    img.drawWidth, img.drawHeight = width_mm * mm, width_mm * mm * ratio
    return img


def footer(title):
    def draw(canvas, doc):
        canvas.saveState()
        canvas.setFont("Helvetica", 7.5)
        w, _ = doc.pagesize
        canvas.drawString(doc.leftMargin, 8 * mm, f"{PROJECT} - {title}")
        canvas.drawRightString(w - doc.rightMargin, 8 * mm, f"page {doc.page}")
        canvas.restoreState()
    return draw


def append_pages(pdf_path, source, pages):
    """Add pages (0-based) of another PDF to the end of pdf_path, unchanged (vector, own page size)."""
    writer = PdfWriter(clone_from=str(pdf_path))
    reader = PdfReader(str(source))
    for i in pages:
        writer.add_page(reader.pages[i])
    with open(pdf_path, "wb") as f:
        writer.write(f)


# ============================== stage pictures ==============================
STAGES = [
    ("Motors under the bottom plate", ("bottom_plate", "motor_bracket", "gearmotor", "shaft"), [
        "Fix each motor in its steel bracket with the bracket's M3 screws, shaft through the outer face.",
        "Turn the bottom plate upside down. Place each bracket 5 mm in from the side edge, centred at X = 0, "
        "the motor pointing inwards. Mark its base holes, drill 4.5 mm, fix with M4 x 12 + washer + nylock nut.",
        "Route each motor's 6-wire cable up through the nearest cable slot (X = +40, Y = +-70).",
    ]),
    ("Drive wheels", ("tire", "rim"), [
        "A drop of blue thread-locker on each set screw.",
        "Slide the hub onto the 6 mm D-shaft, about 2 mm clear of the bracket; tighten the set screw on the flat.",
    ]),
    ("Threaded rods", ("rod_",), [
        "Cut 6 pieces of 130 mm from the M6 rod (screw a nut on first, unscrew it over the cut end).",
        "Each rod: nut + washer 7 mm from the bottom end, up through the plate, washer + nut on top, tighten.",
        "A third nut + washer on each rod with its top exactly 100 mm above the plate (use one gauge for all 6). "
        "The deck rests on these washers. Rods at X = -190, +60, +190, Y = +-135.",
    ]),
    ("Casters", ("caster",), [
        "On each M8 stem: nut down to the caster body + washer, up through the 8.5 mm hole (X = +-160), "
        "washer + nut on top. The height is set later, with all 4 wheels on the floor.",
    ]),
    ("Sensors: line sensor underneath, ultrasonic in front", ("line_sensor", "tcrt5000", "sonar_bracket", "hc_sr04"), [
        "Screw the 8-channel line sensor under the shelf of the line-sensor bracket (2 x M3), sensors down, "
        "sensor 1 on the robot's LEFT.",
        f"Bolt the bracket under the bottom plate, between the drive wheels and the front caster: 2 x M3 x 12 "
        f"through the 2 holes on the centre line (X = {rc.LINE_X} +-12). The sensors end up about 5 mm above the floor.",
        "Push the HC-SR04 into the ultrasonic bracket from behind (cans through the windows); hot glue or "
        "2 zip ties. Bolt the bracket on top of the plate's front edge (2 x M3).",
        "Check that the front caster still swivels a full turn without touching the line-sensor bracket.",
    ]),
    ("Electronics", ("battery", "BTS7960", "esp32", "io_board", "buck", "servo_buck", "power_bus", "inline_fuse",
                     "buzzer"), [
        "Stick every board down with VHB tape, where the picture shows (sheet 02 has the layout with numbers).",
        "BTS7960 drivers at the rear, heatsinks up. ESP32 breakout at the front-left, USB socket towards the "
        "robot's left side. I/O board at the front-right. 5 V buck rear-left, 7.4 V servo buck rear-right, "
        "WAGO buses between the battery and the drivers.",
        "The battery lies across the middle in 2 velcro straps (through the 4 slots) and slides out sideways.",
        "Wiring: weeks 2 and 3 of BUILD_GUIDE.md and CargoRobot_v2_connections.pdf.",
    ]),
    ("Deck: controls, side sensor, servo mounts, bed rests", ("top_deck", "estop", "switch", "button", "led_",
                                                              "tof", "vl53l0x", "servo_mount", "bed_rest"), [
        "Before the deck goes on: E-stop through the 22.5 mm hole (front-left) from above, ring nut below, contact "
        "block clipped on, 2 x 40 cm 16 AWG on its NC terminals.",
        "Master switch (front, middle) and start button (rear) up through their 12.2 mm holes; LEDs in their clips "
        "next to the button (green nearer the button).",
        "Side sensor: VL53L0X behind the window of its bracket, bracket under the deck's right edge (2 x M3).",
        "Servo mounts on the deck's front-right and rear-right corners (3 x M4 each); bed rests at X = +-40, "
        "Y = +95 (M4 from below into the nut in the post), rubber pad on top.",
        "Lower the deck onto the 6 rods, washer + nut on top of each, check it is level.",
    ]),
    ("Servos", ("servo_front", "servo_rear"), [
        "Each servo lies in its mount: body through the window, shaft towards the middle of the robot and at the "
        "deck's right edge, cable at the bottom. 4 x M4 x 12 + nut through the tabs.",
        "Check your servo's tab holes against the mount first (74 x 18 mm in the model).",
        "Servo cables down through the cut-outs next to the mounts.",
    ]),
    ("The bed (shown on its own)", ("tilt_plate", "load_cell", "cell_spacer", "hx711", "horn_adapter",
                                    "servo_horn"), [
        "Load cells on the tilting plate: spacer + cell's fixed end over each slot, arrow pointing down, 2 screws "
        "up from below into the cell. Fixed ends outwards, loaded ends towards the middle. The screws must not "
        "come out of the top of the cell.",
        "HX711 boards on the plate between the cells (VHB tape).",
        "Horn adapters on the plate's two right-hand corners (2 x M4 x 20 each, nuts underneath), then a disc "
        "horn on each adapter's round wall (4 x M3 x 10 into the 2.8 mm holes, thread-locker).",
    ]),
    ("Cargo plate on, bed on the servos", None, [
        "A spacer on each cell's loaded end, the cargo plate on top, 2 screws down through each slot into the "
        "cell. The cargo plate touches only the 4 spacers. No anti-slip mat: the package must slide off.",
        "Lay the bed on its 2 rest posts and push the two horns onto the servo splines (turn a shaft gently by "
        "hand, one tooth at a time, until its horn fits). Fit the horns' centre screws with thread-locker.",
        "Set the casters: all 4 wheels on the floor, the casters just touching (paper test).",
    ]),
    ("Check: the bed swings freely", "tipped", [
        "Lift the bed's left edge by hand (servos unpowered): it must swing to 45 deg and back without touching "
        "anything. The cargo plate's right edge ends just outside the deck.",
        "The servo end points are set later with the 'servo' command (BUILD_GUIDE.md, week 4).",
    ]),
]


def stage_assembly(prefixes, angle=0.0):
    assy = cq.Assembly()
    for gname, items in rc.groups.items():
        for name, wp, color in items:
            if prefixes is None or name.startswith(prefixes):
                assy.add(rc.tipped(wp, angle) if gname == "payload" else wp, name=name, color=cq.Color(*color))
    return assy


def render_stages(folder):
    """One picture per assembly stage, each showing everything fitted so far."""
    paths, done = [], ()
    for i, (title, prefixes, _) in enumerate(STAGES, start=1):
        if prefixes == "tipped":
            assy, focus, pos = stage_assembly(None, rc.TIP_ANGLE), (0, 0, 200), (600, -600, 520)
        elif title.startswith("Sensors"):                      # from below: the line sensor is under the plate
            done += prefixes
            assy, focus, pos = stage_assembly(done), (60, 0, 50), (520, -560, -160)
        elif title.startswith("The bed"):
            assy, focus, pos = stage_assembly(prefixes), (0, 0, 215), (430, -430, 420)
        elif prefixes is None:
            assy, focus, pos = stage_assembly(None), (0, 0, 130), (600, -600, 520)
        else:
            done += prefixes
            assy, focus, pos = stage_assembly(done), (0, 0, 100), (600, -600, 520)
        path = folder / f"stage_{i:02d}.png"
        show(assy, screenshot=str(path), interact=False, width=1100, height=760, roll=0, elevation=0, azimuth=0,
             zoom=1.0, trihedron=False, gradient=False, bgcolor=(1, 1, 1), position=pos, focus=focus, viewup=(0, 0, 1))
        paths.append(path)
        print("rendered stage", i)
    return paths


# ============================== the assembly PDF ==============================
LASER = [
    ["bottom_plate.dxf", "6 mm birch plywood", f"{rc.L} x {rc.W}", "1", "sheet 03"],
    ["top_deck.dxf", "5 mm clear acrylic", f"{rc.L} x {rc.W}", "1", "sheet 04"],
    ["tilt_plate.dxf", "6 mm birch plywood", f"{2 * rc.TILT_X} x {rc.TILT_Y1 - rc.Y_AX}", "1", "sheet 05"],
    ["cargo_plate.dxf", "5 mm clear acrylic", f"{rc.CARGO[0]} x {rc.CARGO[1]}", "1", "sheet 06"],
]
PRINTS = [
    ["servo_mount_front.stl, servo_mount_rear.stl", "1 + 1", "foot flat on the bed", "40 %", "none"],
    ["horn_adapter_front.stl, horn_adapter_rear.stl", "1 + 1", "round wall flat on the bed", "100 %",
     "on build plate only"],
    ["bed_rest.stl", "2", "flat end down", "40 %", "none"],
    ["load_cell_spacer.stl", "8", "flat", "100 %", "none"],
    ["line_sensor_bracket.stl", "1", "flange flat on the bed", "40 %", "under the shelf"],
    ["sonar_bracket.stl", "1", "base flat on the bed", "40 %", "none"],
    ["tof_bracket.stl", "1", "top flange flat on the bed", "40 %", "none"],
]
TOOLS = ("Soldering iron and solder; multimeter with a 10 A range; wire stripper, side cutters, ferrule crimper; "
         "hacksaw and file (threaded rod); drill with a 4.5 mm bit (motor brackets: the only holes you drill); "
         "spanners 7, 10 and 13 mm, hex keys, screwdrivers; heat gun; tape measure, square ruler; phone level / "
         "angle app; a 2 kg reference weight (rice weighed on a kitchen scale).")


def assembly_pdf(stage_pngs, path):
    doc = SimpleDocTemplate(str(path), pagesize=A4, leftMargin=15 * mm, rightMargin=15 * mm, topMargin=14 * mm,
                            bottomMargin=16 * mm, title="Cargo robot v2 - assembly", author="Cargo robot team")
    s = [Spacer(1, 18 * mm), p("Cargo robot v2", TITLE), p("Assembly instructions and list of components", CENTER),
         Spacer(1, 6 * mm), picture(stage_pngs[-2], 165), Spacer(1, 4 * mm),
         p(f"Graduation Project II - {DATE}", CENTER), Spacer(1, 8 * mm),
         p("<b>Contents.</b> 1 Components (bought, laser-cut, 3D-printed). 2 Tools. 3 Assembly, step by step, with a "
           "picture of every stage. 4 Final checks. Then drawing sheets 01 (general arrangement), 02 (layout of the "
           "electronics bay) and 07 (the tilting bed), at A3. The wiring is in CargoRobot_v2_connections.pdf."),
         Spacer(1, 3 * mm),
         p("All positions are in millimetres from the centre of the plates: +X is forward, +Y is left. The bed "
           "tips to the robot's RIGHT."),
         PageBreak()]

    s.append(p("1. Components", H1))
    s.append(p("1.1 Bought parts", H2))
    rows = [["No.", "Group", "Part", "Qty", "Make / buy"]]
    rows += [[str(i), g, item, str(q), how] for i, (g, item, q, how) in enumerate(rc.BOM, start=1)]
    s.append(table(rows, [9, 22, 108, 11, 30]))
    s.append(p("1.2 Laser-cut plates (files in cad/output/dxf/)", H2))
    s.append(table([["File", "Material", "Size (mm)", "Qty", "Hole table"]] + LASER, [48, 45, 35, 12, 40]))
    s.append(p("1.3 3D-printed parts (files in cad/output/parts/; PETG, 0.2 mm layers, 4 walls)", H2))
    s.append(table([["File", "Qty", "Orientation", "Infill", "Supports"]] + PRINTS, [70, 14, 46, 16, 34]))
    s.append(p("Before printing the servo mounts, measure your servo's tab holes: the mounts assume 74 mm along "
               "the servo and 18 mm across (SERVO in cad/robot_cad.py).", SMALL))
    s.append(p("2. Tools", H1))
    s.append(p(TOOLS))
    s.append(PageBreak())

    s.append(p("3. Assembly, step by step", H1))
    s.append(p("Use nylock nuts where listed, and don't over-tighten anything that touches acrylic: firm with a "
               "small spanner is enough. Each picture shows everything fitted so far."))
    for i, ((title, _, steps), png) in enumerate(zip(STAGES, stage_pngs), start=1):
        block = [p(f"Step {i}: {title}", H2), picture(png, 150)]
        block += [p(f"{k}. {t}") for k, t in enumerate(steps, start=1)]
        s.append(KeepTogether(block))
        s.append(Spacer(1, 4 * mm))

    s.append(p("4. Final checks", H1))
    for t in ["Push the robot by hand: it rolls straight, turns smoothly, the casters swivel freely, nothing rubs.",
              "All 4 wheels on the floor: each caster grips a sheet of paper, but a firm tug still pulls it out.",
              "3 kg on the cargo plate: nothing bends visibly, the plate touches only the 4 spacers, no wheel lifts.",
              "The bed swings from 0 to 45 deg by hand without touching anything, and lies level on its rests.",
              "Press the E-stop hard: the deck doesn't bend noticeably.",
              "Take photos of every stage for the report."]:
        s.append(p(f"[ ]  {t}"))
    doc.build(s, onFirstPage=footer("assembly"), onLaterPages=footer("assembly"))
    append_pages(path, OUT / "CargoRobot_v2_drawings.pdf", [0, 1, 6])
    print("wrote", path)


# ============================== the connections PDF ==============================
def where(part, pin):
    return f"{part} <b>{pin}</b>" if pin else part


def connections_pdf(path):
    page = landscape(A4)
    doc = SimpleDocTemplate(str(path), pagesize=page, leftMargin=14 * mm, rightMargin=14 * mm, topMargin=12 * mm,
                            bottomMargin=15 * mm, title="Cargo robot v2 - connections", author="Cargo robot team")
    widths = [12, 84, 84, 38, 51]
    s = [p("Cargo robot v2 - all connections", TITLE),
         p(f"Graduation Project II - {DATE}. Every wire of the robot: the power wires (P), the signal wires (S), "
           "the small parts on the I/O board, then the connections of each part. The 4 schematic sheets follow at "
           "the end (A3). GPIO numbers come from firmware/cargo_robot/config.h.", BODY),
         Spacer(1, 3 * mm),
         p("<b>How to read it.</b> One row = one wire, from the first place to the second. Rows with the same ID "
           "belong to one job (e.g. S6 = the VCC of both motor drivers). <b>Label every wire with its ID at both "
           "ends.</b> Buses: WAGO +12 V MAIN (after the switch), WAGO +12 V MOTOR (after the E-stop), WAGO +5 V, "
           "WAGO +7.4 V and WAGO GND are lever connectors on the bottom plate; the 3V3 bus and GND bus are rows of "
           "linked pins on the I/O board (3V3 comes from the ESP32's 3V3 pin).", BODY),
         Spacer(1, 2 * mm),
         p("<b>Never</b> connect 5 V or more to an ESP32 pin, the MCP23017, the encoders, the HX711s or the line "
           "sensor: they all run on 3.3 V. Wire with the master switch OFF and the battery unplugged.", BODY),
         p("Power wires", H2)]
    head = ["ID", "From", "To", "Wire", "Note"]
    s.append(table([head] + [[w, where(a, pa), where(b, pb), wire, note] for w, a, pa, b, pb, wire, note
                             in wiring.POWER], widths))
    s.append(PageBreak())
    s.append(p("Signal wires", H2))
    s.append(table([head] + [[w, where(a, pa), where(b, pb), wire, note] for w, a, pa, b, pb, wire, note
                             in wiring.SIGNAL], widths))
    s.append(p("The I/O board (5 x 7 cm perfboard, schematic E04)", H2))
    s.append(table([["Part", "Connected"]] + [list(r) for r in wiring.IO_BOARD], [50, 219]))
    s.append(PageBreak())
    s.append(p("Connections of each part", H1))
    s.append(p("The same wires again, listed under every part they touch: use it to check one part at a time."))
    for part in wiring.parts():
        rows = [["ID", "Pin here", "Goes to", "Wire", "Note"]]
        rows += [[w, pin, where(other, opin), wire, note] for w, pin, other, opin, wire, note in wiring.of(part)]
        s.append(KeepTogether([p(part, H2), table(rows, [12, 46, 104, 50, 57])]))
    doc.build(s, onFirstPage=footer("connections"), onLaterPages=footer("connections"))
    sch = OUT / "CargoRobot_v2_schematics.pdf"
    append_pages(path, sch, range(len(PdfReader(str(sch)).pages)))
    print("wrote", path)


# ============================== BUILD_GUIDE.md ==============================
def md(text):
    return str(text).replace("|", "/")


def guide_tables():
    def side(part, pin):
        return f"{part} **{pin}**" if pin else part

    power = ["| # | From | To | Wire | Notes |", "|---|---|---|---|---|"]
    power += [f"| {w} | {md(side(a, pa))} | {md(side(b, pb))} | {md(wire)} | {md(note)} |"
              for w, a, pa, b, pb, wire, note in wiring.POWER]
    signal = ["| # | From | To | Notes |", "|---|---|---|---|"]
    signal += [f"| {w} | {md(side(a, pa))} | {md(side(b, pb))} | {md(note if wire.startswith('jumper') else '; '.join(x for x in (wire, note) if x))} |"
               for w, a, pa, b, pb, wire, note in wiring.SIGNAL]
    return "\n".join(power), "\n".join(signal)


def update_guide():
    text = GUIDE.read_text(encoding="utf-8")
    power, signal = guide_tables()
    for key, body in (("<!-- power wires:", power), ("<!-- signal wires:", signal)):
        start = text.index(key)
        start = text.index("\n", start) + 1
        end = text.index("<!-- end -->", start)
        text = text[:start] + body + "\n" + text[end:]
    start = text.index("<!-- appendix B:")
    parts = [text[start:text.index("\n", start)], "## Appendix B — Full source code", "",
             "Every file below is already in the project folder; this is a copy for reading, printing and the "
             "report. Paths are relative to the project folder.", ""]
    for name in ["config.h", "settings.h", "settings.cpp", "hw.h", "hw.cpp", "control.h", "control.cpp", "robot.h",
                 "robot.cpp", "console.h", "console.cpp", "cargo_robot.ino"]:
        src = (FIRMWARE / name).read_text(encoding="utf-8").rstrip("\n")
        parts += [f"### `firmware/cargo_robot/{name}`", "", "```cpp", src, "```", ""]
    GUIDE.write_text(text[:start] + "\n".join(parts), encoding="utf-8", newline="\n")
    print("updated", GUIDE.name)


def main():
    OUT.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        assembly_pdf(render_stages(Path(tmp)), OUT / "CargoRobot_v2_assembly.pdf")
    connections_pdf(OUT / "CargoRobot_v2_connections.pdf")
    update_guide()


if __name__ == "__main__":
    main()
