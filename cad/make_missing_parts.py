"""
Shopping list of the parts still to buy (12 Nov 2026): the 3D-printed parts and the laser-cut plates (all 6 mm
acrylic) are done, and so are the parts listed in HAVE.

Run:   .venv\\Scripts\\python make_missing_parts.py
Writes ../Missing_Parts_List.pdf
"""
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import Flowable, KeepTogether, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle

OUT = Path(__file__).resolve().parent.parent / "Missing_Parts_List.pdf"
DATE = "12 November 2026"

FONTS = Path("C:/Windows/Fonts")
pdfmetrics.registerFont(TTFont("Arial", str(FONTS / "arial.ttf")))
pdfmetrics.registerFont(TTFont("Arial-Bold", str(FONTS / "arialbd.ttf")))
pdfmetrics.registerFont(TTFont("Arial-Italic", str(FONTS / "ariali.ttf")))
pdfmetrics.registerFontFamily("Arial", normal="Arial", bold="Arial-Bold", italic="Arial-Italic",
                              boldItalic="Arial-Bold")

INK = colors.HexColor("#1f2933")
MUTED = colors.HexColor("#5b6670")
ACCENT = colors.HexColor("#1d5b8f")
FIRST = colors.HexColor("#fff4d6")
FIRST_EDGE = colors.HexColor("#d9a400")
CHANGE = "#b45309"
RULE = colors.HexColor("#cfd6dd")
ZEBRA = colors.HexColor("#f4f6f8")
GREEN = colors.HexColor("#2e7d32")

title = ParagraphStyle("title", fontName="Arial-Bold", fontSize=20, leading=24, textColor=INK)
sub = ParagraphStyle("sub", fontName="Arial", fontSize=10, leading=14, textColor=MUTED)
h2 = ParagraphStyle("h2", fontName="Arial-Bold", fontSize=12.5, leading=16, textColor=ACCENT, spaceBefore=10,
                    spaceAfter=4)
cell = ParagraphStyle("cell", fontName="Arial", fontSize=9, leading=11.5, textColor=INK, alignment=TA_LEFT)
note = ParagraphStyle("note", parent=cell, textColor=MUTED, fontSize=8.5, leading=11)
head = ParagraphStyle("head", parent=cell, fontName="Arial-Bold", textColor=colors.white)
body = ParagraphStyle("body", fontName="Arial", fontSize=9.5, leading=13.5, textColor=INK)

NEW = f' <font color="{CHANGE}"><b>[changed]</b></font>'   # changed with the 6 mm acrylic plates / fixed


class Box(Flowable):
    """A tick box, empty or ticked."""
    def __init__(self, ticked=False, size=3.6 * mm):
        super().__init__()
        self.size, self.ticked = size, ticked

    def wrap(self, *_):
        return self.size, self.size

    def draw(self):
        c, s = self.canv, self.size
        c.setStrokeColor(GREEN if self.ticked else INK)
        c.setLineWidth(0.8)
        c.rect(0, 0, s, s)
        if self.ticked:
            c.setLineWidth(1.4)
            c.line(0.2 * s, 0.5 * s, 0.42 * s, 0.22 * s)
            c.line(0.42 * s, 0.22 * s, 0.85 * s, 0.85 * s)


# (section title, highlight, [(item, qty, what to check / what it's for)])
SECTIONS = [
    ("Buy these first: drive, battery and servo power", True, [
        ("<b>JGB37-520 12 V 110 RPM gear motor SET</b>: motor with Hall encoder (6 wires) + steel L-bracket "
         "+ hex coupling + <b>65 mm wheel</b>", "2",
         "One set per side, both the same RPM. Shaft is 6 mm with a flat (sellers often write \"5.5 mm "
         "D-shaft\"); the coupling must fit it. <b>2 wheels is all you need</b>: they drive and steer, and "
         "the 2 printed ball casters (front and rear) carry the rest."),
        ("<b>Battery: 3S LiPo 11.1 V 5000 mAh</b>, 20 C or more, <b>XT60</b> plug", "1<br/>(2 is better)",
         "\"3S\" = 3 cells. Check it has the small white balance plug. A second battery saves the demo day."),
        ("<b>Buck converter XL4016 8 A</b>, with voltage pot and heat sink", "1",
         "<b>Needed</b>: it feeds the 2 servos at 7.4 V. The battery's 11-12.6 V would burn them (8.4 V max), "
         "and on your 5 A converter the servos would pull the voltage down and reset the ESP32 while the bed "
         "tips. Keep the 5 A one for the ESP32 and sensors."),
    ]),
    ("Battery care", False, [
        ("LiPo balance charger (IMAX B6 / B6AC class)", "1",
         "The plain B6 also needs a 12 V, 5 A or bigger power adapter; the B6AC has one built in."),
        ("LiPo fire-safe bag", "1", "Charge and store the battery inside it."),
        ("LiPo voltage alarm (1-8 S buzzer)", "1", "Plugs into the balance lead. Set it to 3.5 V per cell."),
    ]),
    ("Panel parts: the buttons, switch and lights on the top deck", False, [
        ("Emergency-stop button, 22 mm panel-mount, latching, with an <b>NC</b> contact block (10 A)" + NEW, "1",
         "NC = normally closed: power flows until you press it. Check it clamps a <b>6 mm</b> panel."),
        ("Toggle switch (master), 12 mm hole, rated <b>12 V DC, 15 A or more</b>" + NEW, "1",
         "The DC rating matters, not the AC one. Thread at least <b>9 mm</b> long (6 mm deck + nut)."),
        ("Push button, 12 mm panel-mount, momentary, normally open" + NEW, "1",
         "The start button. Thread at least <b>9 mm</b> long."),
        ("5 mm LEDs, green + red, with 5 mm panel clips" + NEW, "1 each",
         "Check the clips fit a 6 mm panel. If they don't, skip them and hot-glue the LEDs into the 6 mm holes "
         "from below."),
        ("Active buzzer module, 3 pins, 3.3-5 V", "1", "\"Active\" = it beeps by itself when powered."),
    ]),
    ("Power wiring and connectors", False, [
        ("Inline blade-fuse holder (14 AWG) + 10 A blade fuses", "1 + 5", ""),
        ("XT60 connector pairs (male + female)", "4", ""),
        ("WAGO 221-415 lever connector (5-way)", "8", "Power buses without soldering."),
        ("WAGO 221-413 lever connector (3-way)", "6", ""),
        ("Silicone wire 16 AWG, red + black", "3 m each",
         "Battery, fuse, switch, E-stop, motor drivers, servo supply."),
        ("Silicone wire 20 AWG, red + black", "2 m each", "Buck converters, motors."),
    ]),
    ("Signal wiring and small parts", False, [
        ("Jumper wires, female-female and male-female, 20 cm", "40 each", "Signals."),
        ("Servo extension leads, 30 cm", "2", "Servo to the I/O board."),
        ("USB <b>data</b> cable for the ESP32 (micro-USB or USB-C, to match the board), 1 m or longer", "1",
         "Some cheap cables only charge: try it on the laptop first."),
        ("Resistors 1/4 W: 1 kΩ × 3, 2 kΩ × 3, 10 kΩ × 5, 2.2 kΩ × 1, 330 Ω × 2", "1 set",
         "Voltage dividers, pull-ups, LEDs."),
        ("Capacitor 100 nF (ceramic)", "1", ""),
        ("Perfboard 5 × 7 cm", "2", "The I/O board (1 spare)."),
        ("Ferrule kit + ferrule crimper", "1", "A ferrule on every wire that goes into a screw terminal."),
        ("Heat-shrink assortment, zip ties, spiral wrap, 20 mm velcro strap (1 m), double-sided foam tape",
         "1 each", ""),
        ("VHB tape (strong double-sided foam), 19 mm", "1 roll", "Holds the small boards without drilling."),
    ]),
    ("Rods, bolts, screws, nuts and washers (sizes checked for the 6 mm acrylic plates)", False, [
        ("M6 threaded rod, 1 m, zinc-plated steel", "1", "Cut into 6 pieces of 130 mm."),
        ("M6 nuts + M6 washers", "26 each", "24 needed (4 per rod)."),
        ("M8 × 50 bolt + 2 M8 nuts + 2 M8 washers", "2 sets", "One set per ball caster."),
        ("M4 × 10", "2", "Bed rests."),
        ("M4 × 12" + NEW, "6",
         "Servo mounts. <b>No washer</b> under the head here: that leaves about 4 turns of thread in the nut, "
         "plenty. <b>Not longer</b>: the tip would poke out of the foot into the servo."),
        ("M4 × 16", "4", "Servo ear clamps."),
        ("M4 × 20", "12", "Horn adapters (4) + risers from below (8)."),
        ("M4 × 20 <b>countersunk (flat head)</b>", "8",
         "Down through the cargo plate into the risers: flush, so the package slides over them."),
        ("M4 nuts + M4 washers", "32 each", ""),
        ("M3 × 8", "8", "Disc horns onto the horn adapters. <b>Not longer</b>: longer screws hit the "
                        "adapter's flange."),
        ("M3 × 8 <b>countersunk</b>", "6", "Caster rings onto the cups."),
        ("M3 × 12 + M3 nuts", "11 each",
         "Motor brackets (8) + IR modules onto the sensor bar (3). On the motor brackets, check the tip "
         "doesn't touch the motor."),
        ("M3 × <b>16</b> + M3 nuts" + NEW, "4 each",
         "Line sensor bar (2) + ultrasonic bracket (2): the printed part + 6 mm plate + nut. M3 × 12 is too "
         "short (only about 1 thread in the nut)."),
        ("M3 washers" + NEW, "30",
         "Under the M3 heads and nuts on the bottom plate. It's acrylic, not plywood: washers spread the "
         "load so it doesn't crack."),
    ]),
    ("Casters, track and finishing", False, [
        ("Steel bearing ball, 25.4 mm (1 inch)", "3",
         "2 ball casters + 1 spare. A 25 mm glass marble also works."),
        ("Black <b>matte</b> tape, <b>19 mm</b> wide", "1 roll",
         "The line. Not 50 mm: it would cover all 3 sensors and look like a station mark."),
        ("Masking tape", "1 roll", "Laying out the track."),
        ("Self-adhesive rubber pads, 18 mm, 2 mm thick", "2", "On the 2 bed rests."),
        ("Blue thread-locker (medium strength)", "1", "Coupling set screws, horn screws."),
        ("Grease (Vaseline is fine)", "a little", "Caster cups."),
    ]),
]

ACRYLIC = [
    "All four plates are 6 mm acrylic. The first drawings had 6 mm <i>plywood</i> for the bottom and tilting "
    "plates and 5 mm acrylic for the deck and cargo plate, so only the deck and the cargo plate got 1 mm "
    "thicker. The drawings, the build guide and this list now all match. Items marked "
    "<font color=\"%s\"><b>[changed]</b></font> are the ones this affected." % CHANGE,
    "Almost nothing else changes: the rods stay 130 mm and the other screws are still long enough. The top "
    "of the cargo plate is about <b>220 mm</b> above the floor (was 218).",
    "Acrylic cracks more easily than plywood and bends about twice as much. The loads on this robot are "
    "small, so it's fine, but put washers under heads and nuts on the bottom plate, tighten only until snug, "
    "and check the rod, caster and motor-bracket holes for cracks after the first test drives.",
    "The only holes you drill yourself are the motor-bracket holes, now in acrylic. Use a sharp bit, drill "
    "slowly, with a piece of wood clamped underneath. Countersink the cargo plate's 8 holes slowly too.",
    "<b>Two fixes to the old list</b> (not caused by the acrylic): the M3 × 12 screws for the line sensor bar "
    "and the ultrasonic bracket were always too short (now M3 × 16), and the servo-horn screws are M3 × 8, "
    "not M3 × 10.",
]

HAVE = [
    "All 3D-printed parts and all laser-cut plates (6 mm acrylic)",
    "2 × ESP32 DevKitC 38-pin (one spare) + the 38-pin screw-terminal breakout board",
    "2 × BTS7960 (IBT-2) motor drivers",
    "XL4015 5 A buck converter (for the ESP32 and sensors, set to 5.1 V)",
    "2 × DS5160 servos + 2 round metal disc horns (they fit the printed horn adapters)",
    "Sensors: the 3-pin IR line modules (3 + 1 spare) and the HC-SR04 ultrasonic sensor",
    "Tools (soldering iron, multimeter, drill and bits, spanners, ...)",
    "A light-coloured floor for the track (the black tape goes straight on it)",
]

QUESTIONS = [
    "<b>Panel parts you may already own.</b> Panel parts are the controls that mount through holes in the top "
    "deck, like on a control panel: the red emergency-stop button, the master on/off switch, the start button "
    "and the green and red LEDs (section 3). Each is held by a nut on a threaded part underneath the deck. If "
    "you already own any of them, check its thread is at least 9 mm long, so it reaches through the 6 mm deck "
    "and still takes its nut. If you still have to buy them, just check this in the shop.",
]


def section_table(rows, first, start):
    data = [["", Paragraph("#", head), Paragraph("Item", head), Paragraph("Qty", head),
             Paragraph("What to check / what it's for", head)]]
    for i, (item, qty, why) in enumerate(rows):
        data.append([Box(), Paragraph(str(start + i), note), Paragraph(item, cell), Paragraph(qty, cell),
                     Paragraph(why, note)])
    t = Table(data, colWidths=[8 * mm, 8 * mm, 70 * mm, 19 * mm, 75 * mm], repeatRows=1)
    style = [
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#8a6400") if first else ACCENT),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
        ("LEFTPADDING", (0, 0), (-1, -1), 4),
        ("RIGHTPADDING", (0, 0), (-1, -1), 4),
        ("LINEBELOW", (0, 1), (-1, -1), 0.4, RULE),
        ("TOPPADDING", (0, 1), (0, -1), 5.5),
    ]
    for r in range(1, len(data)):
        if first:
            style.append(("BACKGROUND", (0, r), (-1, r), FIRST))
        elif r % 2 == 0:
            style.append(("BACKGROUND", (0, r), (-1, r), ZEBRA))
    if first:
        style.append(("BOX", (0, 0), (-1, -1), 1, FIRST_EDGE))
    t.setStyle(TableStyle(style))
    return t


def bullet_list(lines, ticked=False):
    rows = [[Box(ticked=True) if ticked else Paragraph("•", cell), Paragraph(t, cell)] for t in lines]
    return Table(rows, colWidths=[8 * mm, 172 * mm],
                 style=[("VALIGN", (0, 0), (-1, -1), "TOP"), ("TOPPADDING", (0, 0), (-1, -1), 2.5),
                        ("BOTTOMPADDING", (0, 0), (-1, -1), 2.5), ("LEFTPADDING", (0, 0), (-1, -1), 4)])


def footer(canv, doc):
    canv.saveState()
    canv.setFont("Arial", 8)
    canv.setFillColor(MUTED)
    canv.drawString(15 * mm, 10 * mm, f"Cargo robot: parts still missing  |  {DATE}")
    canv.drawRightString(A4[0] - 15 * mm, 10 * mm, f"page {doc.page}")
    canv.restoreState()


def main():
    story = [
        Paragraph("Cargo Robot: Parts Still Missing", title),
        Spacer(1, 3),
        Paragraph(f"Shopping list, {DATE}. The 3D-printed parts and the laser-cut plates (all <b>6 mm acrylic</b>) "
                  "are done, and so is everything in the \"already have\" section, so everything below is <b>to "
                  "buy</b>. Tick each box when you have it. Names and sizes match the build guide (section 0.2) and "
                  f"the drawings. <font color=\"{CHANGE}\"><b>[changed]</b></font> marks what changed with the "
                  "acrylic plates.", sub),
        Spacer(1, 6),
    ]
    n = 1
    for k, (name, first, rows) in enumerate(SECTIONS, start=1):
        story.append(KeepTogether([Paragraph(f"{k}. {name}", h2), section_table(rows, first, n)]))
        n += len(rows)
    k = len(SECTIONS)
    story.append(KeepTogether([Paragraph(f"{k + 1}. Because the plates are 6 mm acrylic", h2),
                               bullet_list(ACRYLIC)]))
    story.append(KeepTogether([Paragraph(f"{k + 2}. You already have", h2), bullet_list(HAVE, ticked=True)]))
    qs = [Paragraph(f"{k + 3}. One question left", h2)]
    for i, q in enumerate(QUESTIONS, 1):
        qs.append(Table([[Paragraph(f"Q{i}", ParagraphStyle("qn", parent=body, fontName="Arial-Bold",
                                                            textColor=ACCENT)), Paragraph(q, body)]],
                        colWidths=[10 * mm, 170 * mm],
                        style=[("VALIGN", (0, 0), (-1, -1), "TOP"), ("TOPPADDING", (0, 0), (-1, -1), 3),
                               ("BOTTOMPADDING", (0, 0), (-1, -1), 3)]))
    story.append(KeepTogether(qs))
    doc = SimpleDocTemplate(str(OUT), pagesize=A4, leftMargin=15 * mm, rightMargin=15 * mm, topMargin=15 * mm,
                            bottomMargin=17 * mm, title="Cargo Robot: Parts Still Missing",
                            author="Cargo robot project")
    doc.build(story, onFirstPage=footer, onLaterPages=footer)
    print(f"{n - 1} items to buy -> {OUT}")


if __name__ == "__main__":
    main()
