"""
Parts to purchase for the cargo robot (procurement list CR-V2-PL-001, Rev. B, 12/11/2026).
The 3D-printed parts, the laser-cut plates (all 6 mm acrylic) and the parts already in stock are not listed.

Run:   .venv\\Scripts\\python make_missing_parts.py
Writes ../Missing_Parts_List.pdf  (A4 landscape)
Edit SECTIONS below and run it again (bump REV when the list changes).
"""
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas
from reportlab.platypus import Flowable, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle

OUT = Path(__file__).resolve().parent.parent / "Missing_Parts_List.pdf"
PROJECT = "AUTONOMOUS CARGO ROBOT – DESIGN v2"
DOC_NO, REV, DATE = "CR-V2-PL-001", "B", "12/11/2026"
SCHOOL = "Arab Academy for Science, Technology & Maritime Transport – Graduation Project II"

FONTS = Path("C:/Windows/Fonts")
pdfmetrics.registerFont(TTFont("Arial", str(FONTS / "arial.ttf")))
pdfmetrics.registerFont(TTFont("Arial-Bold", str(FONTS / "arialbd.ttf")))
pdfmetrics.registerFont(TTFont("Arial-Italic", str(FONTS / "ariali.ttf")))
pdfmetrics.registerFontFamily("Arial", normal="Arial", bold="Arial-Bold", italic="Arial-Italic",
                              boldItalic="Arial-Bold")

PAGE = landscape(A4)
MARGIN = 12 * mm
WIDTH = PAGE[0] - 2 * MARGIN
NAVY = colors.HexColor("#1b3a5c")
GREY = colors.HexColor("#5f6b76")
RULE = colors.HexColor("#b9c3cc")
BAND = colors.HexColor("#e7edf3")
INK = colors.HexColor("#1d252c")

title_s = ParagraphStyle("title", fontName="Arial-Bold", fontSize=17, leading=21, textColor=NAVY)
sub_s = ParagraphStyle("sub", fontName="Arial", fontSize=9.5, leading=13, textColor=GREY)
cell = ParagraphStyle("cell", fontName="Arial", fontSize=8.3, leading=10.4, textColor=INK)
cell_b = ParagraphStyle("cell_b", parent=cell, fontName="Arial-Bold")
centre = ParagraphStyle("centre", parent=cell, alignment=1)
head = ParagraphStyle("head", parent=cell, fontName="Arial-Bold", fontSize=8, textColor=colors.white)
band = ParagraphStyle("band", parent=cell, fontName="Arial-Bold", fontSize=8.2, textColor=NAVY)
small = ParagraphStyle("small", parent=cell, fontSize=7.6, leading=9.6, textColor=GREY)

DELTA = "Δ"     # changed for the 6 mm acrylic plates (from the issue of 10/11/2026)

# (category, [(description, specification, qty, unit, application, changed)])
SECTIONS = [
    ("Drive, main battery and servo supply (order first: longest lead time)", [
        ("JGB37-520 gear motor set, 12 V, 110 RPM",
         "Motor with Hall encoder (6-wire cable), steel L-bracket, hex coupling and Ø65 mm wheel. Output shaft "
         "Ø6 mm, D-profile (often listed as \"5.5 mm D-shaft\"). Both sets identical.",
         "2", "set", "Drive, left and right. Two driven wheels; the two ball casters carry the rest.", False),
        ("LiPo battery, 3S",
         "11.1 V, 5000 mAh, discharge rating ≥ 20 C, XT60 connector, with balance lead.",
         "1", "pc", "Main power supply. A second battery is recommended for testing.", False),
        ("Buck converter XL4016",
         "8 A, adjustable output, with heat sink. Set to 7.4 V before connecting the servos (max. 8.4 V).",
         "1", "pc", "Servo supply, kept separate from the 5 V logic supply.", False),
    ]),
    ("Battery charging and safety", [
        ("LiPo balance charger", "IMAX B6AC class (mains supply built in). The plain B6 also needs a 12 V, "
         "≥ 5 A DC adapter.", "1", "pc", "Charging the 3S battery.", False),
        ("LiPo safety bag", "Fire-resistant.", "1", "pc", "Charging and storing the battery.", False),
        ("LiPo voltage alarm", "1–8 S, buzzer, connects to the balance lead. Set to 3.5 V per cell.",
         "1", "pc", "Low-battery warning.", False),
    ]),
    ("Panel components (mounted through the 6 mm top deck)", [
        ("Emergency-stop push button", "Ø22 mm panel mount, mushroom head, latching, normally-closed (NC) "
         "contact block, 10 A. Must clamp a 6 mm panel.", "1", "pc", "Cuts motor and servo power.", True),
        ("Toggle switch", "12 mm panel mount, rated ≥ 15 A at 12 V DC. Thread length ≥ 9 mm.",
         "1", "pc", "Master power switch.", True),
        ("Active buzzer module", "3-pin, 3.3–5 V.", "1", "pc", "Audible signals.", False),
    ]),
    ("Power wiring and connectors", [
        ("Inline blade-fuse holder with fuses", "Holder for 14 AWG wire; 10 A blade fuses.", "1 + 5", "pc",
         "Battery protection.", False),
        ("XT60 connector pair", "Male and female.", "4", "pair", "Battery and power connections.", False),
        ("WAGO 221-415 lever connector", "5-way.", "8", "pc", "Power distribution buses.", False),
        ("WAGO 221-413 lever connector", "3-way.", "6", "pc", "Power distribution buses.", False),
        ("Silicone wire, 16 AWG", "Red and black.", "3 + 3", "m",
         "Battery, fuse, switch, E-stop, motor drivers, servo supply.", False),
        ("Silicone wire, 20 AWG", "Red and black.", "2 + 2", "m", "Buck converters, motors.", False),
    ]),
    ("Signal wiring and small parts", [
        ("Jumper wires", "20 cm, female–female and male–female.", "40 + 40", "pc", "Signal connections.", False),
        ("Servo extension lead", "30 cm.", "2", "pc", "Servos to the I/O board.", False),
        ("Perfboard", "50 × 70 mm.", "2", "pc", "I/O board (1 spare).", False),
        ("Wire ferrule kit with crimping tool", "Assorted sizes for 16–22 AWG.", "1", "set",
         "Wire ends in screw terminals.", False),
        ("Cable management kit", "Heat-shrink assortment, cable ties, spiral wrap, 20 mm hook-and-loop strap "
         "(1 m), double-sided foam tape.", "1", "set", "Wiring; battery retention.", False),
        ("VHB mounting tape", "19 mm wide.", "1", "roll", "Mounting the small boards without drilling.", False),
    ]),
    ("Fasteners (lengths verified for the 6 mm acrylic plates; washers on all acrylic)", [
        ("M6 threaded rod", "Zinc-plated steel, 1 m. Cut into 6 pieces of 130 mm.", "1", "pc",
         "Frame columns.", False),
        ("M6 hex nut", "Zinc-plated steel.", "26", "pc", "Frame columns (24 + 2 spare).", False),
        ("M6 washer", "Zinc-plated steel.", "26", "pc", "Frame columns (24 + 2 spare).", False),
        ("M8 × 50 bolt with 2 nuts and 2 washers", "Zinc-plated steel.", "2", "set", "Ball casters.", False),
        ("M4 × 10 screw", "", "2", "pc", "Bed rests.", False),
        ("M4 × 12 screw", "Fitted without a washer (needs its full length to engage the nut).", "6", "pc",
         "Servo mounts.", True),
        ("M4 × 16 screw", "", "4", "pc", "Servo ear clamps.", False),
        ("M4 × 20 screw", "", "12", "pc", "Horn adapters (4), bed risers from below (8).", False),
        ("M4 × 20 countersunk screw", "Flat head, flush with the cargo plate surface.", "8", "pc",
         "Cargo plate to bed risers.", False),
        ("M4 hex nut", "", "32", "pc", "All M4 joints.", False),
        ("M4 washer", "", "32", "pc", "All M4 joints.", False),
        ("M3 × 8 screw", "Maximum length 8 mm (a longer screw hits the adapter flange).", "8", "pc",
         "Disc horns to horn adapters.", False),
        ("M3 × 8 countersunk screw", "", "6", "pc", "Caster rings to caster cups.", False),
        ("M3 × 12 screw with nut", "", "11", "set", "Motor brackets (8), IR modules (3).", False),
        ("M3 × 16 screw with nut", "Replaces M3 × 12 here (too short through the part, plate and nut).", "4",
         "set", "Line sensor bar (2), ultrasonic bracket (2).", True),
        ("M3 washer", "", "30", "pc", "M3 joints on the acrylic bottom plate.", True),
    ]),
    ("Casters, track and consumables", [
        ("Steel ball", "Ø25.4 mm (1 in), bearing grade. A Ø25 mm glass marble is acceptable.", "3", "pc",
         "Ball casters (2 + 1 spare).", False),
        ("Floor tape, matte black", "19 mm wide (50 mm is too wide: it reads as a station mark).", "1", "roll",
         "Line-following track.", False),
        ("Masking tape", "", "1", "roll", "Track layout.", False),
        ("Rubber pad, self-adhesive", "Ø18 mm × 2 mm.", "2", "pc", "Bed rests.", False),
        ("Thread-locker, medium strength", "Blue (removable).", "1", "pc", "Coupling set screws, horn screws.",
         False),
        ("Grease", "Petroleum jelly is suitable.", "1", "pc", "Caster cups.", False),
    ]),
]


class Box(Flowable):
    """Empty tick box for the 'received' column."""
    def __init__(self, size=3.4 * mm):
        super().__init__()
        self.size = size

    def wrap(self, *_):
        return self.size, self.size

    def draw(self):
        self.canv.setStrokeColor(INK)
        self.canv.setLineWidth(0.7)
        self.canv.rect(0, 0, self.size, self.size)


class NumberedCanvas(canvas.Canvas):
    """Draws the running header and footer, with 'Page n of N'."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._pages = []

    def showPage(self):
        self._pages.append(dict(self.__dict__))
        self._startPage()

    def save(self):
        total = len(self._pages)
        for state in self._pages:
            self.__dict__.update(state)
            self.frame(total)
            super().showPage()
        super().save()

    def frame(self, total):
        w, h = PAGE
        self.saveState()
        self.setStrokeColor(NAVY)
        self.setLineWidth(0.8)
        self.line(MARGIN, h - 9 * mm, w - MARGIN, h - 9 * mm)
        self.setFont("Arial-Bold", 8)
        self.setFillColor(NAVY)
        self.drawString(MARGIN, h - 7.6 * mm, PROJECT)
        self.setFont("Arial", 8)
        self.drawRightString(w - MARGIN, h - 7.6 * mm,
                             f"Parts to purchase   |   {DOC_NO}   |   Rev. {REV}   |   {DATE}")
        self.setStrokeColor(RULE)
        self.setLineWidth(0.5)
        self.line(MARGIN, 9.5 * mm, w - MARGIN, 9.5 * mm)
        self.setFillColor(GREY)
        self.setFont("Arial", 7.5)
        self.drawString(MARGIN, 6.5 * mm, SCHOOL)
        self.drawRightString(w - MARGIN, 6.5 * mm, f"Page {self._pageNumber} of {total}")
        self.restoreState()


def item_table():
    cols = ["Rcvd", "Item", "Description", "Specification", "Qty", "Unit", "Application", "Rev."]
    w = [10, 11, 55, 96, 15, 11, 65, 10]
    w = [x * WIDTH / sum(w) for x in w]
    data = [[Paragraph(x, head) for x in cols]]
    bands = []
    for k, (name, rows) in enumerate(SECTIONS, start=1):
        bands.append(len(data))
        data.append([Paragraph(f"{k}&nbsp;&nbsp;{name.upper()}", band)] + [""] * (len(cols) - 1))
        for i, (desc, spec, qty, unit, app, changed) in enumerate(rows, start=1):
            data.append([Box(), Paragraph(f"{k}.{i}", cell), Paragraph(desc, cell_b), Paragraph(spec, cell),
                         Paragraph(qty, centre), Paragraph(unit, centre), Paragraph(app, cell),
                         Paragraph(f"<b>{DELTA}</b>" if changed else "", centre)])
    t = Table(data, colWidths=w, repeatRows=1)
    st = [
        ("BACKGROUND", (0, 0), (-1, 0), NAVY),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LINEBELOW", (0, 1), (-1, -1), 0.4, RULE),
        ("BOX", (0, 0), (-1, -1), 0.8, NAVY),
        ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 4),
        ("TOPPADDING", (0, 0), (-1, -1), 2.4), ("BOTTOMPADDING", (0, 0), (-1, -1), 2.4),
        ("TOPPADDING", (0, 1), (0, -1), 3.4),
    ]
    for r in bands:
        st += [("SPAN", (0, r), (-1, r)), ("BACKGROUND", (0, r), (-1, r), BAND),
               ("LINEABOVE", (0, r), (-1, r), 0.6, NAVY)]
    t.setStyle(TableStyle(st))
    return t


def main():
    n_items = sum(len(rows) for _, rows in SECTIONS)
    story = [
        Paragraph("Parts to Purchase", title_s),
        Paragraph(f"{n_items} line items. The 3D-printed parts, the laser-cut plates and the parts already in "
                  "stock are not listed.", sub_s),
        Spacer(1, 4 * mm),
        item_table(),
        Spacer(1, 2 * mm),
        Paragraph(f"Rcvd = tick when received.   {DELTA} = changed for the 6 mm acrylic plates.   pc = piece.", small),
    ]
    doc = SimpleDocTemplate(str(OUT), pagesize=PAGE, leftMargin=MARGIN, rightMargin=MARGIN,
                            topMargin=14 * mm, bottomMargin=14 * mm,
                            title=f"Parts to Purchase ({DOC_NO} Rev. {REV})", author="Cargo robot project",
                            subject=PROJECT)
    doc.build(story, canvasmaker=NumberedCanvas)
    print(f"{n_items} line items -> {OUT}")


if __name__ == "__main__":
    main()
