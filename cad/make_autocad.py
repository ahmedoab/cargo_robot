"""
AutoCAD version of the cargo robot design (v2, tilting bed): 3D model + A3 drawing set in one DXF file.

Run:   .venv\\Scripts\\python make_autocad.py
Writes ./autocad/
  CargoRobot_v2.dxf   open it in AutoCAD (File > Open, "Files of type: DXF"), then Save As .dwg
      Model tab   : the 3D model (layers 3D-*) - use Orbit / View Cube, visual style "Shaded"
      Layout tabs : 10 A3 sheets ready to plot (DWG To PDF):
                    01 GENERAL ARRANGEMENT   02 LAYOUT (DECK REMOVED)   03 BOTTOM PLATE   04 TOP DECK
                    05 TILTING PLATE   06 CARGO PLATE   07 TILTING BED   08-09 PRINTED PARTS   10 BILL OF MATERIALS
  CargoRobot_v2_drawings.pdf   the same 10 sheets as a PDF (true A3 scale) - print or share without AutoCAD
  previews/*.png      picture of every sheet

Everything is generated from robot_cad.py, so the drawings always match the model.
"""
import math
import textwrap
from pathlib import Path

import cadquery as cq
import ezdxf
from cadquery.occ_impl.exporters.dxf import DxfDocument
from ezdxf import colors
from ezdxf.enums import TextEntityAlignment as TA
from ezdxf.render import MeshBuilder
from OCP.BRepLib import BRepLib
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape

import robot_cad as rc

OUT = Path(__file__).parent / "autocad"
FRAME = (20, 10, 410, 287)            # ISO 5457 A3 frame (20 mm binding margin)
TB = (230, 10, 410, 62)               # title block
VP_C, VP_S = (215, 148.5), (390, 277)  # paper-space viewport covering the frame
N_SHEETS = 10
PROJECT = "AUTONOMOUS CARGO ROBOT - DESIGN v2"
SCHOOL = "ARAB ACADEMY FOR SCIENCE, TECHNOLOGY & MARITIME TRANSPORT - GRADUATION PROJECT II"
DATE = "22/10/2026"

dd = DxfDocument(dxfversion="AC1032", setup=True, doc_units=ezdxf.units.MM)
doc, msp = dd.document, dd.msp


# ============================== document setup ==============================
def setup_document():
    doc.header["$MEASUREMENT"] = 1
    doc.header["$LUNITS"] = 2
    doc.header["$LUPREC"] = 1
    doc.header["$PSLTSCALE"] = 1
    doc.header["$LTSCALE"] = 1.0
    doc.linetypes.add("CENTER_ISO", pattern=[26.0, 18.0, -3.0, 2.0, -3.0], description="Center ___ . ___")
    doc.linetypes.add("HIDDEN_ISO", pattern=[4.5, 3.0, -1.5], description="Hidden _ _ _ _")
    doc.styles.add("ISO", font="isocpeur.ttf")
    for name, color, lw, lt, plot in [
        ("2D-VISIBLE", 7, 35, "Continuous", True),
        ("2D-HIDDEN", 8, 18, "HIDDEN_ISO", True),
        ("CENTER", 1, 18, "CENTER_ISO", True),
        ("DIMENSIONS", 4, 18, "Continuous", True),
        ("TEXT", 7, 25, "Continuous", True),
        ("BALLOONS", 7, 25, "Continuous", True),
        ("TITLEBLOCK", 7, 50, "Continuous", True),
        ("VIEWPORTS", 8, 13, "Continuous", False),
        ("3D-CHASSIS", 33, 25, "Continuous", True),
        ("3D-DRIVE", 250, 25, "Continuous", True),
        ("3D-ELECTRONICS", 3, 25, "Continuous", True),
        ("3D-SENSORS", 5, 25, "Continuous", True),
        ("3D-CONTROLS", 1, 25, "Continuous", True),
        ("3D-TILT", 150, 25, "Continuous", True),
        ("3D-PAYLOAD", 30, 25, "Continuous", True),
    ]:
        doc.layers.add(name, color=color, lineweight=lw, linetype=lt, plot=plot)


def dim_override(scale):
    return {"dimscale": scale, "dimlfac": 1.0, "dimtsz": 0.0, "dimblk": "", "dimpost": "", "dimdle": 0.0,
            "dimtxt": 2.5, "dimasz": 2.5, "dimexe": 1.25, "dimexo": 0.8, "dimgap": 0.8,
            "dimtad": 1, "dimtih": 0, "dimtoh": 0, "dimdec": 1, "dimzin": 8, "dimdsep": ord("."),
            "dimtxsty": "ISO", "dimclrd": 4, "dimclre": 4, "dimclrt": 7}


# ============================== helpers ==============================
def text(layout, s, p, h=2.5, align=TA.LEFT, layer="TEXT", rot=0.0):
    t = layout.add_text(s, height=h, rotation=rot, dxfattribs={"layer": layer, "style": "ISO"})
    t.set_placement(p, align=align)
    return t


def rect(layout, x0, y0, x1, y1, layer="TITLEBLOCK"):
    layout.add_lwpolyline([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], close=True, dxfattribs={"layer": layer})


def line(layout, p, q, layer="TITLEBLOCK"):
    layout.add_line(p, q, dxfattribs={"layer": layer})


def table(layout, x0, y_top, widths, header, rows, row_h=6.0, h=2.5):
    """Simple grid table in paper space. rows may contain multi-line cells (list of str)."""
    x_edges = [x0]
    for w in widths:
        x_edges.append(x_edges[-1] + w)
    y = y_top
    all_rows = [(header, True)] + [(r, False) for r in rows]
    line(layout, (x_edges[0], y), (x_edges[-1], y))
    for cells, is_head in all_rows:
        n_lines = max(len(c) if isinstance(c, list) else 1 for c in cells)
        rh = row_h + (n_lines - 1) * (h + 1.5)
        for i, c in enumerate(cells):
            lines_ = c if isinstance(c, list) else [c]
            for k, s in enumerate(lines_):
                text(layout, str(s), (x_edges[i] + 1.5, y - 1.8 - h - k * (h + 1.5)), h=h)
        y -= rh
        line(layout, (x_edges[0], y), (x_edges[-1], y), "TITLEBLOCK" if is_head else "TEXT")
    for x in x_edges:
        line(layout, (x, y_top), (x, y))
    return y


def hlr(shape, n, vx):
    """Hidden-line-removed projection seen from direction n (towards the viewer), x-axis vx.
    Returns (visible, hidden) compounds in the drawing plane (or None)."""
    algo = HLRBRep_Algo()
    algo.Add(shape.wrapped)
    algo.Projector(HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(*n), gp_Dir(*vx))))
    algo.Update()
    algo.Hide()
    h = HLRBRep_HLRToShape(algo)

    def comp(parts):
        shapes = []
        for s in parts:
            if s is not None and not s.IsNull():
                BRepLib.BuildCurves3d_s(s, 1e-4)        # HLR edges only carry 2D curves
                shapes.append(cq.Shape.cast(s))
        return cq.Compound.makeCompound(shapes) if shapes else None

    return (comp([h.VCompound(), h.Rg1LineVCompound(), h.OutLineVCompound()]),
            comp([h.HCompound(), h.OutLineHCompound()]))


def add_edges(shape, layer, move=(0.0, 0.0)):
    """Write every edge of a planar (XY) shape as DXF LINE / CIRCLE / ARC / polyline, shifted by move."""
    mx, my = move
    attrs = {"layer": layer}
    for e in shape.Edges():
        kind = e.geomType()
        a, b = e.startPoint(), e.endPoint()
        if kind == "LINE":
            msp.add_line((a.x + mx, a.y + my), (b.x + mx, b.y + my), dxfattribs=attrs)
        elif kind == "CIRCLE":
            c, r = e.arcCenter(), e.radius()
            if (a - b).Length < 1e-6:
                msp.add_circle((c.x + mx, c.y + my), r, dxfattribs=attrs)
                continue
            ang = lambda p: math.degrees(math.atan2(p.y - c.y, p.x - c.x)) % 360.0
            sa, ea, ma = ang(a), ang(b), ang(e.positionAt(0.5))
            if (ma - sa) % 360.0 > (ea - sa) % 360.0:       # DXF arcs run counter-clockwise
                sa, ea = ea, sa
            msp.add_arc((c.x + mx, c.y + my), r, sa, ea, dxfattribs=attrs)
        else:                                               # ellipses, splines: fine polyline
            pts = [e.positionAt(i / 32.0) for i in range(33)]
            msp.add_lwpolyline([(p.x + mx, p.y + my) for p in pts], dxfattribs=attrs)


class View:
    """One orthographic / iso view of a shape, placed on a sheet."""

    def __init__(self, shape, n, vx, hidden=False):
        self.n, self.vx = cq.Vector(*n).normalized(), cq.Vector(*vx).normalized()
        self.vy = self.n.cross(self.vx)
        self.visible, self.hidden = hlr(shape, n, vx)
        self.show_hidden = hidden
        bb = self.visible.BoundingBox()
        self.c2 = ((bb.xmin + bb.xmax) / 2, (bb.ymin + bb.ymax) / 2)
        self.size = (bb.xlen, bb.ylen)
        self.offset = (0.0, 0.0)

    def place(self, sheet, paper_center):
        tx, ty = sheet.model(paper_center)
        self.offset = (tx - self.c2[0], ty - self.c2[1])
        add_edges(self.visible, "2D-VISIBLE", self.offset)
        if self.show_hidden and self.hidden is not None:
            add_edges(self.hidden, "2D-HIDDEN", self.offset)
        self.paper_center, self.sheet = paper_center, sheet
        return self

    def m(self, p):
        """World point -> model-space point of this placed view."""
        v = cq.Vector(*p)
        return (self.offset[0] + v.dot(self.vx), self.offset[1] + v.dot(self.vy))

    def label(self, s, gap=7.0):
        x, y = self.paper_center
        text(self.sheet.psp, s, (x, y - self.size[1] / self.sheet.s / 2 - gap), h=3.0, align=TA.MIDDLE_CENTER)


class Sheet:
    def __init__(self, number, name, title, scale, model_center, dwg_no):
        self.psp = doc.layouts.new(f"{number:02d} {name}")
        self.psp.page_setup(size=(420, 297), margins=(0, 0, 0, 0), units="mm",
                            name="ISO_full_bleed_A3_(420.00_x_297.00_MM)", device="DWG To PDF.pc3")
        self.s, self.mc = scale, model_center
        if scale:
            vp = self.psp.add_viewport(center=VP_C, size=VP_S, view_center_point=model_center,
                                       view_height=VP_S[1] * scale, dxfattribs={"layer": "VIEWPORTS"})
            vp.dxf.flags = vp.dxf.flags | 16384                  # lock the viewport zoom
        title_block(self.psp, title, dwg_no, f"1:{scale}" if scale else "-", number)

    def model(self, p):
        return (self.mc[0] + (p[0] - VP_C[0]) * self.s, self.mc[1] + (p[1] - VP_C[1]) * self.s)

    def paper(self, m):
        return (VP_C[0] + (m[0] - self.mc[0]) / self.s, VP_C[1] + (m[1] - self.mc[1]) / self.s)

    def dim(self, p1, p2, base, vertical=False, text_="<>", dec=1):
        msp.add_linear_dim(base=base, p1=p1, p2=p2, angle=90 if vertical else 0, text=text_,
                           dimstyle="EZDXF", override={**dim_override(self.s), "dimdec": dec},
                           dxfattribs={"layer": "DIMENSIONS"}).render()

    def diameter(self, center, radius, angle=None, location=None):
        msp.add_diameter_dim(center=center, radius=radius, angle=angle, location=location,
                             dimstyle="EZ_RADIUS", override=dim_override(self.s),
                             dxfattribs={"layer": "DIMENSIONS"}).render()

    def note(self, lines, x, y, h=2.5, title="NOTES"):
        text(self.psp, title, (x, y), h=3.0)
        for i, s in enumerate(lines):
            text(self.psp, s, (x, y - 5 - i * (h + 2)), h=h)


def title_block(psp, title, dwg_no, scale, sheet_no):
    x0, y0, x1, y1 = FRAME
    rect(psp, x0, y0, x1, y1)
    tx0, ty0, tx1, ty1 = TB
    rect(psp, tx0, ty0, tx1, ty1)
    for y in (22, 34, 48):
        line(psp, (tx0, y), (tx1, y))
    line(psp, (350, 34), (350, 48))
    cells = [(230, "SCALE", scale), (258, "SHEET", f"{sheet_no} / {N_SHEETS}"), (286, "UNITS", "mm"),
             (310, "PROJECTION", "1st ANGLE"), (346, "DATE", DATE), (376, "DRAWN", "")]
    for i, (x, label, value) in enumerate(cells):
        if i:
            line(psp, (x, ty0), (x, 22))
        nxt = cells[i + 1][0] if i + 1 < len(cells) else tx1
        text(psp, label, (x + 1.2, 19.8), h=1.8)
        text(psp, value, ((x + nxt) / 2, 13.5), h=2.8, align=TA.MIDDLE_CENTER)
    text(psp, "PROJECT", (tx0 + 1.2, 59.5), h=1.8)
    text(psp, PROJECT, ((tx0 + tx1) / 2, 53.5), h=4.2, align=TA.MIDDLE_CENTER)
    text(psp, "TITLE", (tx0 + 1.2, 45.6), h=1.8)
    text(psp, title, ((tx0 + 350) / 2, 39.5), h=4.6, align=TA.MIDDLE_CENTER)
    text(psp, "DWG No.", (351.2, 45.6), h=1.8)
    text(psp, dwg_no, (380, 39.5), h=3.5, align=TA.MIDDLE_CENTER)
    text(psp, SCHOOL, ((tx0 + tx1) / 2, 28), h=2.0, align=TA.MIDDLE_CENTER)


def ground_line(view, x0, x1):
    a, b = view.m((x0, 0, 0)), view.m((x1, 0, 0))
    if view.vx.y:                                    # front view: the floor runs along world Y
        a, b = view.m((0, x0, 0)), view.m((0, x1, 0))
    msp.add_line(a, b, dxfattribs={"layer": "2D-VISIBLE"})


def parts(filter_=None, bed_angle=0.0):
    shapes = []
    for gname, items in rc.groups.items():
        for name, wp, _ in items:
            if filter_ is None or filter_(name):
                shapes.extend((rc.tipped(wp, bed_angle) if gname == "payload" else wp).vals())
    return cq.Compound.makeCompound(shapes)


SIDE, FRONT, PLAN = ((0, -1, 0), (1, 0, 0)), ((1, 0, 0), (0, 1, 0)), ((0, 0, 1), (1, 0, 0))
ISO = ((1, -1, 0.8), (1, 1, 0))
BED_NAMES = {name for name, _, _ in rc.groups["payload"]}


# ============================== 3D model ==============================
def model_3d():
    for gname, items in rc.groups.items():
        for name, wp, color in items:
            shape = cq.Compound.makeCompound(wp.vals())
            verts, tris = shape.tessellate(0.25, 0.3)
            mb = MeshBuilder()
            mb.add_mesh(vertices=[(v.x, v.y, v.z) for v in verts], faces=[tuple(t) for t in tris])
            rgb = tuple(int(c * 255) for c in color[:3])
            mesh = mb.render_mesh(msp, dxfattribs={"layer": "3D-" + gname.upper(), "true_color": colors.rgb2int(rgb)})
            if color[3] < 1:
                mesh.transparency = 1.0 - color[3]
    vport = doc.set_modelspace_vport(height=750, center=(0, 0))
    vport.dxf.direction = (1, -1, 0.8)
    vport.dxf.target = (0, 0, 130)


def balloons(sh, view, items, z):
    """Numbered balloons with leaders; items = [(name, (x, y) on the robot, (dx, dy) balloon offset on paper)]."""
    for i, (name, (x, y), (dx, dy)) in enumerate(items, start=1):
        pc = sh.paper(view.m((x, y, z)))
        pb = (pc[0] + dx, pc[1] + dy)
        sh.psp.add_circle(pc, 0.6, dxfattribs={"layer": "BALLOONS"})
        d = ((pc[0] - pb[0]) ** 2 + (pc[1] - pb[1]) ** 2) ** 0.5
        k = 4.0 / d
        line(sh.psp, (pb[0] + (pc[0] - pb[0]) * k, pb[1] + (pc[1] - pb[1]) * k), pc, "BALLOONS")
        sh.psp.add_circle(pb, 4.0, dxfattribs={"layer": "BALLOONS"})
        text(sh.psp, str(i), pb, h=3.0, align=TA.MIDDLE_CENTER, layer="BALLOONS")


# ============================== sheet 01: general arrangement ==============================
def sheet_ga():
    sh = Sheet(1, "GENERAL ARRANGEMENT", "GENERAL ARRANGEMENT", 5, (3000, 0), "CR-V2-001")
    whole = parts()
    bb = whole.BoundingBox()
    side = View(whole, *SIDE).place(sh, (165, 205))
    front = View(whole, *FRONT).place(sh, (68, 205))
    plan = View(whole, *PLAN).place(sh, (165, 116))
    iso = View(parts(bed_angle=rc.TIP_ANGLE), *ISO).place(sh, (322, 214))
    ground_line(side, bb.xmin - 20, bb.xmax + 20)
    ground_line(front, bb.ymin - 20, bb.ymax + 20)
    s = sh.s
    half_w = rc.WHEEL_Y + rc.WHEEL_W / 2                 # outside of the tyres
    top = rc.Z_CARGO + rc.CARGO_T                        # top of the cargo plate
    # side view
    sh.dim(side.m((-rc.L / 2, 0, rc.Z_LOWER)), side.m((bb.xmax, 0, 30)),
           (0, side.m((0, 0, top))[1] + 9 * s), dec=0)
    x_right = side.m((bb.xmax, 0, 0))[0]
    for k, (px, z) in enumerate([(200, rc.Z_BASE), (200, rc.Z_DECK_TOP), (rc.CARGO[0] / 2, top)]):
        sh.dim(side.m((bb.xmax, 0, 0)), side.m((px, 0, z)), (x_right + (8 + 7 * k) * s, 0), vertical=True, dec=0)
    sh.diameter(side.m((0, 0, rc.AXLE_Z)), rc.WHEEL_D / 2, location=side.m((75, 0, -14)))
    # plan view
    sh.dim(plan.m((-rc.L / 2, -rc.W / 2, 0)), plan.m((rc.L / 2, -rc.W / 2, 0)),
           (0, plan.m((0, -half_w, 0))[1] - 8 * s), dec=0)
    xr = plan.m((bb.xmax, 0, 0))[0]
    sh.dim(plan.m((0, -half_w, 0)), plan.m((0, half_w, 0)), (xr + 8 * s, 0), vertical=True, dec=0)
    sh.dim(plan.m((rc.L / 2, -rc.W / 2, 0)), plan.m((rc.L / 2, rc.W / 2, 0)), (xr + 15 * s, 0), vertical=True, dec=0)
    # front view: overall width and wheel track
    y0 = front.m((0, 0, 0))[1]
    sh.dim(front.m((0, -half_w, rc.AXLE_Z)), front.m((0, half_w, rc.AXLE_Z)), (0, y0 - 8 * s), dec=0)
    sh.dim(front.m((0, -rc.WHEEL_Y, 0)), front.m((0, rc.WHEEL_Y, 0)), (0, y0 - 15 * s), dec=0)
    side.label("SIDE VIEW (from the right)  FRONT -->", gap=9)
    front.label("FRONT VIEW", gap=23)
    plan.label("PLAN VIEW  FRONT -->", gap=16)
    iso.label(f"ISOMETRIC VIEW - BED TIPPED {rc.TIP_ANGLE} deg")
    sh.note([
        "1. All dimensions in mm. Weights and loads in kg.",
        "2. Drive: 2 x JGB37-520 12 V 170 RPM gear motors, wheels D100 x 30.",
        "3. Casters: 2 x 50 mm swivel, M8 threaded stem (height set by nuts).",
        "4. Frame: 6 mm plywood + 5 mm acrylic on 6 x M6 threaded rod.",
        f"5. Bed: tipped {rc.TIP_ANGLE} deg to the right by 2 servos on the hinge line.",
        "6. Payload: 4 x 10 kg load cells under the cargo plate; packages up to 3 kg.",
        "7. ESP32 + 8-ch IR line sensor, 3 x HC-SR04, VL53L0X (right side).",
        "8. Sheets: 01 GA, 02 layout, 03-06 plates, 07 bed, 08-09 prints, 10 BOM.",
    ], 233, 150, h=2.3)


# ============================== sheet 02: layout with the deck removed ==============================
def sheet_layout():
    sh = Sheet(2, "LAYOUT (DECK REMOVED)", "LAYOUT - DECK REMOVED", 2, (4700, 0), "CR-V2-002")
    on_deck = ("top_deck", "estop", "switch", "button", "led_", "tof", "vl53l0x", "servo", "bed_rest")
    body = parts(lambda n: not n.startswith(on_deck) and n not in BED_NAMES)
    plan = View(body, (0, 0, 1), (0, -1, 0)).place(sh, (118, 170))      # front of the robot points UP
    plan.label("PLAN VIEW - DECK AND BED REMOVED (front of robot at the top)", gap=9)
    items = [
        ("3S LiPo battery 5000 mAh (lies across, slides out sideways)", (0, 0), (20, 14)),
        ("BTS7960 motor driver - left motor", (-150, 60), (-16, -12)),
        ("BTS7960 motor driver - right motor", (-150, -60), (16, -12)),
        ("ESP32 DevKit on a screw-terminal breakout (USB to the left)", rc.ESP_POS, (-22, 4)),
        ("I/O board: MCP23017 + resistor dividers", rc.IO_POS, (20, 6)),
        ("Buck converter 12 V -> 5.1 V (ESP32, sensors)", (-60, 85), (-12, 16)),
        ("Buck converter 12 V -> 7.4 V (bed servos)", (-72, -95), (22, 6)),
        ("WAGO power buses (+12 V main / motor / 5 V / GND)", (-85, -20), (16, 10)),
        ("Inline fuse 10 A", (45, -110), (20, 2)),
        ("Buzzer", (-110, 105), (-14, -6)),
        ("3 x HC-SR04 ultrasonic on printed bracket", (196, 0), (8, 18)),
        ("8-channel IR line sensor (under the front edge)", (214, 45), (-16, 12)),
        ("M6 threaded rod (x6)", (60, 135), (-14, 0)),
        ("Drive wheel D100 (motor under the plate)", (0, rc.WHEEL_Y), (-6, -30)),
        ("Motor / encoder cable slot (x2)", (40, -70), (16, -4)),
        ("Battery strap slots (x4)", (31, 45), (-12, 16)),
    ]
    balloons(sh, plan, items, rc.Z_LOWER)
    table(sh.psp, 232, 280, [10, 166], ["No.", "COMPONENT"], [[str(i), n] for i, (n, _, _) in enumerate(items, 1)],
          row_h=6.0, h=2.3)
    sh.note([
        "Small boards are held with VHB tape.",
        "Keep 10 cm slack on every wire that goes up to the deck or the bed.",
        "Wiring: see BUILD_GUIDE.md weeks 2 and 3, and the schematics E01-E04.",
    ], 232, 168, h=2.3)


# ============================== sheets 03-06: plates ==============================
def plate_features(face, purpose):
    feats = []
    for w in face.innerWires():
        edges = w.Edges()
        bb = w.BoundingBox()
        c = ((bb.xmin + bb.xmax) / 2, (bb.ymin + bb.ymax) / 2)
        if len(edges) == 1 and edges[0].geomType() == "CIRCLE":
            d = round(2 * edges[0].radius(), 1)
            feats.append(("HOLE", c, d, purpose.get(("HOLE", d), "")))
        else:
            kind = "SLOT" if any(e.geomType() == "CIRCLE" for e in edges) else "CUTOUT"
            size = (round(bb.xlen, 1), round(bb.ylen, 1))
            feats.append((kind, c, size, purpose.get((kind, tuple(sorted(size))), "")))
    order = {"HOLE": 0, "SLOT": 1, "CUTOUT": 2}
    feats.sort(key=lambda f: (order[f[0]], round(f[1][0], 1), round(f[1][1], 1)))
    return feats


def sheet_plate(number, name, dwg_no, wp, z_mid, notes, model_center, purpose):
    sh = Sheet(number, name, name, 2, model_center, dwg_no)
    face = wp.translate((0, 0, -z_mid)).section().faces().val()
    bb = face.BoundingBox()
    cx, cy = (bb.xmin + bb.xmax) / 2, (bb.ymin + bb.ymax) / 2
    tx, ty = sh.model((128, 176))
    ox, oy = tx - cx, ty - cy
    add_edges(face, "2D-VISIBLE", (ox, oy))
    m = lambda x, y: (ox + x, oy + y)
    s = sh.s
    sh.dim(m(bb.xmin, bb.ymin), m(bb.xmax, bb.ymin), (0, oy + bb.ymin - 10 * s))
    sh.dim(m(bb.xmax, bb.ymin), m(bb.xmax, bb.ymax), (ox + bb.xmax + 10 * s, 0), vertical=True)
    # datum mark at the bottom-left corner
    sh.psp.add_circle(sh.paper(m(bb.xmin, bb.ymin)), 1.5, dxfattribs={"layer": "DIMENSIONS"})
    text(sh.psp, "DATUM 0,0", sh.paper(m(bb.xmin + 2, bb.ymin - 2 * s)), h=2.0, align=TA.TOP_LEFT)
    text(sh.psp, "FRONT -->", sh.paper(m(bb.xmax, bb.ymax + 3 * s)), h=2.5, align=TA.BOTTOM_RIGHT)
    rows, counters = [], {"HOLE": 0, "SLOT": 0, "CUTOUT": 0}
    for kind, (x, y), size, why in plate_features(face, purpose):
        counters[kind] += 1
        fid = {"HOLE": "H", "SLOT": "S", "CUTOUT": "C"}[kind] + str(counters[kind])
        if kind == "HOLE":
            r = size / 2
            for a, b in (((x - r - 2, y), (x + r + 2, y)), ((x, y - r - 2), (x, y + r + 2))):
                msp.add_line(m(*a), m(*b), dxfattribs={"layer": "CENTER"})
            size_s, off = f"D{size:g}", r
        else:
            size_s, off = f"{size[0]:g} x {size[1]:g}", max(size) / 2
            if kind == "SLOT":
                size_s += f" (R{min(size) / 2:g})"
        lx, ly = sh.paper(m(x + off * 0.7 + 1.5, y + off * 0.7 + 1.5))
        text(sh.psp, fid, (lx, ly), h=2.2)
        rows.append([fid, f"{x - bb.xmin:.1f}", f"{y - bb.ymin:.1f}", size_s, why])
    y_end = table(sh.psp, 250, 280, [12, 22, 22, 42, 57], ["ID", "X", "Y", "SIZE", "FOR"], rows, row_h=5.2, h=2.2)
    text(sh.psp, "X, Y = centre of the feature from DATUM 0,0 (H = hole, S = slot, C = cut-out)",
         (250, y_end - 5), h=2.0)
    sh.note(notes, 25, 82, h=2.3)


# ============================== sheet 07: the tilting bed ==============================
def sheet_bed():
    sh = Sheet(7, "TILTING BED", "TILTING BED - SERVO HINGE", 2, (9700, 0), "CR-V2-007")
    keep = ("top_deck", "servo_rear", "servo_mount_rear", "bed_rest")

    def bed_shape(angle):
        """Front view of the deck, the rear servo and the bed (the front servo and its mount left out)."""
        shapes = []
        for gname, items in rc.groups.items():
            for name, wp, _ in items:
                if name in BED_NAMES and not name.endswith("_front"):
                    shapes.extend(rc.tipped(wp, angle).vals())
                elif name.startswith(keep):
                    shapes.extend(wp.vals())
        return cq.Compound.makeCompound(shapes)

    flat = View(bed_shape(0), *FRONT, hidden=True).place(sh, (118, 236))
    tip = View(bed_shape(rc.TIP_ANGLE), *FRONT, hidden=True).place(sh, (318, 186))
    s = sh.s
    ax = (0, rc.Y_AX, rc.Z_AX)
    for v in (flat, tip):                                              # centre mark on the hinge axis
        cx, cy = v.m(ax)
        for a, b in (((cx - 9, cy), (cx + 9, cy)), ((cx, cy - 9), (cx, cy + 9))):
            msp.add_line(a, b, dxfattribs={"layer": "CENTER"})
    deck_edge = (0, -rc.W / 2, rc.Z_DECK_TOP)
    top = rc.Z_CARGO + rc.CARGO_T
    # flat: axis position and heights above the deck
    y_deck = flat.m(deck_edge)[1]
    sh.dim(flat.m(deck_edge), flat.m(ax), (0, y_deck - 8 * s), dec=0)
    sh.dim(flat.m(ax), flat.m((0, 0, rc.Z_AX)), (0, y_deck - 15 * s), dec=0)
    xl = flat.m((0, -rc.W / 2, 0))[0]
    sh.dim(flat.m(deck_edge), flat.m(ax), (xl - 8 * s, 0), vertical=True, dec=0)
    sh.dim(flat.m(deck_edge), flat.m((0, -rc.CARGO[1] / 2, top)), (xl - 16 * s, 0), vertical=True, dec=1)
    flat.label("BED FLAT - FROM THE FRONT (front servo not shown)", gap=26)
    # tipped: angle and where the package leaves the bed
    r = math.radians(rc.TIP_ANGLE)
    edge_flat = cq.Vector(0, -rc.CARGO[1] / 2, top)
    rel = edge_flat - cq.Vector(*ax)
    edge = cq.Vector(0, rc.Y_AX + rel.y * math.cos(r) - rel.z * math.sin(r),
                     rc.Z_AX + rel.y * math.sin(r) + rel.z * math.cos(r))
    p_edge = tip.m(edge.toTuple())
    msp.add_line(p_edge, tip.m((0, edge.y + 150, edge.z)), dxfattribs={"layer": "CENTER"})
    msp.add_angular_dim_cra(center=p_edge, radius=110, start_angle=0, end_angle=rc.TIP_ANGLE, distance=0.1,
                            dimstyle="EZ_CURVED", override={**dim_override(s), "dimdec": 0},
                            dxfattribs={"layer": "DIMENSIONS"}).render()
    xl_t = tip.m((0, edge.y, 0))[0]
    sh.dim(tip.m(deck_edge), p_edge, (xl_t - 8 * s, 0), vertical=True, dec=0)
    tip.label(f"BED TIPPED {rc.TIP_ANGLE} deg - THE PACKAGE SLIDES OFF TO THE RIGHT", gap=22)
    out_y, up_z = rc.W / 2 + edge.y, edge.z - rc.Z_DECK_TOP
    text(sh.psp, f"(the right edge of the cargo plate ends up {-out_y:.0f} mm outside the deck, {up_z:.0f} mm above it)", (318, 90),
         h=2.2, align=TA.MIDDLE_CENTER)
    sh.note([
        "1. Two servos on the deck's right edge share one axis: their shafts ARE the hinge",
        "   (front and rear end of the bed, mirror images). No hinge, screw or lever.",
        "2. A printed horn adapter on each end of the tilting plate screws to the servo's",
        "   aluminium disc horn (4 x M3). The load cells ride on the tilting plate.",
        "3. Flat, the bed rests on 2 printed posts and the servos are switched off.",
        f"4. To unload, the ESP32 turns both servos slowly (20 deg/s) to {rc.TIP_ANGLE} deg, waits",
        "   until the load cells read empty, then lowers the bed and switches them off.",
        "5. No anti-slip mat: a cardboard box slides on acrylic from about 20 deg.",
    ], 25, 184, h=2.2, title="HOW IT WORKS")
    rows = [[f"{kg} kg", f"{rc.hinge_torque(kg):.1f} N.m", f"{100 * rc.hinge_torque(kg) / rc.SERVO_STALL_NM:.0f} %"]
            for kg in (1, 3, 5)]
    text(sh.psp, "TORQUE AT THE START OF THE TIP (both servos, 20 cm box)", (25, 140), h=2.6)
    table(sh.psp, 25, 136, [30, 34, 40], ["PACKAGE", "TORQUE", "OF 2 x 60 kg.cm"], rows, row_h=5.6, h=2.3)
    sh.note([
        "Servo: 60 kg.cm at 7.4 V, metal gears, 180 deg, 25T spline,",
        "about 65 x 30 x 48 mm (e.g. DS5160). Check its tab holes:",
        f"{rc.SERVO['hole_l']} x {rc.SERVO['hole_w']} mm here (SERVO in robot_cad.py).",
        "Supply: 7.4 V from its own 10 A buck converter, after the E-stop.",
        "Signal: front servo GPIO 2, rear servo GPIO 12 (schematic E03).",
        f"Hinge axis: {rc.Z_AX:.0f} mm above the floor, {-rc.Y_AX:.0f} mm right of the centre line.",
    ], 25, 104, h=2.2, title="SERVOS")


# ============================== sheets 08-09: printed parts ==============================
def local(wp):
    shape = cq.Compound.makeCompound(wp.vals())
    bb = shape.BoundingBox()
    return shape.translate(cq.Vector(-bb.xmin, -bb.ymin, -bb.zmin)), bb


def overall(sh, v, b, along, base_gap=7, vertical=False):
    """Overall size of a part (b = its bounding box in local coordinates) along X, Y or Z,
    drawn below the view (horizontal) or to its left (vertical)."""
    q = {"x": (b.xlen, 0, 0), "y": (0, b.ylen, 0), "z": (0, 0, b.zlen)}[along]
    pp, pq = v.m((0, 0, 0)), v.m(q)
    cx, cy = sh.model(v.paper_center)
    if vertical:
        sh.dim(pp, pq, (cx - v.size[0] / 2 - base_gap * sh.s, 0), vertical=True, dec=0)
    else:
        sh.dim(pp, pq, (0, cy - v.size[1] / 2 - base_gap * sh.s), dec=0)


def sheet_printed_1():
    sh = Sheet(8, "PRINTED PARTS 1", "PRINTED PARTS (1/2) - BED", 1, (11000, -1400), "CR-V2-008")
    # servo mount (front; the rear one is its mirror image)
    sm, sbb = local(rc.fabricated["servo_mount_front"])
    b = sm.BoundingBox()
    face = View(sm, (-1, 0, 0), (0, -1, 0), hidden=True).place(sh, (72, 246))   # from the bed side
    side = View(sm, (0, 1, 0), (-1, 0, 0), hidden=True).place(sh, (156, 246))
    plan = View(sm, *PLAN, hidden=True).place(sh, (172, 150))
    View(sm, *ISO).place(sh, (78, 160))
    overall(sh, face, b, "y")
    overall(sh, face, b, "z", vertical=True)
    overall(sh, plan, b, "x")
    overall(sh, plan, b, "y", vertical=True)
    yc = rc.Y_AX - rc.SERVO["shaft"] + rc.SERVO["L"] / 2 - sbb.ymin
    hy = (yc - rc.SERVO["hole_l"] / 2, yc + rc.SERVO["hole_l"] / 2)
    hz = (rc.Z_AX - rc.SERVO["hole_w"] / 2 - sbb.zmin, rc.Z_AX + rc.SERVO["hole_w"] / 2 - sbb.zmin)
    sh.dim(face.m((0, hy[0], hz[1])), face.m((0, hy[1], hz[1])), (0, face.m((0, 0, b.zlen))[1] + 6), dec=0)
    sh.dim(face.m((0, hy[0], hz[0])), face.m((0, hy[0], hz[1])),
           (face.m((0, 0, 0))[0] + 7, 0), vertical=True, dec=0)
    face.label("SERVO MOUNT (front) - FROM THE BED SIDE", gap=15)
    side.label("SIDE", gap=8)
    plan.label("PLAN", gap=15)
    text(sh.psp, "4 x D4.5: servo tabs (M4 + nut).", (25, 116), h=2.3)
    text(sh.psp, "3 x D4.5: bolts to the deck. Rear mount = mirror image.", (25, 111), h=2.3)
    # horn adapter (front)
    ha, _ = local(rc.fabricated["horn_adapter_front"])
    hb = ha.BoundingBox()
    hface = View(ha, (1, 0, 0), (0, 1, 0), hidden=True).place(sh, (258, 246))
    hside = View(ha, (0, 1, 0), (-1, 0, 0), hidden=True).place(sh, (316, 246))
    View(ha, *ISO).place(sh, (372, 246))
    overall(sh, hface, hb, "y")
    overall(sh, hface, hb, "z", vertical=True)
    overall(sh, hside, hb, "x")
    hface.label("HORN ADAPTER (front) - FROM THE SERVO", gap=15)
    text(sh.psp, "4 x D2.8 on a D24 circle: the horn's M3 screws cut their own thread.", (316, 203), h=2.1,
         align=TA.MIDDLE_CENTER)
    text(sh.psp, "2 x D4.5: M4 bolts down through the tilting plate.  Rear = mirror.", (316, 198.5), h=2.1,
         align=TA.MIDDLE_CENTER)
    # bed rest post
    br, _ = local(rc.fabricated["bed_rest"])
    bb_ = br.BoundingBox()
    bside = View(br, *SIDE, hidden=True).place(sh, (262, 165))
    overall(sh, bside, bb_, "x")
    overall(sh, bside, bb_, "z", vertical=True)
    bside.label("BED REST (2 off)", gap=15)
    text(sh.psp, "D4.5 through; M4 nut pocket at the top,", (262, 136), h=2.1, align=TA.MIDDLE_CENTER)
    text(sh.psp, "under the 2 mm rubber pad", (262, 132), h=2.1, align=TA.MIDDLE_CENTER)
    # load-cell spacer
    sp, _ = local(rc.fabricated["load_cell_spacer"])
    splan = View(sp, *PLAN).place(sh, (355, 172))
    sside = View(sp, *SIDE).place(sh, (355, 150))
    zone = rc.CELL_ZONE[1] - rc.CELL_ZONE[0]
    L_, W_ = rc.SPACER_LEN, rc.CELL[1]
    sh.dim(splan.m((0, 0, 0)), splan.m((L_, 0, 0)), (0, splan.m((0, 0, 0))[1] - 6))
    sh.dim(splan.m((L_, 0, 0)), splan.m((L_, W_, 0)), (splan.m((L_, 0, 0))[0] + 6, 0), vertical=True)
    a, bx = (L_ - zone - 5.5) / 2, (L_ + zone + 5.5) / 2
    sh.dim(splan.m((a, W_ / 2, 0)), splan.m((bx, W_ / 2, 0)), (0, splan.m((0, W_, 0))[1] + 5))
    sh.dim(sside.m((L_, 0, 0)), sside.m((L_, 0, 5)), (sside.m((L_, 0, 0))[0] + 6, 0), vertical=True)
    text(sh.psp, "LOAD-CELL SPACER (8 off), slot 5.5 wide", (355, 134), h=2.6, align=TA.MIDDLE_CENTER)
    sh.note([
        "1. Material PETG, 0.2 mm layers, 4 walls. STL files: cad/output/parts/",
        "2. Servo mounts: 1 + 1 (front + rear), 40 % infill, foot flat on the bed.",
        "3. Horn adapters: 1 + 1, 100 % infill, round wall flat on the bed.",
        "4. Bed rests: 2 off, 40 % infill. Spacers: 8 off, 100 % infill, flat.",
    ], 25, 96, h=2.3)


def sheet_printed_2():
    sh = Sheet(9, "PRINTED PARTS 2", "PRINTED PARTS (2/2) - SENSORS", 1, (12000, -1400), "CR-V2-009")
    # line sensor bracket
    lb, _ = local(rc.fabricated["line_sensor_bracket"])
    b = lb.BoundingBox()
    lside = View(lb, *SIDE, hidden=True).place(sh, (62, 240))
    lfront = View(lb, *FRONT, hidden=True).place(sh, (158, 240))
    View(lb, *ISO).place(sh, (318, 238))
    overall(sh, lside, b, "x")
    overall(sh, lside, b, "z", vertical=True)
    overall(sh, lfront, b, "y")
    lside.label("LINE SENSOR BRACKET - SIDE", gap=18)
    lfront.label("FROM THE FRONT", gap=18)
    text(sh.psp, "Flange under the bottom plate: 2 x D3.4, 40 mm apart, 16 mm behind the plate's front edge.",
         (25, 188), h=2.2)
    text(sh.psp, "The sensor board hangs under the shelf: 2 x D3.0 (M3), 100 mm apart.", (25, 183.5), h=2.2)
    # ultrasonic bracket
    so, _ = local(rc.fabricated["sonar_bracket"])
    sb = so.BoundingBox()
    splan = View(so, (0, 0, 1), (0, -1, 0), hidden=True).place(sh, (175, 160))     # front of the robot up
    sfront = View(so, *FRONT).place(sh, (175, 96))
    overall(sh, splan, sb, "y")
    overall(sh, sfront, sb, "z", vertical=True)
    splan.label(f"ULTRASONIC BRACKET - PLAN (front up; side faces turned {rc.SONAR_YAW} deg)", gap=17)
    sfront.label("FROM THE FRONT - windows D16.5 for the transducers", gap=8)
    # side sensor bracket
    tb, _ = local(rc.fabricated["tof_bracket"])
    tbb = tb.BoundingBox()
    tface = View(tb, (0, -1, 0), (1, 0, 0), hidden=True).place(sh, (296, 150))
    tside = View(tb, (-1, 0, 0), (0, -1, 0), hidden=True).place(sh, (340, 150))
    View(tb, *ISO).place(sh, (384, 150))
    overall(sh, tface, tbb, "x")
    overall(sh, tface, tbb, "z", vertical=True)
    overall(sh, tside, tbb, "y")
    tface.label("SIDE SENSOR BRACKET (VL53L0X)", gap=15)
    text(sh.psp, "Hangs under the deck (2 x M3); 10 x 8 window for the sensor.", (340, 110), h=2.1,
         align=TA.MIDDLE_CENTER)
    sh.note([
        "1. Material PETG, 0.2 mm layers, 4 walls, 40 % infill. STL: cad/output/parts/",
        "2. Line sensor bracket: flange flat on the bed, supports under the shelf.",
        "3. Ultrasonic bracket: base flat on the bed, no supports.",
        "4. Side sensor bracket: top flange flat on the bed, no supports.",
    ], 25, 48, h=2.3)


# ============================== sheet 10: bill of materials ==============================
def sheet_bom():
    sh = Sheet(10, "BILL OF MATERIALS", "BILL OF MATERIALS", 0, (0, 0), "CR-V2-010")
    rows = []
    for i, (group, item, qty, how) in enumerate(rc.BOM, start=1):
        rows.append([str(i), group, textwrap.wrap(item, 104), str(qty), textwrap.wrap(how, 26)])
    table(sh.psp, 25, 282, [9, 23, 226, 11, 52], ["No.", "GROUP", "DESCRIPTION", "QTY", "MAKE / BUY"],
          rows, row_h=5.0, h=2.1)


# ============================== main ==============================
def main():
    OUT.mkdir(exist_ok=True)
    (OUT / "previews").mkdir(exist_ok=True)
    setup_document()
    model_3d()
    sheet_ga()
    sheet_layout()
    sheet_plate(3, "BOTTOM PLATE", "CR-V2-003", rc.fabricated["bottom_plate"], rc.Z_BASE + rc.BASE_T / 2, [
        "1. Material: 6 mm birch plywood (or 5 mm aluminium), 1 off.",
        "2. Laser-cut from cad/output/dxf/bottom_plate.dxf. All dimensions in mm, +-0.2.",
        "3. Motor-bracket holes (4 x D4.5) are drilled on assembly - see BUILD_GUIDE week 1.",
        "4. The 2 front holes at Y +-20 hold BOTH sensor brackets (one above, one below).",
    ], (5700, 0), {("HOLE", 6.5): "M6 threaded rod", ("HOLE", 8.5): "caster stem M8",
                   ("HOLE", 3.4): "M3 sensor brackets", ("CUTOUT", (4.0, 22.0)): "battery strap",
                   ("CUTOUT", (12.0, 20.0)): "motor/encoder cables"})
    sheet_plate(4, "TOP DECK", "CR-V2-004", rc.fabricated["top_deck"], rc.Z_DECK + rc.DECK_T / 2, [
        "1. Material: 5 mm clear acrylic (keep the protective film), 1 off.",
        "2. Laser-cut from cad/output/dxf/top_deck.dxf. All dimensions in mm, +-0.2.",
        "3. Every hole is cut by the laser; nothing is drilled by hand.",
    ], (6700, 0), {("HOLE", 6.5): "M6 threaded rod", ("HOLE", 22.5): "E-stop 22 mm",
                   ("HOLE", 12.2): "switch / start button", ("HOLE", 6.2): "5 mm LED clip",
                   ("HOLE", 4.5): "M4: servo mount / bed rest", ("HOLE", 3.4): "M3: side sensor",
                   ("CUTOUT", (12.0, 16.0)): "servo cable", ("CUTOUT", (10.0, 30.0)): "load-cell wires"})
    sheet_plate(5, "TILTING PLATE", "CR-V2-005", rc.fabricated["tilt_plate"], rc.Z_TILT + rc.TILT_T / 2, [
        "1. Material: 6 mm birch plywood, 1 off.",
        "2. Laser-cut from cad/output/dxf/tilt_plate.dxf. All dimensions in mm, +-0.2.",
        "3. The bottom edge here (Y = 0) is the RIGHT edge of the robot: it lies on the hinge axis.",
    ], (7700, 0), {("HOLE", 4.5): "M4 horn adapter", ("SLOT", (5.5, 23.5)): "load cells, fixed ends",
                   ("CUTOUT", (2.5, 4.0)): "cable tie"})
    sheet_plate(6, "CARGO PLATE", "CR-V2-006", rc.fabricated["cargo_plate"], rc.Z_CARGO + rc.CARGO_T / 2, [
        "1. Material: 5 mm clear acrylic, 1 off. No anti-slip mat: the package must slide off.",
        "2. Laser-cut from cad/output/dxf/cargo_plate.dxf. All dimensions in mm, +-0.2.",
        "3. Rests only on the 4 load cells (slots S1-S4).",
    ], (8700, 0), {("SLOT", (5.5, 23.5)): "load cells, loaded ends"})
    sheet_bed()
    sheet_printed_1()
    sheet_printed_2()
    sheet_bom()
    doc.layouts.delete("Layout1")
    doc.layouts.set_active_layout("01 GENERAL ARRANGEMENT")
    doc.header["$TILEMODE"] = 0
    path = OUT / "CargoRobot_v2.dxf"
    doc.saveas(path)
    print("wrote", path)

    export_pdf_and_previews()


def export_pdf_and_previews():
    """All sheets -> one A3 PDF at true 1:1 paper scale (plus a PNG per sheet)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from ezdxf.addons.drawing import Frontend, RenderContext
    from ezdxf.addons.drawing.config import BackgroundPolicy, ColorPolicy, Configuration
    from ezdxf.addons.drawing.matplotlib import MatplotlibBackend
    from matplotlib.backends.backend_pdf import PdfPages

    cfg = Configuration(color_policy=ColorPolicy.BLACK, background_policy=BackgroundPolicy.WHITE)
    pdf_path = OUT / "CargoRobot_v2_drawings.pdf"
    with PdfPages(pdf_path) as pdf:
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
            png = OUT / "previews" / (layout.name.replace(" ", "_").replace("(", "").replace(")", "") + ".png")
            fig.savefig(png, dpi=110)
            plt.close(fig)
            print("sheet", layout.name)
    print("wrote", pdf_path)


if __name__ == "__main__":
    main()
