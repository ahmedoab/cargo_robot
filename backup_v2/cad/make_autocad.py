"""
AutoCAD version of the cargo robot design (v2): 3D model + A3 drawing set in one DXF file.

Run:   .venv\\Scripts\\python make_autocad.py
Writes ./autocad/
  CargoRobot_v2.dxf   open it in AutoCAD (File > Open, "Files of type: DXF"), then Save As .dwg
      Model tab   : the 3D model (layers 3D-*) - use Orbit / View Cube, visual style "Shaded"
      Layout tabs : 7 A3 sheets ready to plot (DWG To PDF):
                    01 GENERAL ARRANGEMENT   02 LAYOUT (DECK REMOVED)   03 BOTTOM PLATE
                    04 TOP DECK   05 CARGO PLATE   06 PRINTED PARTS   07 BILL OF MATERIALS
  CargoRobot_v2_drawings.pdf   the same 7 sheets as a PDF (true A3 scale) - print or share without AutoCAD
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
N_SHEETS = 7
PROJECT = "AUTONOMOUS CARGO ROBOT - DESIGN v2"
SCHOOL = "ARAB ACADEMY FOR SCIENCE, TECHNOLOGY & MARITIME TRANSPORT - GRADUATION PROJECT II"
DATE = "28/09/2026"

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


def parts(filter_=None):
    shapes = []
    for items in rc.groups.values():
        for name, wp, _ in items:
            if filter_ is None or filter_(name):
                shapes.extend(wp.vals())
    return cq.Compound.makeCompound(shapes)


SIDE, FRONT, PLAN = ((0, -1, 0), (1, 0, 0)), ((1, 0, 0), (0, 1, 0)), ((0, 0, 1), (1, 0, 0))
ISO = ((1, -1, 0.8), (1, 1, 0))


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


# ============================== sheet 01: general arrangement ==============================
def sheet_ga():
    sh = Sheet(1, "GENERAL ARRANGEMENT", "GENERAL ARRANGEMENT", 5, (3000, 0), "CR-V2-001")
    whole = parts()
    bb = whole.BoundingBox()
    side = View(whole, *SIDE).place(sh, (165, 205))
    front = View(whole, *FRONT).place(sh, (68, 205))
    plan = View(whole, *PLAN).place(sh, (165, 116))
    iso = View(whole, *ISO).place(sh, (322, 214))
    ground_line(side, bb.xmin - 20, bb.xmax + 20)
    ground_line(front, bb.ymin - 20, bb.ymax + 20)
    s = sh.s
    half_w = rc.WHEEL_Y + rc.WHEEL_W / 2                 # outside of the tyres
    top = rc.Z_DECK_TOP + 80                             # top of the lidar
    # side view
    sh.dim(side.m((-rc.L / 2, 0, rc.Z_LOWER)), side.m((bb.xmax, 0, 89)),
           (0, side.m((0, 0, top))[1] + 9 * s), dec=0)
    x_right = side.m((bb.xmax, 0, 0))[0]
    for k, (px, z) in enumerate([(200, rc.Z_BASE), (200, rc.Z_DECK_TOP), (rc.LIDAR_X + 13, top)]):
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
    iso.label("ISOMETRIC VIEW")
    sh.note([
        "1. All dimensions in mm. Weights and loads in kg.",
        "2. Drive: 2 x JGB37-520 12 V 170 RPM gear motors, wheels D100 x 30.",
        "3. Casters: 2 x 50 mm swivel, M8 threaded stem (height set by nuts).",
        "4. Frame: 6 mm plywood + 5 mm acrylic on 6 x M6 threaded rod.",
        "5. Payload: 4 x 10 kg load cells under the cargo plate, max 15 kg.",
        "6. Sensors: RPLIDAR A1M8 (front), USB camera tilted 35 deg down.",
        "7. Sheets: 01 GA, 02 layout, 03-05 plates, 06 printed parts, 07 BOM.",
    ], 233, 150, h=2.3)


# ============================== sheet 02: layout with the deck removed ==============================
def sheet_layout():
    sh = Sheet(2, "LAYOUT (DECK REMOVED)", "LAYOUT - DECK REMOVED", 2, (4700, 0), "CR-V2-002")
    hidden_parts = ("top_deck", "estop", "switch", "rplidar", "lidar", "load_cell", "cell_spacer", "hx711", "cargo")
    body = parts(lambda n: not n.startswith(hidden_parts))
    plan = View(body, (0, 0, 1), (0, -1, 0)).place(sh, (118, 172))      # front of the robot points UP
    plan.label("PLAN VIEW - DECK, CARGO PLATE AND LIDAR REMOVED (front of robot at the top)", gap=9)
    items = [
        ("3S LiPo battery 5000 mAh (lies across, slides out sideways)", (0, 0), (18, 10)),
        ("BTS7960 motor driver - left motor", (-150, 60), (-16, -12)),
        ("BTS7960 motor driver - right motor", (-150, -60), (16, -12)),
        ("Raspberry Pi 5 on M2.5 standoffs", (120, 65), (-18, 10)),
        ("Buck converter 12 V -> 5.1 V", (-60, 85), (-16, 8)),
        ("WAGO power buses (+12 V main / motor / GND)", (-60, -85), (16, 8)),
        ("Inline fuse 10 A", (45, -110), (18, 0)),
        ("USB camera on printed mount", (212, 0), (14, 6)),
        ("M6 threaded rod (x6)", (60, 135), (-14, 0)),
        ("Drive wheel D100 (motor under the plate)", (0, rc.WHEEL_Y), (0, -34)),
        ("Motor / encoder cable slot (x2)", (40, -70), (14, -8)),
        ("Battery strap slots (x4)", (31, 45), (-12, 14)),
    ]
    for i, (name, (x, y), (dx, dy)) in enumerate(items, start=1):
        pc = sh.paper(plan.m((x, y, rc.Z_LOWER)))
        pb = (pc[0] + dx, pc[1] + dy)
        sh.psp.add_circle(pc, 0.6, dxfattribs={"layer": "BALLOONS"})
        d = ((pc[0] - pb[0]) ** 2 + (pc[1] - pb[1]) ** 2) ** 0.5
        k = 4.0 / d
        line(sh.psp, (pb[0] + (pc[0] - pb[0]) * k, pb[1] + (pc[1] - pb[1]) * k), pc, "BALLOONS")
        sh.psp.add_circle(pb, 4.0, dxfattribs={"layer": "BALLOONS"})
        text(sh.psp, str(i), pb, h=3.0, align=TA.MIDDLE_CENTER, layer="BALLOONS")
    table(sh.psp, 232, 280, [10, 166], ["No.", "COMPONENT"], [[str(i), n] for i, (n, _, _) in enumerate(items, 1)],
          row_h=6.5, h=2.4)
    sh.note([
        "Small boards are held with VHB tape; the Pi on M2.5 standoffs.",
        "Keep 10 cm slack on every wire that goes up to the deck.",
        "Wiring: see BUILD_GUIDE.md, weeks 2 and 3.",
    ], 232, 184, h=2.3)


# ============================== sheets 03-05: plates ==============================
PURPOSE = {
    ("HOLE", 6.5): "M6 threaded rod", ("HOLE", 8.5): "caster stem M8", ("HOLE", 2.7): "Raspberry Pi M2.5",
    ("HOLE", 3.4): "camera mount M3", ("HOLE", 22.5): "E-stop 22 mm", ("HOLE", 12.2): "master switch",
    ("SLOT", (5.5, 23.5)): "load-cell screws", ("CUTOUT", (4.0, 22.0)): "battery strap",
    ("CUTOUT", (12.0, 20.0)): "motor/encoder cables", ("CUTOUT", (12.0, 80.0)): "wiring pass-through",
}


def plate_features(face):
    feats = []
    for w in face.innerWires():
        edges = w.Edges()
        bb = w.BoundingBox()
        c = ((bb.xmin + bb.xmax) / 2, (bb.ymin + bb.ymax) / 2)
        if len(edges) == 1 and edges[0].geomType() == "CIRCLE":
            d = round(2 * edges[0].radius(), 1)
            feats.append(("HOLE", c, d, PURPOSE.get(("HOLE", d), "")))
        else:
            kind = "SLOT" if any(e.geomType() == "CIRCLE" for e in edges) else "CUTOUT"
            size = (round(bb.xlen, 1), round(bb.ylen, 1))
            feats.append((kind, c, size, PURPOSE.get((kind, tuple(sorted(size))), "")))
    order = {"HOLE": 0, "SLOT": 1, "CUTOUT": 2}
    feats.sort(key=lambda f: (order[f[0]], round(f[1][0], 1), round(f[1][1], 1)))
    return feats


def sheet_plate(number, name, dwg_no, wp, z_mid, notes, model_center):
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
    for kind, (x, y), size, purpose in plate_features(face):
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
        rows.append([fid, f"{x - bb.xmin:.1f}", f"{y - bb.ymin:.1f}", size_s, purpose])
    y_end = table(sh.psp, 250, 280, [12, 22, 22, 42, 57], ["ID", "X", "Y", "SIZE", "FOR"], rows, row_h=5.6, h=2.3)
    text(sh.psp, "X, Y = centre of the feature from DATUM 0,0 (H = hole, S = slot, C = cut-out)",
         (250, y_end - 5), h=2.0)
    sh.note(notes, 25, 82, h=2.3)


# ============================== sheet 06: printed parts ==============================
def local(wp):
    shape = cq.Compound.makeCompound(wp.vals())
    bb = shape.BoundingBox()
    return shape.translate(cq.Vector(-bb.xmin, -bb.ymin, -bb.zmin)), bb


def sheet_printed():
    sh = Sheet(6, "PRINTED PARTS", "PRINTED PARTS", 1, (6400, -1400), "CR-V2-006")
    cm, cbb = local(rc.fabricated["camera_mount"])
    side = View(cm, *SIDE, hidden=True).place(sh, (118, 218))
    front = View(cm, *FRONT, hidden=True).place(sh, (48, 218))
    plan = View(cm, *PLAN, hidden=True).place(sh, (118, 150))
    View(cm, *ISO).place(sh, (190, 215))
    b = cm.BoundingBox()
    sh.dim(plan.m((0, 0, 0)), plan.m((b.xlen, 0, 0)), (0, plan.m((0, 0, 0))[1] - 8))
    sh.dim(plan.m((b.xlen, 0, 0)), plan.m((b.xlen, b.ylen, 0)), (plan.m((b.xlen, 0, 0))[0] + 8, 0), vertical=True)
    sh.dim(side.m((b.xlen, 0, 0)), side.m((b.xlen, 0, b.zlen)), (side.m((b.xlen, 0, 0))[0] + 8, 0), vertical=True)
    hx, hy = rc.CAM_FOOT_HOLES[0][0] - cbb.xmin, rc.CAM_FOOT_HOLES[0][1] - cbb.ymin
    hy2 = rc.CAM_FOOT_HOLES[1][1] - cbb.ymin
    sh.dim(plan.m((0, hy, 0)), plan.m((hx, hy, 0)), (0, plan.m((0, b.ylen, 0))[1] + 6))
    sh.dim(plan.m((hx, hy, 0)), plan.m((hx, hy2, 0)), (plan.m((0, 0, 0))[0] - 7, 0), vertical=True)
    sh.diameter(plan.m((hx, hy2, 0)), 1.7, 135)
    side.label("CAMERA MOUNT - SIDE", gap=8)
    front.label("FRONT", gap=8)
    plan.label("PLAN", gap=16)
    text(sh.psp, "Holder face tilted 35 deg down. Drill 2 mm holes to suit the camera board.",
         (25, 104), h=2.3)

    sp, _ = local(rc.fabricated["load_cell_spacer"])
    splan = View(sp, *PLAN).place(sh, (318, 236))
    sside = View(sp, *SIDE).place(sh, (318, 206))
    zone = rc.CELL_ZONE[1] - rc.CELL_ZONE[0]
    L_, W_ = rc.SPACER_LEN, rc.CELL[1]
    sh.dim(splan.m((0, 0, 0)), splan.m((L_, 0, 0)), (0, splan.m((0, 0, 0))[1] - 6))
    sh.dim(splan.m((L_, 0, 0)), splan.m((L_, W_, 0)), (splan.m((L_, 0, 0))[0] + 6, 0), vertical=True)
    a, bx = (L_ - zone - 5.5) / 2, (L_ + zone + 5.5) / 2
    sh.dim(splan.m((a, W_ / 2, 0)), splan.m((bx, W_ / 2, 0)), (0, splan.m((0, W_, 0))[1] + 5))
    sh.dim(sside.m((L_, 0, 0)), sside.m((L_, 0, 5)), (sside.m((L_, 0, 0))[0] + 6, 0), vertical=True)
    text(sh.psp, "LOAD-CELL SPACER (8 off), slot 5.5 wide", (318, 192), h=3.0, align=TA.MIDDLE_CENTER)
    sh.note([
        "1. Material PETG, 0.2 mm layers, 4 walls.",
        "2. Camera mount: 1 off, 40 % infill, supports on build plate only.",
        "3. Load-cell spacer: 8 off, 100 % infill, print flat.",
        "4. STL files: cad/output/parts/",
    ], 25, 96, h=2.3)


# ============================== sheet 07: bill of materials ==============================
def sheet_bom():
    sh = Sheet(7, "BILL OF MATERIALS", "BILL OF MATERIALS", 0, (0, 0), "CR-V2-007")
    rows = []
    for i, (group, item, qty, how) in enumerate(rc.BOM, start=1):
        rows.append([str(i), group, textwrap.wrap(item, 92), str(qty), textwrap.wrap(how, 30)])
    table(sh.psp, 25, 280, [10, 24, 205, 14, 70], ["No.", "GROUP", "DESCRIPTION", "QTY", "MAKE / BUY"],
          rows, row_h=6.2, h=2.3)


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
    ], (5700, 0))
    sheet_plate(4, "TOP DECK", "CR-V2-004", rc.fabricated["top_deck"], rc.Z_DECK + rc.DECK_T / 2, [
        "1. Material: 5 mm clear acrylic (keep the protective film), 1 off.",
        "2. Laser-cut from cad/output/dxf/top_deck.dxf. All dimensions in mm, +-0.2.",
        "3. Lidar holes (4 x D2.7) are drilled on assembly, using the lidar as template.",
    ], (6700, 0))
    sheet_plate(5, "CARGO PLATE", "CR-V2-005", rc.fabricated["cargo_plate"], rc.Z_CARGO + rc.CARGO_T / 2, [
        "1. Material: 5 mm clear acrylic, 1 off. Anti-slip mat on top.",
        "2. Laser-cut from cad/output/dxf/cargo_plate.dxf. All dimensions in mm, +-0.2.",
        "3. Rests only on the 4 load cells (slots S1-S4).",
    ], (7700, 0))
    sheet_printed()
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
