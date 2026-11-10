"""Render PNG previews and check for part-to-part collisions, with the bed flat and tipped.
Run: python check_render.py               (python check_render.py --no-render: checks only)"""
import sys
from itertools import combinations, product
from pathlib import Path

import cadquery as cq
from cadquery.vis import show

import robot_cad as rc

out = Path(__file__).parent / "output" / "views"
out.mkdir(parents=True, exist_ok=True)


def overlap(b1, b2):
    return not (b1.xmax < b2.xmin or b2.xmax < b1.xmin or b1.ymax < b2.ymin or b2.ymax < b1.ymin
                or b1.zmax < b2.zmin or b2.zmax < b1.zmin)


def collisions(pairs):
    """Touching faces are fine; only real overlapping volume is reported."""
    hits = []
    for (n1, s1), (n2, s2) in pairs:
        if overlap(s1.BoundingBox(), s2.BoundingBox()):
            v = s1.intersect(s2).Volume()
            if v > 1.0:
                hits.append((round(v, 1), n1, n2))
    return sorted(hits, reverse=True)


def solid(wp):
    return cq.Compound.makeCompound(wp.vals())


fixed = [(name, solid(wp)) for g, items in rc.groups.items() if g != "payload" for name, wp, _ in items]
bed = [(name, solid(wp)) for name, wp, _ in rc.groups["payload"]]

# --- collisions: fixed parts among themselves, the bed among itself, and the bed against the rest at every angle ---
print(f"{len(fixed) + len(bed)} parts, collisions (mm^3, part, part):")
hits = collisions(combinations(fixed, 2)) + collisions(combinations(bed, 2))
angles = (0, 10, 20, 30, rc.TIP_ANGLE, rc.TIP_ANGLE + 5)
for a in angles:
    turned = [(f"{n} @{a}deg", cq.Compound.makeCompound(rc.tipped(cq.Workplane().add(s), a).vals())) for n, s in bed]
    hits += collisions(product(turned, fixed))
for h in hits:
    print("  ", h)
if not hits:
    print(f"   none (bed checked at {', '.join(str(a) for a in angles)} deg)")

# --- all four wheels must touch the floor (z = 0); everything else must clear it ---
ground = {n: round(s.BoundingBox().zmin, 2) for n, s in fixed if n.startswith(("tire_", "caster_ball_"))}
print("lowest point of each wheel / caster ball (must all be 0):", ground)
low = sorted((round(s.BoundingBox().zmin, 1), n) for n, s in fixed if not n.startswith(("tire_", "caster_ball_")))
print("closest to the floor (mm):", ", ".join(f"{n} {z}" for z, n in low[:6]))
bb = cq.Compound.makeCompound([s for _, s in fixed + bed]).BoundingBox()
print(f"overall size L x W x H: {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm")

# --- every bolt's path: a rod slightly thinner than the bolt must pass through all the parts it joins ---
def vprobe(d, x, y, z0, z1):
    return solid(rc.cyl_z(d, z1 - z0, x, y, z0))


def xprobe(d, x0, x1, y, z):
    return solid(rc.cyl_x(d, x0, x1, y, z))


probes = []
for front in (True, False):
    s = 1 if front else -1
    for hx, hy in rc.servo_mount_holes(front):
        probes.append((f"servo mount bolt ({hx:.0f}, {hy:.0f})", vprobe(3.8, hx, hy, rc.Z_DECK - 3,
                                                                        rc.Z_DECK_TOP + rc.SERVO_FOOT - 0.5)))
    for sy in (-1, 1):
        xs = sorted((s * (rc.X_EAR - 12), s * (rc.X_EAR + rc.MOUNT_T + 3)))
        probes.append((f"servo ear clamp screw ({s * rc.X_EAR:.0f}, {rc.Y_SERVO_MID + sy * rc.CLAMP_DY:.0f})",
                       xprobe(3.8, xs[0], xs[1], rc.Y_SERVO_MID + sy * rc.CLAMP_DY, rc.Z_AX)))
    for ax, ay in rc.ADAPTER_BOLTS:
        probes.append((f"horn adapter bolt ({s * ax:.0f}, {ay:.0f})", vprobe(3.8, s * ax, ay, rc.Z_TILT - 3,
                                                                           rc.Z_TILT_TOP + 8)))
    x_face = rc.TILT_X + rc.ADAPTER_WALL                    # horn screws, M3 x 8: through the horn, into the wall only
    for k in range(4):
        import math
        a = math.radians(45 + 90 * k)
        for r in rc.HORN_SLOTS:
            xs = sorted((s * (x_face - 5.5), s * (x_face - 0.2)))
            probes.append((f"horn screw r{r:g} @{45 + 90 * k}deg ({'front' if front else 'rear'})",
                           xprobe(2.4, xs[0], xs[1], rc.Y_AX + r * math.cos(a), rc.Z_AX + r * math.sin(a))))
for x, y in rc.REST_PTS:
    probes.append((f"bed rest bolt ({x}, {y})", vprobe(3.8, x, y, rc.Z_DECK - 3, rc.Z_DECK_TOP + 11)))
for x, y in rc.riser_holes(bottom=True):
    probes.append((f"riser bolt from below ({x:.0f}, {y:.0f})", vprobe(3.8, x, y, rc.Z_TILT - 3, rc.Z_TILT_TOP + 15)))
for x, y in rc.riser_holes(bottom=False):
    probes.append((f"riser bolt from above ({x:.0f}, {y:.0f})", vprobe(3.8, x, y, rc.Z_CARGO - 14,
                                                                       rc.Z_CARGO + rc.CARGO_T + 3)))
for x, y in rc.LINE_BRKT_HOLES:
    probes.append((f"line sensor bar bolt ({x:.0f}, {y:.0f})", vprobe(2.8, x, y, rc.Z_BASE - 8, rc.Z_LOWER + 3)))
for y in rc.LINE_Y:
    probes.append((f"IR module screw (y {y})", xprobe(2.8, rc.BAR_X - 6, rc.BAR_X + 3, y, rc.IR_HOLE_Z)))
for x, y in rc.SONAR_HOLES:
    probes.append((f"sonar bracket bolt ({x}, {y})", vprobe(2.8, x, y, rc.Z_BASE - 3, rc.Z_LOWER + 6)))
for s in (1, -1):
    probes.append((f"caster M8 bolt (x {s * rc.CASTER_X})", vprobe(7.6, s * rc.CASTER_X, 0, rc.CUP_HEAD[1] - 0.5,
                                                                     rc.Z_LOWER + 10)))
skip = ("caster_bolt", "rod_", "ir_module", "servo_horn")   # modelled hardware / boards without their holes
solids = [(n, s) for n, s in fixed + bed if not n.startswith(skip)]
bad = []
for pname, p in probes:
    for n, s in solids:
        if overlap(p.BoundingBox(), s.BoundingBox()):
            v = p.intersect(s).Volume()
            if v > 0.05:
                bad.append((pname, n, round(v, 2)))
print(f"bolt paths checked: {len(probes)}; blocked:", bad if bad else "none")

# --- printed parts: fit a 200 x 200 mm printer, one piece each, a flat face on the bed ---
print("printed parts, in their printing position (size mm; area on the bed mm^2):")
for name, wp in rc.fabricated.items():
    posed = solid(rc.print_pose(name, wp))
    b = posed.BoundingBox()
    nsol = len(posed.Solids())
    on_bed = sum(f.Area() for f in posed.Faces() if f.BoundingBox().zmax < 0.01)
    flag = "" if (b.xlen <= 200 and b.ylen <= 200 and b.zlen <= 200 and nsol == 1 and on_bed > 50) else "   <-- CHECK"
    print(f"   {name:20s} {b.xlen:5.1f} x {b.ylen:5.1f} x {b.zlen:5.1f}   {on_bed:7.0f}   solids {nsol}{flag}")

# --- the bed when tipped: where the package leaves it ---
edge =cq.Vector(0, -rc.CARGO[1] / 2, rc.Z_CARGO + rc.CARGO_T)        # top of the cargo plate's right edge
for a in (0, rc.TIP_ANGLE):
    p = rc.tipped(cq.Workplane().add(cq.Vertex.makeVertex(*edge.toTuple())), a).val().Center()
    print(f"bed at {a:2d} deg: right edge of the cargo plate at Y = {p.y:.0f} mm, Z = {p.z:.0f} mm above the floor")

# --- servo torque: what the two servos must hold, flat to tipped ---
print(f"torque on the hinge (N.m), 20 cm cube in the middle of the bed; "
      f"the two servos stall at {rc.SERVO_STALL_NM:.1f} N.m:")
for kg in (1, 3, 5):
    row = "   ".join(f"{a:2d} deg {rc.hinge_torque(kg, a):4.1f}" for a in (0, 10, 20))
    print(f"   {kg} kg: {row}   ({100 * rc.hinge_torque(kg) / rc.SERVO_STALL_NM:.0f} % of stall at the start)")

if "--no-render" in sys.argv:
    sys.exit()

# --- renders ---
fz = 160
views = {
    "render_iso": (0, dict(position=(720, -720, 600), focus=(0, 0, fz))),
    "render_iso_rear": (0, dict(position=(-720, 720, 600), focus=(0, 0, fz))),
    "render_side": (0, dict(position=(0, -1400, fz), focus=(0, 0, fz))),
    "render_front": (0, dict(position=(1400, 0, fz), focus=(0, 0, fz))),
    "render_top": (0, dict(position=(0, 0, 1500), focus=(0, 0, 0), viewup=(1, 0, 0))),
    "render_tipped": (rc.TIP_ANGLE, dict(position=(720, -720, 650), focus=(0, 0, fz + 40))),
    "render_tipped_front": (rc.TIP_ANGLE, dict(position=(1400, 0, fz + 60), focus=(0, 0, fz + 60))),
}
for name, (angle, cam) in views.items():
    cam.setdefault("viewup", (0, 0, 1))
    show(rc.build_assembly(angle), screenshot=str(out / f"{name}.png"), interact=False, width=1400, height=1000,
         roll=0, elevation=0, azimuth=0, zoom=1.0, trihedron=False, gradient=False,
         bgcolor=(1, 1, 1), **cam)
    print("wrote", name)
