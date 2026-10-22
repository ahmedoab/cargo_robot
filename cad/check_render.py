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

# --- all four wheels must touch the floor (z = 0) ---
ground = {n: round(s.BoundingBox().zmin, 2) for n, s in fixed if n.startswith(("tire_", "caster_wheel_"))}
print("lowest point of each wheel (must all be 0):", ground)
bb = cq.Compound.makeCompound([s for _, s in fixed + bed]).BoundingBox()
print(f"overall size L x W x H: {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm")

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
