"""Render PNG previews and check for part-to-part collisions. Run: python check_render.py"""
from itertools import combinations
from pathlib import Path

import cadquery as cq
from cadquery.vis import show

import robot_cad as rc

out = Path(__file__).parent / "output" / "views"
out.mkdir(parents=True, exist_ok=True)

# --- collisions (touching faces are fine; only real overlapping volume is reported) ---
flat = [(name, cq.Compound.makeCompound(wp.vals()))
        for items in rc.groups.values() for name, wp, _ in items]
hits = []
for (n1, s1), (n2, s2) in combinations(flat, 2):
    b1, b2 = s1.BoundingBox(), s2.BoundingBox()
    if b1.xmax < b2.xmin or b2.xmax < b1.xmin or b1.ymax < b2.ymin or b2.ymax < b1.ymin \
            or b1.zmax < b2.zmin or b2.zmax < b1.zmin:
        continue
    v = s1.intersect(s2).Volume()
    if v > 1.0:
        hits.append((round(v, 1), n1, n2))
print(f"{len(flat)} parts, collisions (mm^3, part, part):")
for h in sorted(hits, reverse=True):
    print("  ", h)
if not hits:
    print("   none")

# --- all four wheels must touch the floor (z = 0) ---
ground = {n: round(s.BoundingBox().zmin, 2) for n, s in flat if n.startswith(("tire_", "caster_wheel_"))}
print("lowest point of each wheel (must all be 0):", ground)
bb = cq.Compound.makeCompound([s for _, s in flat]).BoundingBox()
print(f"overall size L x W x H: {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm")

# --- renders ---
assy = rc.build_assembly()
fz = 160
views = {
    "render_iso": dict(position=(900, -900, 750), focus=(0, 0, fz)),
    "render_iso_rear": dict(position=(-900, 900, 750), focus=(0, 0, fz)),
    "render_side": dict(position=(0, -1400, fz), focus=(0, 0, fz)),
    "render_front": dict(position=(1400, 0, fz), focus=(0, 0, fz)),
    "render_top": dict(position=(0, 0, 1500), focus=(0, 0, 0), viewup=(1, 0, 0)),
}
for name, cam in views.items():
    cam.setdefault("viewup", (0, 0, 1))
    show(assy, screenshot=str(out / f"{name}.png"), interact=False, width=1400, height=1000,
         roll=0, elevation=0, azimuth=0, zoom=1.0, trihedron=False, gradient=False,
         bgcolor=(1, 1, 1), **cam)
    print("wrote", name)
