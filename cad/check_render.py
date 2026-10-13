"""Render PNG previews and check for part-to-part collisions. Run: python check_render.py"""
from itertools import combinations, product
from pathlib import Path

import cadquery as cq
from cadquery.vis import show

import robot_cad as rc

out = Path(__file__).parent / "output" / "views"
out.mkdir(parents=True, exist_ok=True)


def solid(wp):
    return cq.Compound.makeCompound(wp.vals())


def overlaps(pairs):
    """Pairs of parts that share real volume (touching faces are fine)."""
    hits = []
    for (n1, s1), (n2, s2) in pairs:
        b1, b2 = s1.BoundingBox(), s2.BoundingBox()
        if b1.xmax < b2.xmin or b2.xmax < b1.xmin or b1.ymax < b2.ymin or b2.ymax < b1.ymin \
                or b1.zmax < b2.zmin or b2.zmax < b1.zmin:
            continue
        v = s1.intersect(s2).Volume()
        if v > 1.0:
            hits.append((round(v, 1), n1, n2))
    return sorted(hits, reverse=True)


def report(hits):
    for h in hits:
        print("  ", h)
    if not hits:
        print("   none")


# --- collisions with the pusher parked ---
flat = [(name, solid(wp)) for items in rc.groups.values() for name, wp, _ in items]
print(f"{len(flat)} parts, collisions (mm^3, part, part):")
report(overlaps(combinations(flat, 2)))

# --- the pusher must clear everything along its stroke ---
fixed = [(n, s) for n, s in flat if n not in rc.MOVING]
for label, face in (("half-way", (rc.PUSH_HOME + rc.PUSH_END) / 2), ("fully out", rc.PUSH_END)):
    moving = [(n, solid(wp)) for n, wp, _ in rc.pusher_moving(face)]
    print(f"pusher {label} (pushing face at X = {face:.0f}), collisions:")
    report(overlaps(product(moving, fixed)))

# --- all four wheels must touch the floor (z = 0) ---
ground = {n: round(s.BoundingBox().zmin, 2) for n, s in flat if n.startswith(("tire_", "caster_wheel_"))}
print("lowest point of each wheel (must all be 0):", ground)
bb = cq.Compound.makeCompound([s for _, s in flat]).BoundingBox()
print(f"overall size L x W x H: {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm")
pusher_top = max(s.BoundingBox().zmax for n, s in flat if n.startswith("pusher_"))
print(f"highest pusher part: {pusher_top:.1f} mm (the lidar scans at {rc.AXLE_Z + 200:.0f} mm, so it must stay below)")

# --- renders ---
fz = 160
views = {
    "render_iso": dict(position=(900, -900, 750), focus=(0, 0, fz)),
    "render_iso_rear": dict(position=(-900, 900, 750), focus=(0, 0, fz)),
    "render_side": dict(position=(0, -1400, fz), focus=(0, 0, fz)),
    "render_front": dict(position=(1400, 0, fz), focus=(0, 0, fz)),
    "render_top": dict(position=(0, 0, 1500), focus=(0, 0, 0), viewup=(1, 0, 0)),
    "render_pusher_out": dict(position=(-900, -900, 750), focus=(0, 0, fz), push_face=rc.PUSH_END),
}
for name, cam in views.items():
    cam.setdefault("viewup", (0, 0, 1))
    assy = rc.build_assembly(cam.pop("push_face", rc.PUSH_HOME))
    show(assy, screenshot=str(out / f"{name}.png"), interact=False, width=1400, height=1000,
         roll=0, elevation=0, azimuth=0, zoom=1.0, trihedron=False, gradient=False,
         bgcolor=(1, 1, 1), **cam)
    print("wrote", name)
