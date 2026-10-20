"""Collision and printability checks, then PNG renders. Run: python check_render.py"""
from itertools import combinations, product
from pathlib import Path

import cadquery as cq
from cadquery.vis import show

import robot_cad as rc

out = Path(__file__).parent / "output" / "views"
out.mkdir(parents=True, exist_ok=True)


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


# --- collisions with the blade parked ---
flat = [(name, rc.solid(wp)) for items in rc.groups.values() for name, wp, _ in items]
print(f"{len(flat)} parts, collisions (mm^3, part, part):")
report(overlaps(combinations(flat, 2)))

# --- the pusher must clear everything along its stroke ---
fixed = [(n, s) for n, s in flat if n not in rc.MOVING]
for label, face in (("half-way", (rc.PUSH_HOME + rc.PUSH_END) / 2), ("fully out", rc.PUSH_END)):
    moving = [(n, rc.solid(wp)) for n, wp, _ in rc.pusher_moving(face)]
    print(f"blade {label} (pushing face at Y = {face:.0f}), collisions:")
    report(overlaps(product(moving, fixed)))

# --- the belt clamp must stay between the pulleys ---
clamp = [rc.CARR_Y[0] + rc.CLAMP_AT[0], rc.CARR_Y[1] + rc.CLAMP_AT[1]]
print(f"belt clamp travels Y {clamp[0]:.1f} .. {clamp[1]:.1f}; pulley flange from {rc.PULLEY_Y - 8:.1f}, "
      f"idler flange from {rc.IDLER_Y + 9:.1f}:", "ok" if clamp[1] <= rc.PULLEY_Y - 8 and clamp[0] >= rc.IDLER_Y + 9 else "NO")

# --- all four wheels on the floor ---
ground = {n: round(s.BoundingBox().zmin, 2) for n, s in flat if n.startswith(("tire_", "ball_caster_"))}
print("lowest point of each wheel (must all be 0):", ground)
bb = cq.Compound.makeCompound([s for _, s in flat]).BoundingBox()
print(f"overall size L x W x H: {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm "
      f"(X {bb.xmin:.0f}..{bb.xmax:.0f}, Y {bb.ymin:.0f}..{bb.ymax:.0f})")
print(f"blade: {rc.Z_BLADE - rc.BED_TOP:.0f} mm above the bed, stroke {rc.PUSH_HOME - rc.PUSH_END:.0f} mm, "
      f"stops {rc.PUSH_END + rc.W / 2:.0f} mm inside the robot's right side")

# --- printability: each part as printed must fit the printer and stand on a real flat face ---
print(f"printed parts (as printed; printer limit {rc.PRINTER}):")
total = {}
for name, p in rc.printed.items():
    s = rc.solid(rc.print_pose(p["shape"], p["orient"]))
    b = s.BoundingBox()
    fits = (b.xlen <= rc.PRINTER[0] and b.ylen <= rc.PRINTER[1] or b.ylen <= rc.PRINTER[0] and b.xlen <= rc.PRINTER[1]) \
        and b.zlen <= rc.PRINTER[2]
    contact = sum(f.Area() for f in s.Faces() if f.BoundingBox().zmax < 0.01 and f.normalAt().z < -0.99) / 100
    g = rc.grams(s, p["material"]) * p["qty"]
    total[p["material"]] = total.get(p["material"], 0) + g
    flag = "" if fits and contact > 1.0 else "   <-- CHECK"
    print(f"  {name:22s} {p['material']:4s} x{p['qty']}  {b.xlen:5.0f} x {b.ylen:5.0f} x {b.zlen:5.0f}  "
          f"on the bed: {contact:6.1f} cm2{flag}")
print("plastic, approx.:", {m: f"{g / 1000:.2f} kg" for m, g in total.items()})

# --- renders ---
fz = 170
belt = rc.box2(-500, 500, -rc.W / 2 - 40 - 400, -rc.W / 2 - 40, rc.BED_TOP - 4 - 60, rc.BED_TOP - 4)   # a conveyor, 40 mm away
views = {
    "render_iso": dict(position=(950, -950, 800), focus=(0, 0, fz), package=True),
    "render_iso_rear": dict(position=(-950, 950, 800), focus=(0, 0, fz)),
    "render_side": dict(position=(0, -1500, fz), focus=(0, 0, fz)),
    "render_front": dict(position=(1500, 0, fz), focus=(0, 0, fz)),
    "render_top": dict(position=(0, 0, 1600), focus=(0, 0, 0), viewup=(1, 0, 0)),
    "render_layout": dict(position=(0, 0, 1600), focus=(0, 0, 0), viewup=(1, 0, 0), hide=("payload", "pusher", "controls")),
    "render_pusher_out": dict(position=(900, -1300, 900), focus=(0, -200, fz), push_face=rc.PUSH_END, conveyor=True),
}
for name, cam in views.items():
    cam.setdefault("viewup", (0, 0, 1))
    face = cam.pop("push_face", rc.PUSH_HOME)
    hide = cam.pop("hide", ())
    assy = rc.build_assembly(face, package=cam.pop("package", False))
    if hide:
        assy = cq.Assembly(name="layout")
        for gname, items in rc.groups.items():
            if gname not in hide:
                for n, wp, colour in items:
                    assy.add(wp, name=n, color=cq.Color(*colour))
    if cam.pop("conveyor", False):
        shift = rc.PUSH_END - (rc.PUSH_HOME - 5)
        assy.add(rc.PACKAGE.translate((0, shift, 0)), name="package", color=cq.Color(0.72, 0.53, 0.30, 1))
        assy.add(belt, name="conveyor_belt", color=cq.Color(0.25, 0.25, 0.27, 1))
    show(assy, screenshot=str(out / f"{name}.png"), interact=False, width=1400, height=1000,
         roll=0, elevation=0, azimuth=0, zoom=1.0, trihedron=False, gradient=False, bgcolor=(1, 1, 1), **cam)
    print("wrote", name)
