"""Check the conveyor model against itself and against the robot, and render pictures.

Run from the project folder:   cad\\.venv\\Scripts\\python conveyor\\check_render.py
Needs ../cad/robot_cad.py (the robot's model) for the fit checks and for the picture with the robot.
"""
import sys
from itertools import combinations
from pathlib import Path

import cadquery as cq
from cadquery.vis import show

import conveyor_cad as cv

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "cad"))
import robot_cad as rc  # noqa: E402

out = Path(__file__).parent / "output" / "views"
out.mkdir(parents=True, exist_ok=True)

# --- no two parts may share volume (touching faces are fine) ---
parts = [(n, cv.solid(n)) for n in cv.PARTS]
hits = []
for (n1, s1), (n2, s2) in combinations(parts, 2):
    b1, b2 = s1.BoundingBox(), s2.BoundingBox()
    if b1.xmax < b2.xmin or b2.xmax < b1.xmin or b1.ymax < b2.ymin or b2.ymax < b1.ymin \
            or b1.zmax < b2.zmin or b2.zmax < b1.zmin:
        continue
    v = s1.intersect(s2).Volume()
    if v > 1.0:
        hits.append((round(v, 1), n1, n2))
print(f"{len(parts)} parts, collisions (mm^3, part, part):", sorted(hits, reverse=True) or "none")

# --- does it fit the robot? (the requirements of BUILD_GUIDE.md section 8.7) ---
bed_z = rc.Z_CARGO + rc.CARGO_T                              # top of the robot's cargo plate
lidar_z = rc.AXLE_Z + 200                                    # the lidar's scan plane
rear = -rc.L / 2                                             # the robot's rear plane (its X)
longest = rc.PUSH_HOME - (rc.CARGO_X - rc.CARGO[0] / 2)      # longest package the bed holds
bed_gap = (rc.CARGO_X - rc.CARGO[0] / 2) - rear              # bed edge to the robot's rear plane
touching = [(n, s.BoundingBox()) for items in rc.groups.values() for n, wp, _ in items
            for s in [cq.Compound.makeCompound(wp.vals())] if s.BoundingBox().xmin <= rear + 0.5]
on_face = [n for n, b in touching if b.zmin < cv.NEAR_Z]
checks = [
    ("belt is 5-30 mm below the robot's bed", 5 <= bed_z - cv.BELT_Z <= 30, f"{bed_z - cv.BELT_Z:.1f} mm"),
    ("belt is inside the 180-205 mm the guide asks for", 180 <= cv.BELT_Z <= 205, f"{cv.BELT_Z} mm"),
    ("the robot's rear meets the low side plate", len(on_face) >= 2, f"{len(on_face)} robot parts touch it"),
    ("the low plate is at least 40 cm wide", cv.LENGTH >= 400, f"{cv.LENGTH} mm"),
    ("the pusher blade passes above the low plate and the belt", rc.Z_BLADE > max(cv.NEAR_Z, cv.BELT_Z),
     f"blade from {rc.Z_BLADE:.1f} mm, belt at {cv.BELT_Z} mm"),
    ("a package crosses no more than 30 mm to reach the belt", bed_gap + cv.Y_OUT - cv.BELT_W / 2 <= 30,
     f"{bed_gap + cv.Y_OUT - cv.BELT_W / 2:.0f} mm"),
    ("the longest package stops short of the backstop", longest <= cv.Y_OUT + cv.Y_IN, f"{longest:.0f} of {cv.Y_OUT + cv.Y_IN:.0f} mm"),
    ("the backstop is above the lidar plane", cv.BACKSTOP_Z >= lidar_z + 10, f"{cv.BACKSTOP_Z} mm, lidar at {lidar_z:.0f} mm"),
]
print("fit to the robot:")
for what, ok, detail in checks:
    print(f"  {'ok ' if ok else 'NO '} {what}  ({detail})")

# --- renders ---
cam = dict(interact=False, width=1400, height=1000, roll=0, elevation=0, azimuth=0, zoom=1.0,
           trihedron=False, gradient=False, bgcolor=(1, 1, 1), viewup=(0, 0, 1))
fz = 150
views = {
    "render_iso": dict(position=(-1000, -1300, 900), focus=(0, 30, fz)),
    "render_iso_far": dict(position=(1000, 1300, 900), focus=(0, 30, fz)),
    "render_end": dict(position=(1900, 40, fz), focus=(0, 40, fz)),
    "render_top": dict(position=(0, 40, 2000), focus=(0, 40, 0), viewup=(0, 1, 0)),
}
for name, view in views.items():
    show(cv.build_assembly(), screenshot=str(out / f"{name}.png"), **{**cam, **view})
    print("wrote", name)

# the robot docked against it with the pusher fully out, and a package that has just been pushed onto the belt
X_DOCK = -150                                                # where along the conveyor the robot docks
scene = cq.Assembly(name="robot_at_conveyor")
scene.add(cv.build_assembly(), name="conveyor")
scene.add(rc.build_assembly(rc.PUSH_END), name="robot",
          loc=cq.Location(cq.Vector(X_DOCK, -cv.Y_OUT + rear, 0), cq.Vector(0, 0, 1), -90))
package = cv.box2(X_DOCK - 90, X_DOCK + 90, -cv.Y_OUT, -cv.Y_OUT + 220, cv.BELT_Z, cv.BELT_Z + 120)
scene.add(package, name="package", color=cq.Color(0.72, 0.53, 0.30, 1))
show(scene, screenshot=str(out / "render_with_robot.png"),
     **{**cam, "position": (-1300, -1500, 1100), "focus": (-80, -180, 140)})
print("wrote render_with_robot")
