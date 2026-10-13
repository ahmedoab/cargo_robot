"""
3D model of a small belt conveyor: the drop-off that the cargo robot (design v3) pushes its cargo onto.

FOR SIMULATION AND PICTURES ONLY. It has the right outer shape and the right heights, but it is not a
build design: no bearings, fasteners, belt tensioner or wiring are modelled.

Run:   python conveyor_cad.py          (needs: pip install cadquery)
Writes ./output/
  conveyor.step        the whole conveyor, coloured, one named body per part      (millimetres)
  conveyor.stl         the whole conveyor as one mesh                             (millimetres)
  conveyor.glb         the whole conveyor, coloured                               (METRES, glTF)
  parts/*.stl          frame (everything that stands still), belt, the two rollers, end stop  (millimetres)
  dimensions.csv       the numbers a simulation needs

Axes: X = the way the belt carries a package, Y = across the belt, Z = up.
Origin: on the floor, under the middle of the belt. Units: mm.
The robot backs up against the low side plate (at -Y). The tall side plate (at +Y) is the backstop.
"""
import csv
import math
from pathlib import Path

import cadquery as cq

# ============================== PARAMETERS ==============================
LENGTH = 800                     # side plates, end to end (X)
BELT_W, BELT_T = 300, 2          # belt width and thickness
BELT_Z = 195                     # top of the belt above the floor (the robot's bed is at 211)
BELT_SPEED = 0.10                # m/s, towards +X: what a simulation should move the belt at
ROLLER_D, ROLLER_LEN = 50, 320   # the two rollers
ROLLER_INSET = 50                # roller centres from the ends of the side plates
PLATE_T = 9                      # side plates (plywood)
NEAR_DROP = 4                    # the robot-side plate stops this far below the belt surface
BACKSTOP_Z = 300                 # top of the far plate: the robot's lidar scans at 250, so it sees this wall
BED_T = 9                        # board under the belt's top run
ROD_D = 8                        # tie rods across the frame
END_STOP_H = 60                  # the end stop stands this high above the belt

# ---- derived ----
ROLLER_X = LENGTH / 2 - ROLLER_INSET             # roller centres at X = -350 (idler) and +350 (drive)
ROLLER_Z = BELT_Z - BELT_T - ROLLER_D / 2        # 168
Y_IN = (ROLLER_LEN + 6) / 2                      # inner face of the side plates = 163
Y_OUT = Y_IN + PLATE_T                           # outer face = 172 (at -Y: the face the robot touches)
NEAR_Z = BELT_Z - NEAR_DROP                      # top of the robot-side plate = 191
BED_Z = BELT_Z - BELT_T - 1                      # top of the board under the belt = 192

# ============================== COLOURS ==============================
PLYWOOD = (0.85, 0.70, 0.48, 1)
PVC = (0.72, 0.74, 0.76, 1)
RUBBER = (0.10, 0.10, 0.10, 1)
STEEL = (0.62, 0.63, 0.66, 1)
MOTOR = (0.35, 0.35, 0.38, 1)
PRINT = (1.0, 0.55, 0.1, 1)


# ============================== HELPERS ==============================
def box2(x0, x1, y0, y1, z0, z1):
    """Box given by its lowest and highest corner."""
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def cyl_y(d, y0, y1, cx=0, cz=0):
    """Cylinder along +Y from y0 to y1 (y1 > y0)."""
    return (cq.Workplane("XY").circle(d / 2).extrude(y1 - y0)
            .rotate((0, 0, 0), (1, 0, 0), -90).translate((cx, y0, cz)))


def hex_nut_y(af, y0, y1, cx, cz, hole):
    """Hex nut on a rod that runs along Y: af = width across flats."""
    return (cq.Workplane("XY").polygon(6, af / math.cos(math.pi / 6)).extrude(y1 - y0)
            .cut(cq.Workplane("XY").circle(hole / 2).extrude(y1 - y0))
            .rotate((0, 0, 0), (1, 0, 0), -90).translate((cx, y0, cz)))


# ============================== MODEL ==============================
ROD_XZ = ([(x, BED_Z - BED_T - ROD_D / 2) for x in (-250, 0, 250)]     # three rods carry the board under the belt
          + [(x, 40) for x in (-300, 300)])                             # two near the floor keep the frame square
AXLES = [(-ROLLER_X, ROLLER_Z), (ROLLER_X, ROLLER_Z)]


def side_plate(near):
    """Near (robot side, -Y): low, and nothing goes through to its outer face, so the robot meets a flat wall.
    Far (+Y): tall, the backstop."""
    if near:
        p = box2(-LENGTH / 2, LENGTH / 2, -Y_OUT, -Y_IN, 0, NEAR_Z)
        for x, z in ROD_XZ + AXLES:                                     # blind holes, from the inside
            p = p.cut(cyl_y(ROD_D + 0.4, -Y_OUT + 3, -Y_IN + 1, x, z))
    else:
        p = box2(-LENGTH / 2, LENGTH / 2, Y_IN, Y_OUT, 0, BACKSTOP_Z)
        for x, z in ROD_XZ + AXLES:
            p = p.cut(cyl_y(ROD_D + 0.4, Y_IN - 1, Y_OUT + 1, x, z))
    return p


def tie_rods():
    rods = None
    for x, z in ROD_XZ:
        r = cyl_y(ROD_D, -Y_OUT + 3, Y_OUT + 8, x, z).union(hex_nut_y(13, Y_OUT, Y_OUT + 6.5, x, z, ROD_D))
        rods = r if rods is None else rods.union(r)
    return rods


def roller(x, y_shaft_end):
    """PVC-pipe roller on an 8 mm shaft. The shaft stops inside the near plate and ends at y_shaft_end."""
    return cyl_y(ROLLER_D, -ROLLER_LEN / 2, ROLLER_LEN / 2, x, ROLLER_Z).union(
        cyl_y(ROD_D, -Y_OUT + 3, y_shaft_end, x, ROLLER_Z))


def belt():
    """Endless belt round the two rollers; its top run is the carrying surface at Z = BELT_Z."""
    outer, inner = ROLLER_D + 2 * BELT_T, ROLLER_D
    ring = (cq.Workplane("XZ").slot2D(2 * ROLLER_X + outer, outer).extrude(BELT_W / 2, both=True)
            .cut(cq.Workplane("XZ").slot2D(2 * ROLLER_X + inner, inner).extrude(BELT_W / 2 + 1, both=True)))
    return ring.translate((0, 0, ROLLER_Z))


def motor_bracket():
    """Holds the gear motor on the outside of the far plate, in line with the drive roller."""
    x0, x1, z0, z1 = ROLLER_X - 28, ROLLER_X + 28, ROLLER_Z - 28, ROLLER_Z + 28
    b = (box2(x0, x1, Y_OUT + 23, Y_OUT + 27, z0, z1)
         .union(box2(x0, x0 + 6, Y_OUT, Y_OUT + 23, z0, z1))
         .union(box2(x1 - 6, x1, Y_OUT, Y_OUT + 23, z0, z1)))
    return b.cut(cyl_y(13, Y_OUT + 22, Y_OUT + 28, ROLLER_X, ROLLER_Z))


def end_stop():
    """Board across the end of the belt: packages stop here instead of falling off. Leave it out for a
    conveyor that drops packages off its end."""
    x0, x1 = LENGTH / 2 - PLATE_T, LENGTH / 2
    return (box2(x0, x1, -Y_OUT, Y_IN, BELT_Z + 2, BELT_Z + 2 + END_STOP_H)
            .union(box2(x0, x1, -Y_OUT, -Y_IN, NEAR_Z, BELT_Z + 2)))            # foot on the near plate


# name -> (shape, colour, which mesh file it goes into)
PARTS = {
    "side_plate_near": (side_plate(near=True), PLYWOOD, "frame"),
    "side_plate_far": (side_plate(near=False), PLYWOOD, "frame"),
    "belt_bed": (box2(-ROLLER_X + ROLLER_D / 2 + 5, ROLLER_X - ROLLER_D / 2 - 5, -Y_IN, Y_IN, BED_Z - BED_T, BED_Z),
                 PLYWOOD, "frame"),
    "tie_rods": (tie_rods(), STEEL, "frame"),
    "motor_bracket": (motor_bracket(), PRINT, "frame"),
    "coupler": (cyl_y(19, Y_OUT + 2, Y_OUT + 23, ROLLER_X, ROLLER_Z).cut(cyl_y(8, Y_OUT + 1, Y_OUT + 24, ROLLER_X, ROLLER_Z)),
                STEEL, "frame"),
    "gear_motor": (cyl_y(37, Y_OUT + 27, Y_OUT + 53, ROLLER_X, ROLLER_Z)
                   .union(cyl_y(35, Y_OUT + 53, Y_OUT + 85, ROLLER_X, ROLLER_Z)), MOTOR, "frame"),
    "roller_idler": (roller(-ROLLER_X, Y_OUT), PVC, "roller_idler"),
    "roller_drive": (roller(ROLLER_X, Y_OUT + 23), PVC, "roller_drive"),      # its shaft runs on into the coupler
    "belt": (belt(), RUBBER, "belt"),
    "end_stop": (end_stop(), PLYWOOD, "end_stop"),
}


def solid(name):
    return cq.Compound.makeCompound(PARTS[name][0].vals())


def build_assembly(scale=1.0):
    assy = cq.Assembly(name="conveyor")
    for name, (_, colour, _) in PARTS.items():
        shape = solid(name)
        assy.add(shape.scale(scale) if scale != 1.0 else shape, name=name, color=cq.Color(*colour))
    return assy


def dimensions():
    """(name, value, unit, note): what a simulation needs to know."""
    whole = cq.Compound.makeCompound([solid(n) for n in PARTS]).BoundingBox()
    belt_len = 2 * (2 * ROLLER_X) + math.pi * (ROLLER_D + BELT_T)
    return [
        ("overall_length_x", round(whole.xlen), "mm", "side plates, end to end"),
        ("overall_width_y", round(whole.ylen), "mm", f"from {whole.ymin:.0f} (robot side) to {whole.ymax:.0f} (end of the motor)"),
        ("overall_height_z", round(whole.zlen), "mm", "top of the backstop plate"),
        ("belt_surface_z", BELT_Z, "mm", "top of the belt above the floor"),
        ("belt_width", BELT_W, "mm", "Y from -150 to +150"),
        ("belt_carrying_length", 2 * ROLLER_X, "mm", f"flat top run, X from {-ROLLER_X:.0f} to {ROLLER_X:.0f}"),
        ("belt_loop_length", round(belt_len), "mm", "all the way round both rollers"),
        ("belt_speed", BELT_SPEED, "m/s", "towards +X"),
        ("roller_diameter", ROLLER_D, "mm", "axes along Y at X = -350 and +350, Z = 168"),
        ("roller_speed", round(BELT_SPEED / (math.pi * (ROLLER_D + BELT_T) / 1000) * 60, 1), "rpm", "for that belt speed"),
        ("docking_face_y", -Y_OUT, "mm", "outer face of the robot-side plate: flat, from the floor up"),
        ("docking_face_top_z", NEAR_Z, "mm", "the robot-side plate ends just below the belt"),
        ("belt_edge_from_docking_face", Y_OUT - BELT_W / 2, "mm", "the gap a package has to cross"),
        ("backstop_face_y", Y_IN, "mm", "inner face of the tall plate: packages stop against it"),
        ("backstop_top_z", BACKSTOP_Z, "mm", "above the robot's lidar plane (250 mm)"),
        ("end_stop_face_x", LENGTH / 2 - PLATE_T, "mm", "packages stop here at the end of the belt"),
    ]


def main():
    out = Path(__file__).parent / "output"
    (out / "parts").mkdir(parents=True, exist_ok=True)

    build_assembly().export(str(out / "conveyor.step"))
    build_assembly(scale=0.001).export(str(out / "conveyor.glb"), tolerance=0.0001, angularTolerance=0.2)
    mesh = dict(tolerance=0.1, angularTolerance=0.2)
    cq.exporters.export(cq.Compound.makeCompound([solid(n) for n in PARTS]), str(out / "conveyor.stl"), **mesh)
    for group in sorted({g for _, _, g in PARTS.values()}):
        shapes = [solid(n) for n, (_, _, g) in PARTS.items() if g == group]
        cq.exporters.export(cq.Compound.makeCompound(shapes), str(out / "parts" / f"{group}.stl"), **mesh)
    print("wrote conveyor.step, conveyor.stl, conveyor.glb and", len({g for _, _, g in PARTS.values()}), "part meshes")

    with open(out / "dimensions.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Name", "Value", "Unit", "Note"])
        w.writerows(dimensions())
    for name, value, unit, note in dimensions():
        print(f"  {name:30s} {value:>7} {unit:4s} {note}")


if __name__ == "__main__":
    main()
