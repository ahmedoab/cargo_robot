"""Export the robot body (everything except the wheels and caster wheels) as one STL for the ROS model.

Run:   .venv\\Scripts\\python export_ros_mesh.py
Writes robot_ws/src/cargo_bot/meshes/cargo_bot_body.stl in the base_link frame (origin = centre of the
drive axle), in millimetres (the URDF scales it by 0.001).
"""
from pathlib import Path

import cadquery as cq

import robot_cad as rc

OUT = Path(__file__).resolve().parents[1] / "robot_ws" / "src" / "cargo_bot" / "meshes" / "cargo_bot_body.stl"
SKIP = ("tire_", "rim_", "shaft_", "caster_wheel_", "omni_")

shapes = [s for items in rc.groups.values() for name, wp, _ in items if not name.startswith(SKIP) for s in wp.vals()]
body = cq.Compound.makeCompound(shapes).translate(cq.Vector(0, 0, -rc.AXLE_Z))
OUT.parent.mkdir(parents=True, exist_ok=True)
cq.exporters.export(body, str(OUT), tolerance=0.4, angularTolerance=0.4)
print("wrote", OUT, f"({OUT.stat().st_size / 1e6:.1f} MB)")
