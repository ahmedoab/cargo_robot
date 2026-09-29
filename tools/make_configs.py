#!/usr/bin/env python3
"""Creates config/nav2_params.yaml and config/slam.yaml for this robot, starting from the default
files installed with ROS 2 Jazzy (so they always match your installed Nav2 version).

Run on the Pi after installing Nav2 + slam_toolbox, then rebuild the workspace:
    python3 tools/make_configs.py
"""
from pathlib import Path

import yaml

SHARE = Path("/opt/ros/jazzy/share")
OUT = Path(__file__).resolve().parents[1] / "robot_ws" / "src" / "cargo_bot" / "config"

FOOTPRINT = "[[0.235, 0.19], [0.235, -0.19], [-0.215, -0.19], [-0.215, 0.19]]"   # metres, incl. wheels
MAX_V = 0.35          # m/s (speed_limiter slows the robot further when it is loaded)
MAX_W = 1.0           # rad/s
SCAN = "/scan_filtered"

# Regulated Pure Pursuit: simple, light on the CPU, never reverses (the cargo blocks the rear view)
RPP = {
    "plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    "desired_linear_vel": MAX_V,
    "lookahead_dist": 0.6,
    "min_lookahead_dist": 0.3,
    "max_lookahead_dist": 0.9,
    "lookahead_time": 1.5,
    "rotate_to_heading_angular_vel": 0.8,
    "transform_tolerance": 0.2,
    "use_velocity_scaled_lookahead_dist": False,
    "min_approach_linear_velocity": 0.05,
    "approach_velocity_scaling_dist": 0.6,
    "use_collision_detection": True,
    "max_allowed_time_to_collision_up_to_carrot": 1.0,
    "use_regulated_linear_velocity_scaling": True,
    "use_cost_regulated_linear_velocity_scaling": False,
    "regulated_linear_scaling_min_radius": 0.9,
    "regulated_linear_scaling_min_speed": 0.25,
    "use_rotate_to_heading": True,
    "rotate_to_heading_min_angle": 0.785,
    "allow_reversing": False,
    "max_angular_accel": 2.0,
    "max_robot_pose_search_dist": 10.0,
}


def use_filtered_scan(node):
    """Point every lidar subscription at /scan_filtered (robot body + cargo removed)."""
    if isinstance(node, dict):
        for key, value in node.items():
            if key in ("topic", "scan_topic") and value in ("scan", "/scan"):
                node[key] = SCAN
            else:
                use_filtered_scan(value)
    elif isinstance(node, list):
        for value in node:
            use_filtered_scan(value)


def nav2():
    p = yaml.safe_load((SHARE / "nav2_bringup" / "params" / "nav2_params.yaml").read_text())
    use_filtered_scan(p)
    for costmap in ("local_costmap", "global_costmap"):
        prm = p[costmap][costmap]["ros__parameters"]
        prm.pop("robot_radius", None)
        prm["footprint"] = FOOTPRINT
    p["controller_server"]["ros__parameters"]["FollowPath"] = RPP
    vs = p["velocity_smoother"]["ros__parameters"]
    vs["max_velocity"] = [MAX_V, 0.0, MAX_W]
    vs["min_velocity"] = [0.0, 0.0, -MAX_W]
    vs["max_accel"] = [0.5, 0.0, 1.5]          # gentle: the cargo must not slide
    vs["max_decel"] = [-0.8, 0.0, -2.0]
    p["amcl"]["ros__parameters"]["laser_max_range"] = 12.0
    (OUT / "nav2_params.yaml").write_text(yaml.safe_dump(p, sort_keys=False))
    print("wrote", OUT / "nav2_params.yaml")


def slam():
    p = yaml.safe_load((SHARE / "slam_toolbox" / "config" / "mapper_params_online_async.yaml").read_text())
    prm = p["slam_toolbox"]["ros__parameters"]
    prm["scan_topic"] = SCAN
    prm["max_laser_range"] = 12.0
    prm["mode"] = "mapping"
    (OUT / "slam.yaml").write_text(yaml.safe_dump(p, sort_keys=False))
    print("wrote", OUT / "slam.yaml")


if __name__ == "__main__":
    nav2()
    slam()
    print("now rebuild:  cd ~/cargo_robot/robot_ws && colcon build --symlink-install")
