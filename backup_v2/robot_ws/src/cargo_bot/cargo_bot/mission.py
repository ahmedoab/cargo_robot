"""Delivery mission: home -> pickup (wait for cargo) -> dropoff (wait until unloaded) -> home.

Needs Nav2 running with a map (see BUILD_GUIDE.md, week 8) and a stations file:

    home:    [x, y, yaw_degrees]
    pickup:  [x, y, yaw_degrees]
    dropoff: [x, y, yaw_degrees]

Run:  ros2 run cargo_bot mission --stations ~/maps/stations.yaml [--loops 3] [--set-initial-pose]
"""
import argparse
import math
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Float32


def make_pose(nav, x, y, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class Mission:
    def __init__(self, nav, stations, load_kg, hold_s):
        self.nav, self.stations, self.load_kg, self.hold_s = nav, stations, load_kg, hold_s
        self.weight = None
        nav.create_subscription(Float32, "payload_weight", self.on_weight, 10)

    def on_weight(self, msg):
        self.weight = msg.data

    def go(self, name):
        x, y, yaw = self.stations[name]
        self.nav.get_logger().info(f"-> going to {name} ({x:.2f}, {y:.2f})")
        self.nav.goToPose(make_pose(self.nav, x, y, yaw))
        while not self.nav.isTaskComplete():
            rclpy.spin_once(self.nav, timeout_sec=0.1)
        result = self.nav.getResult()
        if result != TaskResult.SUCCEEDED:
            raise RuntimeError(f"navigation to {name} failed: {result}")
        self.nav.get_logger().info(f"arrived at {name}")

    def wait_weight(self, loaded):
        """Block until the bed has been loaded (or emptied) for hold_s seconds in a row."""
        what = "cargo to be LOADED" if loaded else "cargo to be REMOVED"
        self.nav.get_logger().info(f"waiting for {what} ...")
        since = None
        while True:
            rclpy.spin_once(self.nav, timeout_sec=0.1)
            if self.weight is None:
                continue
            ok = self.weight > self.load_kg if loaded else self.weight < self.load_kg / 2.0
            if not ok:
                since = None
            elif since is None:
                since = time.monotonic()
            elif time.monotonic() - since > self.hold_s:
                self.nav.get_logger().info(f"payload = {self.weight:.2f} kg")
                return

    def run(self, loops):
        for i in range(loops):
            self.nav.get_logger().info(f"=== delivery {i + 1}/{loops} ===")
            self.go("pickup")
            self.wait_weight(loaded=True)
            self.go("dropoff")
            self.wait_weight(loaded=False)
        self.go("home")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stations", required=True, help="YAML file with home / pickup / dropoff")
    parser.add_argument("--loops", type=int, default=1)
    parser.add_argument("--load-kg", type=float, default=0.3, help="weight that counts as 'loaded'")
    parser.add_argument("--hold", type=float, default=2.0, help="seconds the weight must be stable")
    parser.add_argument("--set-initial-pose", action="store_true", help="tell AMCL the robot is at 'home'")
    args, _ = parser.parse_known_args()

    with open(args.stations) as f:
        stations = yaml.safe_load(f)
    for key in ("home", "pickup", "dropoff"):
        if key not in stations:
            sys.exit(f"stations file is missing '{key}'")

    rclpy.init()
    nav = BasicNavigator()
    if args.set_initial_pose:
        nav.setInitialPose(make_pose(nav, *stations["home"]))
    nav.waitUntilNav2Active()
    mission = Mission(nav, stations, args.load_kg, args.hold)
    try:
        mission.run(args.loops)
        nav.get_logger().info("mission complete")
    except (RuntimeError, KeyboardInterrupt) as e:
        nav.cancelTask()
        nav.get_logger().error(str(e) or "mission cancelled")
    finally:
        nav.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
