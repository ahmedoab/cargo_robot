"""Delivery mission: home -> pickup (wait for cargo) -> dropoff (unload) -> home.

At the drop-off the cargo is unloaded either by hand (the default: the robot waits until the bed is empty)
or, with --conveyor, by the robot itself: it backs up until its rear touches the conveyor, pushes the cargo
off with the pusher (pusher_node must be running), checks that the bed is empty and drives clear.

Needs Nav2 running with a map (see BUILD_GUIDE.md, week 8) and a stations file:

    home:    [x, y, yaw_degrees]
    pickup:  [x, y, yaw_degrees]
    dropoff: [x, y, yaw_degrees]      with --conveyor: rear towards the conveyor, about 30 cm clear of it

Run:  ros2 run cargo_bot mission --stations ~/maps/stations.yaml [--loops 3] [--set-initial-pose] [--conveyor]
"""
import argparse
import math
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger


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
    def __init__(self, nav, stations, load_kg, hold_s, conveyor=None):
        """conveyor: None = unload by hand, or a dict with dock_speed, dock_distance, clear_distance, push_timeout."""
        self.nav, self.stations, self.load_kg, self.hold_s = nav, stations, load_kg, hold_s
        self.conveyor = conveyor
        self.weight = None
        self.odom = None                 # (x, y, forward speed) from the wheel odometry
        self.pusher_state = None
        nav.create_subscription(Float32, "payload_weight", self.on_weight, 10)
        if conveyor:
            nav.create_subscription(Odometry, "odom", self.on_odom, 10)
            nav.create_subscription(String, "pusher/state", self.on_pusher_state, 10)
            self.cmd_pub = nav.create_publisher(Twist, "cmd_vel", 10)
            self.push_client = nav.create_client(Trigger, "pusher/push")

    def on_weight(self, msg):
        self.weight = msg.data

    def on_odom(self, msg):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x)

    def on_pusher_state(self, msg):
        self.pusher_state = msg.data

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

    def wait_weight(self, loaded, timeout=None):
        """Block until the bed has been loaded (or emptied) for hold_s seconds in a row.
        With a timeout (seconds), give up with an error instead of waiting for ever."""
        what = "cargo to be LOADED" if loaded else "cargo to be REMOVED"
        self.nav.get_logger().info(f"waiting for {what} ...")
        since = None
        t_end = None if timeout is None else time.monotonic() + timeout
        while True:
            rclpy.spin_once(self.nav, timeout_sec=0.1)
            if t_end is not None and time.monotonic() > t_end:
                seen = "no payload_weight data" if self.weight is None else f"payload = {self.weight:.2f} kg"
                raise RuntimeError(f"gave up waiting for {what} ({seen})")
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

    def creep(self, speed, distance, until_contact=False):
        """Drive straight at speed (m/s, negative = backwards) for distance metres of wheel odometry.
        With until_contact, also stop when the wheels stand still: the robot is pressing on something.
        Returns (why it stopped: "distance" or "contact", metres travelled)."""
        t0 = time.monotonic()
        while self.odom is None:
            rclpy.spin_once(self.nav, timeout_sec=0.1)
            if time.monotonic() - t0 > 5.0:
                raise RuntimeError("no odometry (is base_driver running?)")
        x0, y0, _ = self.odom
        travelled = lambda: math.hypot(self.odom[0] - x0, self.odom[1] - y0)
        cmd = Twist()
        cmd.linear.x = float(speed)
        t0, still_since, reason = time.monotonic(), None, "distance"
        time_limit = 5.0 + 3.0 * distance / abs(speed)
        try:
            while travelled() < distance:
                self.cmd_pub.publish(cmd)
                rclpy.spin_once(self.nav, timeout_sec=0.05)
                now = time.monotonic()
                if now - t0 > time_limit:
                    raise RuntimeError("the robot is not moving (E-stop pressed, or overloaded?)")
                if not until_contact or now - t0 < 1.0:           # the first second: still starting to move
                    continue
                if abs(self.odom[2]) > 0.2 * abs(speed):
                    still_since = None
                elif still_since is None:
                    still_since = now
                elif now - still_since > 0.5:
                    reason = "contact"
                    break
        finally:
            self.cmd_pub.publish(Twist())
        return reason, travelled()

    def push_cargo_off(self):
        """One pusher cycle: blade out to the rear, then back. Returns when the blade is parked again."""
        if not self.push_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("pusher_node is not running (start robot.launch.py with pusher:=true)")
        future = self.push_client.call_async(Trigger.Request())
        while not future.done():
            rclpy.spin_once(self.nav, timeout_sec=0.1)
        if not future.result().success:
            raise RuntimeError(f"the pusher refused to start: {future.result().message}")
        self.nav.get_logger().info("pushing the cargo off ...")
        t_end = time.monotonic() + self.conveyor["push_timeout"]
        left_home = False
        while True:
            rclpy.spin_once(self.nav, timeout_sec=0.1)
            if self.pusher_state == "fault":
                raise RuntimeError("pusher fault (the reason is in the pusher_node log)")
            if self.pusher_state not in (None, "home"):
                left_home = True
            elif left_home and self.pusher_state == "home":
                return
            if time.monotonic() > t_end:
                raise RuntimeError("the pusher did not finish in time")

    def unload_on_conveyor(self):
        c, log = self.conveyor, self.nav.get_logger().info
        log("backing up to the conveyor ...")
        why, moved = self.creep(-c["dock_speed"], c["dock_distance"], until_contact=True)
        if why != "contact":
            raise RuntimeError(f"backed up {moved:.2f} m without touching the conveyor - not pushing "
                               "(record the dropoff station closer to it, or raise --dock-distance)")
        if moved < 0.03:        # it never moved, so standing still does not mean "touching the conveyor"
            raise RuntimeError("the robot did not move when backing up - not pushing "
                               "(E-stop pressed, overloaded, or already against the conveyor)")
        log(f"touching the conveyor after {moved:.2f} m")
        self.push_cargo_off()
        self.wait_weight(loaded=False, timeout=10.0)
        log("driving clear of the conveyor ...")
        self.creep(c["dock_speed"] * 2.0, c["clear_distance"])

    def run(self, loops):
        for i in range(loops):
            self.nav.get_logger().info(f"=== delivery {i + 1}/{loops} ===")
            self.go("pickup")
            self.wait_weight(loaded=True)
            self.go("dropoff")
            if self.conveyor:
                self.unload_on_conveyor()
            else:
                self.wait_weight(loaded=False)
        self.go("home")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stations", required=True, help="YAML file with home / pickup / dropoff")
    parser.add_argument("--loops", type=int, default=1)
    parser.add_argument("--load-kg", type=float, default=0.3, help="weight that counts as 'loaded'")
    parser.add_argument("--hold", type=float, default=2.0, help="seconds the weight must be stable")
    parser.add_argument("--set-initial-pose", action="store_true", help="tell AMCL the robot is at 'home'")
    parser.add_argument("--conveyor", action="store_true",
                        help="unload at the drop-off with the pusher instead of waiting for a person")
    parser.add_argument("--dock-distance", type=float, default=0.60,
                        help="m: how far the robot may back up while looking for the conveyor")
    parser.add_argument("--dock-speed", type=float, default=0.05, help="m/s: backing-up speed")
    parser.add_argument("--clear-distance", type=float, default=0.30,
                        help="m: how far it drives forward again after unloading")
    parser.add_argument("--push-timeout", type=float, default=120.0, help="s: longest time for one pusher cycle")
    args, _ = parser.parse_known_args()

    with open(args.stations) as f:
        stations = yaml.safe_load(f)
    for key in ("home", "pickup", "dropoff"):
        if key not in stations:
            sys.exit(f"stations file is missing '{key}'")
    conveyor = None
    if args.conveyor:
        conveyor = dict(dock_speed=args.dock_speed, dock_distance=args.dock_distance,
                        clear_distance=args.clear_distance, push_timeout=args.push_timeout)

    rclpy.init()
    nav = BasicNavigator()
    if args.set_initial_pose:
        nav.setInitialPose(make_pose(nav, *stations["home"]))
    nav.waitUntilNav2Active()
    mission = Mission(nav, stations, args.load_kg, args.hold, conveyor)
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
