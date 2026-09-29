"""Payload node: 4 load cells (HX711) -> weight on the cargo bed.

  publishes  payload_weight   std_msgs/Float32            total kg (median-filtered)
             payload_corners  std_msgs/Float32MultiArray  kg per corner [FL, FR, RL, RR]
  service    payload/tare     std_srvs/Trigger            zero the scale (bed must be empty)

Calibrate once with tools/test_hx711.py --calibrate <kg> and put 'scale' (and optionally 'offsets')
into config/robot.yaml.
"""
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from std_srvs.srv import Trigger

from cargo_bot import hw
from cargo_bot.control import median
from cargo_bot.pins import PINS

SATURATED = (-(1 << 23), (1 << 23) - 1)


class PayloadNode(Node):
    def __init__(self):
        super().__init__("payload_node")
        self.declare_parameters("", [
            ("scale", 22000.0),                      # raw counts (sum of 4 cells) per kg - calibrate!
            ("offsets", [0.0, 0.0, 0.0, 0.0]),       # raw zero of each cell (used when tare_on_start is false)
            ("signs", [1, 1, 1, 1]),                 # -1 for a cell wired "upside down"
            ("tare_on_start", True),                 # zero automatically at start (bed must be empty!)
            ("median_window", 5),
        ])
        p = lambda name: self.get_parameter(name).value
        self.scale = p("scale")
        self.offsets = list(p("offsets"))
        self.signs = list(p("signs"))
        self.window = deque(maxlen=p("median_window"))

        self.cells = hw.HX711Array(hw.open_chip(), PINS["hx711_sck"], PINS["hx711_dout"])
        self.weight_pub = self.create_publisher(Float32, "payload_weight", 10)
        self.corner_pub = self.create_publisher(Float32MultiArray, "payload_corners", 10)
        self.create_service(Trigger, "payload/tare", self.on_tare)
        self.bad_reads = 0

        if p("tare_on_start"):
            ok, msg = self.tare()
            self.get_logger().info(msg) if ok else self.get_logger().error(msg)
        self.create_timer(0.02, self.poll)          # HX711 gives a new reading 10x per second

    def read(self):
        raw = self.cells.read_raw(timeout=0.3)
        if raw is None or any(v in SATURATED for v in raw):
            self.bad_reads += 1
            if self.bad_reads % 50 == 1:
                missing = self.cells.not_ready()
                self.get_logger().warn(f"load-cell read failed (not ready: {missing}) - check HX711 wiring")
            return None
        return raw

    def tare(self, samples=20):
        readings = [r for r in (self.read() for _ in range(samples)) if r is not None]
        if len(readings) < samples // 2:
            return False, "tare failed: HX711 boards not responding"
        self.offsets = [median([r[i] for r in readings]) for i in range(len(readings[0]))]
        self.window.clear()
        return True, f"tare done, offsets = {[round(o) for o in self.offsets]}"

    def on_tare(self, request, response):
        response.success, response.message = self.tare()
        return response

    def poll(self):
        if self.cells.not_ready():
            return
        raw = self.read()
        if raw is None:
            return
        corners = [s * (r - o) / self.scale for r, o, s in zip(raw, self.offsets, self.signs)]
        self.window.append(sum(corners))
        self.weight_pub.publish(Float32(data=float(median(self.window))))
        self.corner_pub.publish(Float32MultiArray(data=[float(c) for c in corners]))

    def shutdown(self):
        self.cells.close()
        hw.close_chip()


def main():
    rclpy.init()
    node = PayloadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
