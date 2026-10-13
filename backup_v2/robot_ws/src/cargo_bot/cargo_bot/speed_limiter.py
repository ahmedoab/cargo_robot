"""Speed limiter: the heavier the cargo, the slower the robot.

  subscribes  cmd_vel         geometry_msgs/Twist  from teleop / Nav2 / line follower
              payload_weight  std_msgs/Float32     from payload_node
  publishes   cmd_vel_safe    geometry_msgs/Twist  what the base driver actually executes
              speed_factor    std_msgs/Float32     1.0 = full speed, 0.0 = refusing to move (overload)
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32

from cargo_bot.control import clamp, payload_speed_factor


class SpeedLimiter(Node):
    def __init__(self):
        super().__init__("speed_limiter")
        self.declare_parameters("", [
            ("max_payload_kg", 15.0),
            ("min_factor", 0.5),          # speed multiplier at full payload
            ("overload_ratio", 1.1),      # above max_payload_kg * this -> stop
            ("max_linear", 0.5),          # m/s, hard limit
            ("max_angular", 1.5),         # rad/s, hard limit
            ("weight_timeout", 2.0),      # s without weight data -> assume full payload
        ])
        p = lambda name: self.get_parameter(name).value
        self.max_payload, self.min_factor = p("max_payload_kg"), p("min_factor")
        self.overload_ratio, self.weight_timeout = p("overload_ratio"), p("weight_timeout")
        self.max_lin, self.max_ang = p("max_linear"), p("max_angular")
        self.weight = None
        self.weight_time = None
        self.last_factor = None

        self.pub = self.create_publisher(Twist, "cmd_vel_safe", 10)
        self.factor_pub = self.create_publisher(Float32, "speed_factor", 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_subscription(Float32, "payload_weight", self.on_weight, 10)

    def on_weight(self, msg):
        self.weight = msg.data
        self.weight_time = self.get_clock().now()

    def factor(self):
        if self.weight is None:
            return 1.0                               # no load cells running (e.g. payload:=false)
        age = (self.get_clock().now() - self.weight_time).nanoseconds * 1e-9
        if age > self.weight_timeout:
            return self.min_factor                   # load cells stopped talking: be careful
        return payload_speed_factor(self.weight, self.max_payload, self.min_factor, self.overload_ratio)

    def on_cmd(self, msg):
        k = self.factor()
        if k == 0.0 and self.last_factor != 0.0:
            self.get_logger().warn(f"OVERLOAD ({self.weight:.1f} kg) - refusing to move")
        self.last_factor = k
        out = Twist()
        out.linear.x = clamp(msg.linear.x * k, -self.max_lin, self.max_lin)
        out.angular.z = clamp(msg.angular.z * k, -self.max_ang, self.max_ang)
        self.pub.publish(out)
        self.factor_pub.publish(Float32(data=float(k)))


def main():
    rclpy.init()
    node = SpeedLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
