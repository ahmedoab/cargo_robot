"""Base driver node.

  subscribes  cmd_vel        geometry_msgs/Twist   wanted robot speed (the launch file remaps it to cmd_vel_safe)
  publishes   odom           nav_msgs/Odometry     wheel odometry
              joint_states   sensor_msgs/JointState  wheel angles (for the robot model in RViz)
              estop          std_msgs/Bool         True while the E-stop has cut motor power
  tf          odom -> base_footprint

Every 1/rate s: read encoders -> update odometry -> PID each wheel -> set motor PWM.
Motors stop if no cmd_vel arrives for cmd_timeout s, or while the E-stop is pressed.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry as OdometryMsg
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from cargo_bot import hw
from cargo_bot.control import LowPass, Odometry, WheelPID, limit_wheels, twist_to_wheels, yaw_to_quaternion
from cargo_bot.pins import PINS


class BaseDriver(Node):
    def __init__(self):
        super().__init__("base_driver")
        self.declare_parameters("", [
            ("wheel_radius", 0.05),          # m
            ("wheel_separation", 0.33),      # m, centre of left tyre to centre of right tyre
            ("ticks_per_rev", 2464),         # encoder counts per WHEEL turn (x4 decoding) - measure it!
            ("max_wheel_speed", 12.0),       # rad/s, commands are scaled down to this
            ("rate", 50.0),                  # Hz
            ("cmd_timeout", 0.5),            # s
            ("kf", 0.05), ("kp", 0.02), ("ki", 0.10), ("min_duty", 0.10),
            ("pwm_freq", 1000),
            ("invert_left_motor", False), ("invert_right_motor", False),
            ("invert_left_encoder", False), ("invert_right_encoder", False),
            ("use_estop_sense", True),
            ("odom_frame", "odom"), ("base_frame", "base_footprint"), ("publish_tf", True),
        ])
        p = lambda name: self.get_parameter(name).value
        self.R, self.sep, self.tpr = p("wheel_radius"), p("wheel_separation"), p("ticks_per_rev")
        self.max_w, self.timeout = p("max_wheel_speed"), p("cmd_timeout")
        self.odom_frame, self.base_frame, self.publish_tf = p("odom_frame"), p("base_frame"), p("publish_tf")

        chip = hw.open_chip()
        self.enable = hw.DigitalOut(chip, PINS["enable"], 0)
        self.motor_l = hw.Motor(chip, PINS["left_fwd"], PINS["left_rev"], p("pwm_freq"), p("invert_left_motor"))
        self.motor_r = hw.Motor(chip, PINS["right_fwd"], PINS["right_rev"], p("pwm_freq"), p("invert_right_motor"))
        self.enc_l = hw.QuadratureEncoder(chip, PINS["left_enc_a"], PINS["left_enc_b"], p("invert_left_encoder"))
        self.enc_r = hw.QuadratureEncoder(chip, PINS["right_enc_a"], PINS["right_enc_b"], p("invert_right_encoder"))
        self.estop_in = hw.DigitalIn(chip, PINS["estop_sense"]) if p("use_estop_sense") else None
        self.enable.write(1)

        gains = dict(kf=p("kf"), kp=p("kp"), ki=p("ki"), min_duty=p("min_duty"))
        self.pid_l, self.pid_r = WheelPID(**gains), WheelPID(**gains)
        self.speed_l, self.speed_r = LowPass(0.5), LowPass(0.5)
        self.odom = Odometry()
        self.last_counts = (self.enc_l.count, self.enc_r.count)
        self.target = (0.0, 0.0)                     # wheel speeds, rad/s
        self.last_cmd_time = self.get_clock().now()
        self.last_time = self.get_clock().now()
        self.estop_active = None

        self.odom_pub = self.create_publisher(OdometryMsg, "odom", 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.estop_pub = self.create_publisher(Bool, "estop", 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_timer(1.0 / p("rate"), self.loop)
        self.create_timer(1.0, self.publish_estop)
        self.get_logger().info("base_driver ready")

    def on_cmd(self, msg):
        left, right = twist_to_wheels(msg.linear.x, msg.angular.z, self.sep, self.R)
        self.target = limit_wheels(left, right, self.max_w)
        self.last_cmd_time = self.get_clock().now()

    def loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now

        # 1. encoders -> wheel speeds and odometry
        cl, cr = self.enc_l.count, self.enc_r.count
        dl, dr = cl - self.last_counts[0], cr - self.last_counts[1]
        self.last_counts = (cl, cr)
        rad_per_tick = 2.0 * math.pi / self.tpr
        wl = self.speed_l.update(dl * rad_per_tick / dt)
        wr = self.speed_r.update(dr * rad_per_tick / dt)
        d, d_theta = self.odom.update(dl * rad_per_tick * self.R, dr * rad_per_tick * self.R, self.sep)
        self.publish_odom(now, d / dt, d_theta / dt, cl * rad_per_tick, cr * rad_per_tick)

        # 2. safety: E-stop and command timeout
        estop = self.estop_in is not None and self.estop_in.read() == 0
        if estop != self.estop_active:
            self.estop_active = estop
            self.publish_estop()
            if estop:
                self.get_logger().warn("E-STOP: motor power is OFF")
            else:
                self.get_logger().info("motor power is ON")
        stale = (now - self.last_cmd_time).nanoseconds * 1e-9 > self.timeout
        target_l, target_r = (0.0, 0.0) if (estop or stale) else self.target

        # 3. PID -> motors
        if estop:
            self.pid_l.reset()
            self.pid_r.reset()
        self.motor_l.set(self.pid_l.update(target_l, wl, dt))
        self.motor_r.set(self.pid_r.update(target_r, wr, dt))

    def publish_odom(self, now, v, w, pos_l, pos_r):
        stamp = now.to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(self.odom.theta)
        msg = OdometryMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.odom.x
        msg.pose.pose.position.y = self.odom.y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
        msg.pose.covariance[14] = msg.pose.covariance[21] = msg.pose.covariance[28] = 1e6
        msg.pose.covariance[35] = 0.05
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        msg.twist.covariance[0] = 0.01
        msg.twist.covariance[7] = msg.twist.covariance[14] = msg.twist.covariance[21] = 1e6
        msg.twist.covariance[28] = 1e6
        msg.twist.covariance[35] = 0.05
        self.odom_pub.publish(msg)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.odom.x
            t.transform.translation.y = self.odom.y
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tf_pub.sendTransform(t)

        js = JointState()
        js.header.stamp = stamp
        js.name = ["left_wheel_joint", "right_wheel_joint"]
        js.position = [pos_l, pos_r]
        self.joint_pub.publish(js)

    def publish_estop(self):
        self.estop_pub.publish(Bool(data=bool(self.estop_active)))

    def shutdown(self):
        self.motor_l.set(0.0)
        self.motor_r.set(0.0)
        self.enable.write(0)
        for part in (self.motor_l, self.motor_r, self.enc_l, self.enc_r, self.enable, self.estop_in):
            if part is not None:
                part.close()
        hw.close_chip()


def main():
    rclpy.init()
    node = BaseDriver()
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
