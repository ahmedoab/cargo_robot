"""Pusher node (design v3): slides the cargo off the rear of the robot with the lead-screw blade.

  services   pusher/push    std_srvs/Trigger   start one unload cycle: blade out to the END switch, then back
             pusher/home    std_srvs/Trigger   bring the blade back to the HOME switch (also clears a fault)
             pusher/stop    std_srvs/Trigger   stop the motor now
  publishes  pusher/state   std_msgs/String    home | pushing | out | returning | stopped | fault
  subscribes estop          std_msgs/Bool      from base_driver: True while the E-stop has cut motor power

The services answer at once; watch pusher/state to see when the cycle has finished.
The motor stops on a limit switch, when a stroke takes longer than stroke_timeout (jammed, or no motor
power), when the E-stop is pressed during a stroke, and when the node shuts down.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from cargo_bot import hw
from cargo_bot.control import PusherCycle
from cargo_bot.pins import PINS


class PusherNode(Node):
    def __init__(self):
        super().__init__("pusher_node")
        self.declare_parameters("", [
            ("duty_out", 1.0),               # 0..1, motor power while pushing the cargo off
            ("duty_in", 1.0),                # 0..1, motor power while returning
            ("stroke_timeout", 45.0),        # s, give up if a limit switch is not reached in this time
            ("dwell", 0.5),                  # s, pause at the rear before returning
            ("invert_motor", False),         # true if "out" moves the blade to the FRONT
            ("pwm_freq", 1000),
            ("rate", 50.0),                  # Hz
        ])
        p = lambda name: self.get_parameter(name).value
        self.dt = 1.0 / p("rate")
        self.cycle = PusherCycle(p("duty_out"), p("duty_in"), p("stroke_timeout"), p("dwell"))

        chip = hw.open_chip()
        try:
            self.motor = hw.Motor(chip, PINS["pusher_out"], PINS["pusher_in"], p("pwm_freq"), p("invert_motor"))
            # NC contact to GND + pull-up: 0 = not pressed, 1 = pressed (or the wire is broken)
            self.switches = [hw.DigitalIn(chip, PINS[name], pull_down=False) for name in ("pusher_home", "pusher_end")]
        except Exception as e:
            self.get_logger().error(f"cannot use the pusher pins: {e}. If it says 'GPIO busy', SPI is still "
                                    "enabled on the Pi: see BUILD_GUIDE.md, section 3.3")
            raise
        self.pressed = [False, False]        # debounced: [HOME, END]
        self.last_raw = [0, 0]
        self.estop = False
        self.last_state = None

        self.state_pub = self.create_publisher(String, "pusher/state", 10)
        self.create_subscription(Bool, "estop", self.on_estop, 10)
        self.create_service(Trigger, "pusher/push", self.on_push)
        self.create_service(Trigger, "pusher/home", self.on_home)
        self.create_service(Trigger, "pusher/stop", self.on_stop)
        self.create_timer(self.dt, self.loop)
        self.create_timer(0.2, self.publish_state)
        self.get_logger().info("pusher_node ready")

    def on_estop(self, msg):
        self.estop = msg.data

    def on_push(self, request, response):
        if self.estop:
            response.success, response.message = False, "the E-stop is pressed"
        else:
            response.success, response.message = self.cycle.push()
        return response

    def on_home(self, request, response):
        response.success, response.message = self.cycle.home()
        return response

    def on_stop(self, request, response):
        self.cycle.stop()
        response.success, response.message = True, "stopped"
        return response

    def loop(self):
        for i, switch in enumerate(self.switches):
            raw = switch.read()
            self.pressed[i] = bool(raw and self.last_raw[i])     # pressed = two readings in a row
            self.last_raw[i] = raw
        duty = self.cycle.update(self.pressed[0], self.pressed[1], self.estop, self.dt)
        self.motor.set(duty)
        if self.cycle.state != self.last_state:
            self.last_state = self.cycle.state
            if self.cycle.state == "fault":
                self.get_logger().error(f"pusher fault: {self.cycle.reason}")
            else:
                self.get_logger().info(f"pusher: {self.cycle.state}")
            self.publish_state()

    def publish_state(self):
        self.state_pub.publish(String(data=self.cycle.state))

    def shutdown(self):
        self.motor.set(0.0)
        for part in [self.motor] + self.switches:
            part.close()
        hw.close_chip()


def main():
    rclpy.init()
    node = PusherNode()
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
