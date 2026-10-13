"""Pure-Python maths used by the nodes (no ROS, no GPIO - easy to test on any computer)."""
import math


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def twist_to_wheels(v, w, wheel_separation, wheel_radius):
    """Robot speed (v m/s forward, w rad/s turning left) -> wheel speeds (rad/s) (left, right)."""
    left = (v - w * wheel_separation / 2.0) / wheel_radius
    right = (v + w * wheel_separation / 2.0) / wheel_radius
    return left, right


def limit_wheels(left, right, max_speed):
    """Scale both wheels down together (keeps the curve) so neither exceeds max_speed rad/s."""
    biggest = max(abs(left), abs(right))
    if biggest <= max_speed or biggest == 0.0:
        return left, right
    k = max_speed / biggest
    return left * k, right * k


class WheelPID:
    """Wheel speed controller: feed-forward + PI. Input/feedback in rad/s, output motor duty -1..1.

    duty = sign(target) * (min_duty + kf * |target|) + kp * error + integral
      kf       duty per rad/s (about 1 / top speed at 100 % duty)
      min_duty duty needed to just start turning (static friction)
    """

    def __init__(self, kf, kp, ki, min_duty=0.0, i_limit=0.3):
        self.kf, self.kp, self.ki, self.min_duty, self.i_limit = kf, kp, ki, min_duty, i_limit
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, target, measured, dt):
        if abs(target) < 1e-3:
            self.reset()
            return 0.0
        error = target - measured
        self.integral = clamp(self.integral + self.ki * error * dt, -self.i_limit, self.i_limit)
        ff = math.copysign(self.min_duty + self.kf * abs(target), target)
        return clamp(ff + self.kp * error + self.integral, -1.0, 1.0)


class LowPass:
    def __init__(self, alpha):
        self.alpha, self.value = alpha, 0.0

    def update(self, x):
        self.value += self.alpha * (x - self.value)
        return self.value


class Odometry:
    """Dead-reckoning pose from the distance each wheel travelled."""

    def __init__(self):
        self.x = self.y = self.theta = 0.0

    def update(self, d_left, d_right, wheel_separation):
        d = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / wheel_separation
        mid = self.theta + d_theta / 2.0
        self.x += d * math.cos(mid)
        self.y += d * math.sin(mid)
        self.theta = math.atan2(math.sin(self.theta + d_theta), math.cos(self.theta + d_theta))
        return d, d_theta


def yaw_to_quaternion(yaw):
    """(x, y, z, w) for a rotation of yaw radians about Z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def payload_speed_factor(weight_kg, max_payload_kg, min_factor=0.5, overload_ratio=1.1):
    """1.0 empty -> min_factor at full payload -> 0.0 (refuse to move) above max * overload_ratio."""
    if weight_kg <= 0.0:
        return 1.0
    if weight_kg > max_payload_kg * overload_ratio:
        return 0.0
    frac = min(weight_kg / max_payload_kg, 1.0)
    return 1.0 - (1.0 - min_factor) * frac


def median(values):
    s = sorted(values)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


class PusherCycle:
    """The cargo pusher's sequence. The node reads the two limit switches, calls update() at a fixed rate
    and applies the duty it returns (+ = blade towards the rear, - = blade back towards the front).

    state   home       parked at the front (HOME switch pressed)
            pushing    moving to the rear, sliding the cargo off
            out        pausing at the rear (END switch pressed)
            returning  moving back to the front
            stopped    standing somewhere between the two switches
            fault      gave up; 'reason' says why, home() clears it
    """

    START_GRACE = 2.0       # s: by now the blade must have left the switch it started from

    def __init__(self, duty_out=1.0, duty_in=1.0, stroke_timeout=45.0, dwell=0.5):
        self.duty_out, self.duty_in = duty_out, duty_in
        self.stroke_timeout, self.dwell = stroke_timeout, dwell
        self.state, self.reason = "stopped", ""
        self.elapsed = 0.0
        self.at_home = self.at_end = False

    def push(self):
        """Start one unload cycle (out, pause, back). Returns (accepted, message)."""
        if self.state == "home":
            self._go("pushing")
            return True, "pushing"
        if self.state == "fault":
            return False, f"fault: {self.reason} - call pusher/home first"
        if self.state == "stopped":
            return False, "the blade is not parked - call pusher/home first"
        return False, "busy"

    def home(self):
        """Bring the blade back to the front. Also clears a fault. Returns (accepted, message)."""
        self.reason = ""
        if self.at_home and not self.at_end:
            self.state = "home"
            return True, "already parked"
        self._go("returning")
        return True, "returning"

    def stop(self):
        self.state = "home" if self.at_home else "stopped"

    def _go(self, state):
        self.state, self.elapsed = state, 0.0

    def _fault(self, reason):
        self.state, self.reason = "fault", reason
        return 0.0

    def update(self, at_home, at_end, estop, dt):
        """at_home / at_end: True while that limit switch is pressed. Returns the motor duty, -1..1."""
        self.at_home, self.at_end = at_home, at_end
        moving = self.state in ("pushing", "out", "returning")
        if at_home and at_end:
            return self._fault("both limit switches read 'pressed': a switch wire is broken or unplugged")
        if moving and estop:
            return self._fault("the E-stop was pressed during the stroke")
        if moving:
            self.elapsed += dt

        if self.state == "pushing":
            if at_end:
                self._go("out")
            elif at_home and self.elapsed > self.START_GRACE:
                return self._fault("the blade did not leave the HOME switch: motor wired backwards "
                                   "(set invert_motor), jammed, or no motor power")
            elif self.elapsed > self.stroke_timeout:
                return self._fault("the blade did not reach the END switch in time: jammed, or no motor power")
            else:
                return self.duty_out
        if self.state == "out":
            if self.elapsed >= self.dwell:
                self._go("returning")
            return 0.0
        if self.state == "returning":
            if at_home:
                self.state = "home"
            elif at_end and self.elapsed > self.START_GRACE:
                return self._fault("the blade did not leave the END switch: jammed, or no motor power")
            elif self.elapsed > self.stroke_timeout:
                return self._fault("the blade did not reach the HOME switch in time: jammed, or no motor power")
            else:
                return -self.duty_in

        if self.state == "home" and not at_home:
            self.state = "stopped"
        elif self.state == "stopped" and at_home:
            self.state = "home"
        return 0.0
