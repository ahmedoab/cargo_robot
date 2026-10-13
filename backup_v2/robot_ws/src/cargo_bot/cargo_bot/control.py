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
