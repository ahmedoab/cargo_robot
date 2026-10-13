#!/usr/bin/env python3
"""Motor + encoder bench test.   PUT THE ROBOT ON A BOX SO BOTH WHEELS ARE IN THE AIR.

  python3 tools/test_motors.py          each wheel forward, then backward, 30 % for 2 s
  python3 tools/test_motors.py --max    each wheel at 100 % for 3 s   -> prints kf
  python3 tools/test_motors.py --min    slow ramp until the wheel moves -> prints min_duty

Uses the invert_* and ticks_per_rev values from config/robot.yaml, so re-run it after changing them.
"""
import argparse
import math
import time

import common
from cargo_bot import hw
from cargo_bot.pins import PINS


def speed(enc, seconds, ticks):
    c0 = enc.count
    time.sleep(seconds)
    return (enc.count - c0) / ticks * 2 * math.pi / seconds      # rad/s at the wheel


def basic(motors, encs, ticks, duty):
    for side in ("left", "right"):
        for sign, name in ((1, "FORWARD"), (-1, "BACKWARD")):
            input(f"\n[{side} wheel {name}] press Enter, watch the {side} wheel ...")
            c0 = encs[side].count
            motors[side].set(sign * duty)
            time.sleep(2.0)
            motors[side].set(0)
            time.sleep(0.5)
            delta = encs[side].count - c0
            print(f"  encoder change: {delta:+d} counts ({delta / ticks:+.2f} wheel turns), "
                  f"missed edges so far: {encs[side].errors}")
            if delta == 0:
                print("  !! encoder did not count: check encoder power (3.3 V), GND and A/B wires")
            elif (delta > 0) != (sign > 0):
                print(f"  !! count has the wrong sign: set invert_{side}_encoder: true in robot.yaml")
    print("\nCheck by eye: on FORWARD, did each wheel turn the way that drives the robot FORWARD?")
    print("If not, set invert_<side>_motor: true in robot.yaml, then run this test again "
          "(the encoder sign must also come out right).")


def max_speed(motors, encs, ticks):
    speeds = {}
    for side in ("left", "right"):
        input(f"\n[{side} wheel 100 %] press Enter ...")
        motors[side].set(1.0)
        time.sleep(1.0)                                  # let it spin up
        speeds[side] = speed(encs[side], 2.0, ticks)
        motors[side].set(0)
        print(f"  top speed {speeds[side]:.1f} rad/s = {speeds[side] * 30 / math.pi:.0f} rpm")
    slowest = min(abs(s) for s in speeds.values())
    print(f"\nPut this in config/robot.yaml:   kf: {1.0 / slowest:.3f}")


def min_duty(motors, encs):
    found = []
    for side in ("left", "right"):
        input(f"\n[{side} wheel ramp] press Enter ...")
        duty, c0 = 0.0, encs[side].count
        while duty < 0.5 and abs(encs[side].count - c0) < 20:
            duty += 0.01
            motors[side].set(duty)
            time.sleep(0.3)
        motors[side].set(0)
        print(f"  started moving at {duty:.2f}")
        found.append(duty)
    print(f"\nPut this in config/robot.yaml:   min_duty: {0.9 * max(found):.2f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--max", action="store_true")
    ap.add_argument("--min", action="store_true")
    ap.add_argument("--duty", type=float, default=0.3)
    args = ap.parse_args()
    cfg = common.config("base_driver")
    ticks = cfg["ticks_per_rev"]

    chip = hw.open_chip()
    enable = hw.DigitalOut(chip, PINS["enable"], 0)
    motors = {
        "left": hw.Motor(chip, PINS["left_fwd"], PINS["left_rev"], invert=cfg["invert_left_motor"]),
        "right": hw.Motor(chip, PINS["right_fwd"], PINS["right_rev"], invert=cfg["invert_right_motor"]),
    }
    encs = {
        "left": hw.QuadratureEncoder(chip, PINS["left_enc_a"], PINS["left_enc_b"], cfg["invert_left_encoder"]),
        "right": hw.QuadratureEncoder(chip, PINS["right_enc_a"], PINS["right_enc_b"], cfg["invert_right_encoder"]),
    }
    sense = hw.DigitalIn(chip, PINS["estop_sense"])
    print("motor power:", "ON" if sense.read() else "OFF  (E-stop pressed, or the GPIO26 divider is missing)")
    enable.write(1)
    try:
        if args.max:
            max_speed(motors, encs, ticks)
        elif args.min:
            min_duty(motors, encs)
        else:
            basic(motors, encs, ticks, args.duty)
    finally:
        for m in motors.values():
            m.set(0)
        enable.write(0)
        for part in list(motors.values()) + list(encs.values()) + [enable, sense]:
            part.close()
        hw.close_chip()


if __name__ == "__main__":
    main()
