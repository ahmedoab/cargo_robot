#!/usr/bin/env python3
"""Pusher bench test (design v3).   KEEP FINGERS AND WIRES AWAY FROM THE BLADE AND THE LEAD SCREW.

  python3 tools/test_pusher.py              show the two limit switches; press each one by hand
  python3 tools/test_pusher.py --jog out    run the blade towards the REAR for 1 s (stops on the END switch)
  python3 tools/test_pusher.py --jog in     run the blade towards the FRONT for 1 s (stops on the HOME switch)
  python3 tools/test_pusher.py --cycle      one full push: out to END, back to HOME, with the time of each stroke

Uses invert_motor and the duty values of pusher_node in config/robot.yaml, so re-run it after changing them.
Stop robot.launch.py first: only one program can use the GPIO pins at a time.
"""
import argparse
import sys
import time

import common
from cargo_bot import hw
from cargo_bot.control import PusherCycle
from cargo_bot.pins import PINS

RATE = 50.0


def watch(home, end):
    print("Press each limit switch by hand (Ctrl+C to stop). Expected: '-' when released, 'PRESSED' when pressed.")
    last = None
    try:
        while True:
            now = (home.read(), end.read())
            if now != last:
                last = now
                note = "   !! both pressed: a wire is off (C -> GND, NC -> the pin)" if all(now) else ""
                print(f"  HOME: {'PRESSED' if now[0] else '-':8}  END: {'PRESSED' if now[1] else '-':8}{note}")
            time.sleep(0.02)
    except KeyboardInterrupt:
        print()


def jog(motor, home, end, direction, duty, seconds):
    switch, sign, where = (end, 1, "REAR") if direction == "out" else (home, -1, "FRONT")
    if switch.read():
        print(f"the {'END' if direction == 'out' else 'HOME'} switch is already pressed - not moving")
        return
    input(f"[jog {direction}] the blade should move towards the {where}. Press Enter ...")
    motor.set(sign * duty)
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end and not switch.read():
        time.sleep(0.005)
    motor.set(0)
    print("  stopped on the limit switch" if switch.read() else f"  ran for {seconds:.1f} s")
    print(f"Did it move towards the {where}? If it went the other way, set invert_motor: true under pusher_node\n"
          "in robot.yaml. If it did not move at all, check the motor power (E-stop), wires P17-P19 and S31-S34.")


def cycle(motor, home, end, cfg):
    seq = PusherCycle(cfg["duty_out"], cfg["duty_in"], cfg["stroke_timeout"], cfg["dwell"])
    seq.update(bool(home.read()), bool(end.read()), False, 0.0)
    if seq.state != "home":
        input("The blade is not parked. Press Enter to bring it HOME first ...")
        seq.home()
    else:
        input("[cycle] the blade goes to the REAR and comes back. Bed clear? Press Enter ...")
        seq.push()
    started_push = seq.state == "pushing"
    last, t_mark, dt = seq.state, time.monotonic(), 1.0 / RATE
    while seq.state in ("pushing", "out", "returning"):
        motor.set(seq.update(bool(home.read()), bool(end.read()), False, dt))
        if seq.state != last:
            if last in ("pushing", "returning"):
                print(f"  {last}: {time.monotonic() - t_mark:.1f} s")
            last, t_mark = seq.state, time.monotonic()
        time.sleep(dt)
    motor.set(0)
    if seq.state == "fault":
        print(f"  !! FAULT: {seq.reason}")
    elif started_push:
        print("  done: the blade is parked. A full stroke should take roughly 15-25 s.")
    else:
        print("  the blade is parked. Run --cycle again for a full push.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--jog", choices=("out", "in"))
    ap.add_argument("--cycle", action="store_true")
    ap.add_argument("--seconds", type=float, default=1.0, help="length of a --jog")
    args = ap.parse_args()
    cfg = common.config("pusher_node")

    chip = hw.open_chip()
    parts = []
    try:
        try:
            motor = hw.Motor(chip, PINS["pusher_out"], PINS["pusher_in"], cfg.get("pwm_freq", 1000), cfg["invert_motor"])
            parts.append(motor)
            home = hw.DigitalIn(chip, PINS["pusher_home"], pull_down=False)
            parts.append(home)
            end = hw.DigitalIn(chip, PINS["pusher_end"], pull_down=False)
            parts.append(end)
            sense = hw.DigitalIn(chip, PINS["estop_sense"])
            parts.append(sense)
        except Exception as e:
            sys.exit(f"cannot use the pusher pins: {e}\n"
                     "  'GPIO busy' means another program is using them (stop robot.launch.py),\n"
                     "  or SPI is still enabled on the Pi (BUILD_GUIDE.md, section 3.3).")
        print("motor power:", "ON" if sense.read() else "OFF  (E-stop pressed, or the GPIO26 divider is missing)")
        if args.jog:
            duty = cfg["duty_out"] if args.jog == "out" else cfg["duty_in"]
            jog(motor, home, end, args.jog, duty, args.seconds)
        elif args.cycle:
            cycle(motor, home, end, cfg)
        else:
            watch(home, end)
    finally:
        for part in parts:
            if isinstance(part, hw.Motor):
                part.set(0)
            part.close()
        hw.close_chip()


if __name__ == "__main__":
    main()
