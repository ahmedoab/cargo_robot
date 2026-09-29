#!/usr/bin/env python3
"""Live encoder counts. Measure ticks_per_rev:

  1. python3 tools/test_encoders.py
  2. put a tape mark on each tyre, turn the wheel by hand exactly 10 turns FORWARD
  3. ticks_per_rev = count / 10   (both wheels should agree within ~1 %)
Ctrl+C to stop.
"""
import time

import common
from cargo_bot import hw
from cargo_bot.pins import PINS


def main():
    cfg = common.config("base_driver")
    chip = hw.open_chip()
    left = hw.QuadratureEncoder(chip, PINS["left_enc_a"], PINS["left_enc_b"], cfg["invert_left_encoder"])
    right = hw.QuadratureEncoder(chip, PINS["right_enc_a"], PINS["right_enc_b"], cfg["invert_right_encoder"])
    try:
        while True:
            print(f"\rleft {left.count:+8d}   right {right.count:+8d}   "
                  f"(missed edges {left.errors}/{right.errors})   ", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print(f"\n\nleft  / 10 = {left.count / 10:.1f}\nright / 10 = {right.count / 10:.1f}")
    finally:
        left.close()
        right.close()
        hw.close_chip()


if __name__ == "__main__":
    main()
