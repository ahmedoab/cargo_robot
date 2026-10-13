#!/usr/bin/env python3
"""Load-cell test and calibration.

  python3 tools/test_hx711.py                 live readings of the 4 cells + total kg (bed empty at start)
  python3 tools/test_hx711.py --calibrate 2.0 calibrate with a known 2.0 kg weight -> prints scale + signs
"""
import argparse
import time

import common
from cargo_bot import hw
from cargo_bot.control import median
from cargo_bot.pins import CORNERS, PINS


def average(cells, n=30):
    rows = [r for r in (cells.read_raw() for _ in range(n)) if r is not None]
    if len(rows) < n // 2:
        raise SystemExit(f"HX711 not answering - boards not ready: {cells.not_ready()}")
    return [median([r[i] for r in rows]) for i in range(len(rows[0]))]


def live(cells):
    cfg = common.config("payload_node")
    scale, signs = cfg["scale"], cfg["signs"]
    print("zeroing - keep the cargo bed EMPTY ...")
    zero = average(cells, 10)
    print("change from zero (raw counts):")
    print("   ".join(f"{c:>12}" for c in CORNERS) + "     total kg (uses scale/signs from robot.yaml)")
    while True:
        raw = cells.read_raw()
        if raw is None:
            print(f"not ready: {[CORNERS[i] for i in cells.not_ready()]}")
            continue
        delta = [s * (r - z) for r, z, s in zip(raw, zero, signs)]
        print("   ".join(f"{d:12.0f}" for d in delta) + f"     {sum(delta) / scale:8.3f} kg")
        time.sleep(0.2)


def calibrate(cells, kg):
    input("1) EMPTY the cargo bed, then press Enter ...")
    zero = average(cells)
    input(f"2) put the {kg} kg weight in the MIDDLE of the bed, then press Enter ...")
    loaded = average(cells)
    delta = [l - z for l, z in zip(loaded, zero)]
    total = sum(delta)
    majority = 1 if total >= 0 else -1
    signs = [1 if d * majority >= 0 or abs(d) < 0.05 * abs(total) / 4 else -1 for d in delta]
    for c, d, s in zip(CORNERS, delta, signs):
        flag = "" if s == 1 else "   <-- opposite sign: swap this cell's A+/A- wires, or keep sign -1"
        print(f"   {c:12s} change {d:+10.0f}{flag}")
    scale = sum(s * d for s, d in zip(signs, delta)) / kg
    print("\nPut this in config/robot.yaml (payload_node):")
    print(f"    scale: {scale:.1f}")
    print(f"    signs: {signs}")
    if abs(scale) < 1000:
        print("!! very small change - check the load-cell wiring (E+/E-/A+/A-)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calibrate", type=float, metavar="KG")
    args = ap.parse_args()
    cells = hw.HX711Array(hw.open_chip(), PINS["hx711_sck"], PINS["hx711_dout"])
    try:
        calibrate(cells, args.calibrate) if args.calibrate else live(cells)
    except KeyboardInterrupt:
        pass
    finally:
        cells.close()
        hw.close_chip()


if __name__ == "__main__":
    main()
