"""Raspberry Pi hardware drivers built on lgpio (works on Pi 4 and Pi 5, Ubuntu 24.04).

    Motor              one BTS7960 channel (RPWM = forward, LPWM = reverse), software PWM
    DigitalOut/In      plain GPIO output / input
    QuadratureEncoder  counts A/B encoder edges in the background (x4 decoding)
    HX711Array         several HX711 boards on one shared clock line

All pin numbers are BCM GPIO numbers, not physical header pin numbers.
"""
import glob
import os
import time

# lgpio writes small notification files into its working directory; make sure it is writable.
os.environ.setdefault("LG_WD", "/tmp")
import lgpio  # noqa: E402

_chip = None


def open_chip():
    """Open the gpiochip that drives the 40-pin header (auto-detects Pi 4 / Pi 5)."""
    global _chip
    if _chip is not None:
        return _chip
    numbers = sorted(int(p.rsplit("gpiochip", 1)[1]) for p in glob.glob("/dev/gpiochip[0-9]*"))
    for n in numbers:
        try:
            h = lgpio.gpiochip_open(n)
        except Exception:
            continue
        label = lgpio.gpio_get_chip_info(h)[3]
        if isinstance(label, bytes):
            label = label.decode()
        if label.startswith(("pinctrl-rp1", "pinctrl-bcm2711", "pinctrl-bcm2835")):
            _chip = h
            return h
        lgpio.gpiochip_close(h)
    raise RuntimeError("Raspberry Pi GPIO chip not found - is this a Pi, and is your user in the 'gpio' group?")


def close_chip():
    global _chip
    if _chip is not None:
        lgpio.gpiochip_close(_chip)
        _chip = None


class DigitalOut:
    def __init__(self, h, pin, level=0):
        self.h, self.pin = h, pin
        lgpio.gpio_claim_output(h, pin, level)

    def write(self, level):
        lgpio.gpio_write(self.h, self.pin, 1 if level else 0)

    def close(self):
        lgpio.gpio_write(self.h, self.pin, 0)
        lgpio.gpio_free(self.h, self.pin)


class DigitalIn:
    def __init__(self, h, pin, pull_down=True):
        self.h, self.pin = h, pin
        lgpio.gpio_claim_input(h, pin, lgpio.SET_PULL_DOWN if pull_down else lgpio.SET_PULL_UP)

    def read(self):
        return lgpio.gpio_read(self.h, self.pin)

    def close(self):
        lgpio.gpio_free(self.h, self.pin)


class Motor:
    """One BTS7960 (IBT-2) motor channel. set(+0.5) = 50 % forward, set(-1) = full reverse, set(0) = brake."""

    def __init__(self, h, pin_fwd, pin_rev, freq=1000, invert=False):
        self.h, self.fwd, self.rev, self.freq, self.invert = h, pin_fwd, pin_rev, freq, invert
        for p in (pin_fwd, pin_rev):
            lgpio.gpio_claim_output(h, p, 0)
        self._last = None
        self.set(0.0)

    def set(self, duty):
        duty = max(-1.0, min(1.0, float(duty)))
        if self.invert:
            duty = -duty
        pct = round(abs(duty) * 100.0, 1)
        state = (pct, duty >= 0)
        if state == self._last:
            return                      # nothing changed - don't restart the PWM
        self._last = state
        on, off = (self.fwd, self.rev) if duty >= 0 else (self.rev, self.fwd)
        lgpio.tx_pwm(self.h, off, self.freq, 0)
        lgpio.tx_pwm(self.h, on, self.freq, pct)

    def close(self):
        for p in (self.fwd, self.rev):
            lgpio.tx_pwm(self.h, p, 0, 0)
            lgpio.gpio_write(self.h, p, 0)
            lgpio.gpio_free(self.h, p)


class QuadratureEncoder:
    """Counts every edge of channels A and B (x4 decoding). Read .count at any time."""

    # index = (old_state << 2) | new_state, where state = (A << 1) | B
    _TABLE = (0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0)

    def __init__(self, h, pin_a, pin_b, invert=False):
        self.h, self.pin_a, self.pin_b = h, pin_a, pin_b
        self.sign = -1 if invert else 1
        self.count = 0
        self.errors = 0                 # impossible transitions = missed edges (should stay ~0)
        for p in (pin_a, pin_b):
            lgpio.gpio_claim_alert(h, p, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
        self._a = lgpio.gpio_read(h, pin_a)
        self._b = lgpio.gpio_read(h, pin_b)
        self._state = (self._a << 1) | self._b
        self._cbs = [lgpio.callback(h, p, lgpio.BOTH_EDGES, self._edge) for p in (pin_a, pin_b)]

    def _edge(self, chip, gpio, level, tick):
        if level > 1:                   # 2 = watchdog timeout, not an edge
            return
        if gpio == self.pin_a:
            self._a = level
        else:
            self._b = level
        new = (self._a << 1) | self._b
        step = self._TABLE[(self._state << 2) | new]
        if step == 0 and new != self._state:
            self.errors += 1
        self._state = new
        self.count += step * self.sign

    def close(self):
        for cb in self._cbs:
            cb.cancel()
        for p in (self.pin_a, self.pin_b):
            lgpio.gpio_free(self.h, p)


class HX711Array:
    """Several HX711 boards sharing one SCK line, each with its own DOUT line (channel A, gain 128)."""

    def __init__(self, h, pin_sck, pins_dout):
        self.h, self.sck, self.douts = h, pin_sck, list(pins_dout)
        lgpio.gpio_claim_output(h, pin_sck, 0)       # SCK low = chips powered up
        for d in self.douts:
            lgpio.gpio_claim_input(h, d, lgpio.SET_PULL_UP)

    def not_ready(self):
        """Indexes of boards that have no new reading yet (DOUT high)."""
        return [i for i, d in enumerate(self.douts) if lgpio.gpio_read(self.h, d)]

    def read_raw(self, timeout=0.5):
        """Signed 24-bit reading of every board, or None if they were not all ready in time."""
        t_end = time.monotonic() + timeout
        while self.not_ready():
            if time.monotonic() > t_end:
                return None
            time.sleep(0.002)
        h, sck, douts = self.h, self.sck, self.douts
        write, read = lgpio.gpio_write, lgpio.gpio_read
        vals = [0] * len(douts)
        for _ in range(24):
            write(h, sck, 1)            # keep SCK high as short as possible (>60 us = chip powers down)
            write(h, sck, 0)
            for i, d in enumerate(douts):
                vals[i] = (vals[i] << 1) | read(h, d)
        write(h, sck, 1)                # 25th pulse: next conversion = channel A, gain 128
        write(h, sck, 0)
        return [v - (1 << 24) if v & 0x800000 else v for v in vals]

    def close(self):
        lgpio.gpio_write(self.h, self.sck, 0)
        for p in [self.sck] + self.douts:
            lgpio.gpio_free(self.h, p)
