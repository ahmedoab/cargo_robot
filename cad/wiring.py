"""
Every wire of the cargo robot, in one place.

Each connection is (wire ID, part A, pin A, part B, pin B, wire, note). Several rows can share one wire ID when
one job needs several wires (e.g. S6 = the VCC of both motor drivers). GPIO numbers are read from
firmware/cargo_robot/config.h, so they always match the firmware.

Used by:
  make_docs.py        -> autocad/CargoRobot_v2_connections.pdf and the wire tables in BUILD_GUIDE.md (weeks 2, 3)
  make_schematics.py  -> the pin numbers on the schematics
"""
import re
from pathlib import Path

CONFIG = Path(__file__).resolve().parents[1] / "firmware" / "cargo_robot" / "config.h"


def read_pins():
    """Every 'constexpr int NAME = n;' (or '= {a, b, ...}') in config.h."""
    pins = {}
    for m in re.finditer(r"constexpr int (\w+)(?:\[\w+\])? = (\{[^}]*\}|0x[0-9A-Fa-f]+|\d+);", CONFIG.read_text()):
        v = m.group(2)
        pins[m.group(1)] = [int(x) for x in v.strip("{}").split(",")] if v.startswith("{") else int(v, 0)
    return pins


P = read_pins()


def gpio(key, i=None):
    return f"GPIO {P[key] if i is None else P[key][i]}"


ESP = "ESP32 (breakout)"
L_DRV, R_DRV = "BTS7960 left U2", "BTS7960 right U3"
L_MOT, R_MOT = "Left motor M1", "Right motor M2"
SONAR = "HC-SR04 (front)"
LINE = ("IR line sensor 1 (left)", "IR line sensor 2 (middle)", "IR line sensor 3 (right)")

POWER = [
    ("P1", "Battery BT1", "+ (XT60)", "Fuse holder F1", "in", "16 AWG red, 10 cm", "XT60 soldered"),
    ("P2", "Fuse holder F1", "out", "Master switch SW1", "terminal 1", "16 AWG red", ""),
    ("P3", "Master switch SW1", "terminal 2", "WAGO +12 V MAIN", "", "16 AWG red", "WAGO 221-415"),
    ("P4", "Battery BT1", "- (XT60)", "WAGO GND", "", "16 AWG black", "WAGO 221-415, link a second one"),
    ("P5", "WAGO +12 V MAIN", "", "E-stop S1", "NC terminal 1", "16 AWG red, 40 cm", "ferrule"),
    ("P6", "E-stop S1", "NC terminal 2", "WAGO +12 V MOTOR", "", "16 AWG red, 40 cm", "ferrule"),
    ("P7", "WAGO +12 V MOTOR", "", L_DRV, "B+", "16 AWG red", "ferrule"),
    ("P8", "WAGO +12 V MOTOR", "", R_DRV, "B+", "16 AWG red", "ferrule"),
    ("P9", "WAGO GND", "", L_DRV, "B-", "16 AWG black", "ferrule"),
    ("P10", "WAGO GND", "", R_DRV, "B-", "16 AWG black", "ferrule"),
    ("P11", L_DRV, "M+", L_MOT, "red (motor +)", "20 AWG or the motor's wire", "ferrule"),
    ("P11", L_DRV, "M-", L_MOT, "white (motor -)", "20 AWG or the motor's wire", "ferrule"),
    ("P12", R_DRV, "M+", R_MOT, "red (motor +)", "20 AWG or the motor's wire", "ferrule"),
    ("P12", R_DRV, "M-", R_MOT, "white (motor -)", "20 AWG or the motor's wire", "ferrule"),
    ("P13", "WAGO +12 V MAIN", "", "5 V buck U1", "IN+", "20 AWG red", "ferrule"),
    ("P14", "WAGO GND", "", "5 V buck U1", "IN-", "20 AWG black", "ferrule"),
    ("P15", "5 V buck U1", "OUT+", "WAGO +5 V", "", "20 AWG red", "set to 5.10 V first"),
    ("P15", "5 V buck U1", "OUT-", "WAGO GND", "", "20 AWG black", ""),
    ("P15", "WAGO +5 V", "", ESP, "5V", "20 AWG red", ""),
    ("P15", "WAGO GND", "", ESP, "GND", "20 AWG black", "the logic ground starts here"),
    ("P16", "Battery BT1", "balance plug", "LiPo voltage alarm", "", "-", "set it to 3.5 V per cell"),
    ("P17", "WAGO +12 V MOTOR", "", "Servo buck U5", "IN+", "16 AWG red", "ferrule"),
    ("P18", "WAGO GND", "", "Servo buck U5", "IN-", "16 AWG black", "ferrule"),
    ("P19", "Servo buck U5", "OUT+", "WAGO +7.4 V", "", "16 AWG red", "set to 7.4 V first"),
    ("P19", "Servo buck U5", "OUT-", "WAGO GND", "", "16 AWG black", ""),
    ("P19", "WAGO +7.4 V", "", "Front servo SV1", "red (+)", "servo lead", ""),
    ("P19", "WAGO GND", "", "Front servo SV1", "brown (-)", "servo lead", ""),
    ("P19", "WAGO +7.4 V", "", "Rear servo SV2", "red (+)", "servo lead", ""),
    ("P19", "WAGO GND", "", "Rear servo SV2", "brown (-)", "servo lead", ""),
]

J = "jumper 24-26 AWG"
SIGNAL = [
    ("S1", L_DRV, "RPWM", ESP, gpio("PIN_L_RPWM"), J, "forward PWM"),
    ("S2", L_DRV, "LPWM", ESP, gpio("PIN_L_LPWM"), J, "reverse PWM"),
    ("S3", R_DRV, "RPWM", ESP, gpio("PIN_R_RPWM"), J, "forward PWM"),
    ("S4", R_DRV, "LPWM", ESP, gpio("PIN_R_LPWM"), J, "reverse PWM"),
    ("S5", L_DRV, "R_EN + L_EN (linked)", ESP, gpio("PIN_MOTOR_EN"), J, "MOTOR_EN; 10k pull-down to GND"),
    ("S5", R_DRV, "R_EN + L_EN (linked)", ESP, gpio("PIN_MOTOR_EN"), J, "MOTOR_EN"),
    ("S6", L_DRV, "VCC", "3V3 bus", "", J, "3.3 V, not 5 V"),
    ("S6", R_DRV, "VCC", "3V3 bus", "", J, "3.3 V, not 5 V"),
    ("S7", L_DRV, "GND (logic)", "GND bus", "", J, ""),
    ("S7", R_DRV, "GND (logic)", "GND bus", "", J, ""),
    ("S8", L_MOT, "blue (encoder VCC)", "3V3 bus", "", J, "3.3 V, never 5 V"),
    ("S9", L_MOT, "black (encoder GND)", "GND bus", "", J, ""),
    ("S10", L_MOT, "yellow (encoder A)", ESP, gpio("PIN_ENC_L_A"), J, "10k pull-up to 3V3"),
    ("S11", L_MOT, "green (encoder B)", ESP, gpio("PIN_ENC_L_B"), J, "10k pull-up to 3V3"),
    ("S12", R_MOT, "blue (encoder VCC)", "3V3 bus", "", J, "3.3 V, never 5 V"),
    ("S13", R_MOT, "black (encoder GND)", "GND bus", "", J, ""),
    ("S14", R_MOT, "yellow (encoder A)", ESP, gpio("PIN_ENC_R_A") + " (VP)", J, "10k pull-up to 3V3"),
    ("S15", R_MOT, "green (encoder B)", ESP, gpio("PIN_ENC_R_B") + " (VN)", J, "10k pull-up to 3V3"),
    ("S16", SONAR, "VCC", "WAGO +5 V", "", J, "5 V"),
    ("S17", SONAR, "GND", "GND bus", "", J, ""),
    ("S18", SONAR, "TRIG", ESP, gpio("PIN_SONAR_TRIG"), J, ""),
    ("S19", SONAR, "ECHO", ESP, gpio("PIN_SONAR_ECHO"), J, "through a 1k / 2k divider (5 V -> 3.3 V)"),
    ("S20", "Front servo SV1", "orange (signal)", ESP, gpio("PIN_SERVO_FRONT"), "servo lead", ""),
    ("S21", "Rear servo SV2", "orange (signal)", ESP, gpio("PIN_SERVO_REAR"), "servo lead",
     "nothing else on this pin (must be LOW at start-up)"),
    ("S22", ESP, "3V3", "3V3 bus", "", J, "feeds every 3.3 V part"),
    ("S22", ESP, "GND", "GND bus", "", J, ""),
]
for i, part in enumerate(LINE):
    SIGNAL += [("S23", part, "VCC", "3V3 bus", "", J, "3.3 V, not 5 V"), ("S23", part, "GND", "GND bus", "", J, "")]
for i, part in enumerate(LINE):
    SIGNAL.append(("S24", part, "OUT", ESP, gpio("PIN_LINE", i), J, ("left", "middle", "right")[i]))
SIGNAL += [
    ("S25", "Start button S2", "terminal 1", ESP, gpio("PIN_BUTTON"), J, "internal pull-up"),
    ("S25", "Start button S2", "terminal 2", "GND bus", "", J, ""),
    ("S26", "Green LED D1", "+ (long leg)", ESP, gpio("PIN_LED_GREEN"), J, "through 330 ohm"),
    ("S26", "Green LED D1", "- (short leg)", "GND bus", "", J, ""),
    ("S26", "Red LED D2", "+ (long leg)", ESP, gpio("PIN_LED_RED"), J, "through 330 ohm"),
    ("S26", "Red LED D2", "- (short leg)", "GND bus", "", J, ""),
    ("S27", "Buzzer module", "VCC", "3V3 bus", "", J, ""),
    ("S27", "Buzzer module", "GND", "GND bus", "", J, ""),
    ("S27", "Buzzer module", "I/O", ESP, gpio("PIN_BUZZER"), J, ""),
    ("S28", "Bus-sense divider", "top of 10k", "WAGO +12 V MOTOR", "", J, "after the E-stop"),
    ("S28", "Bus-sense divider", "bottom of 2.2k", "GND bus", "", J, "100 nF across the 2.2k"),
    ("S28", "Bus-sense divider", "junction", ESP, gpio("PIN_BUS_SENSE"), J,
     "about 2.3 V at 12.6 V; 0 V = E-stop pressed"),
]

ALL = POWER + SIGNAL

# Small parts soldered on the I/O perfboard (sheet E04)
IO_BOARD = [
    ("1 x 10k", f"MOTOR_EN (GPIO {P['PIN_MOTOR_EN']}) -> GND: motors off until the ESP32 has started"),
    ("4 x 10k", f"encoder lines GPIO {P['PIN_ENC_L_A']}, {P['PIN_ENC_L_B']}, {P['PIN_ENC_R_A']}, "
                f"{P['PIN_ENC_R_B']} -> 3V3 (these pins have no pull-ups inside)"),
    ("1k + 2k", "ultrasonic ECHO divider: ECHO - 1k - junction - 2k - GND; junction to the ESP32"),
    ("10k + 2.2k + 100 nF", f"bus sense: +12 V MOTOR - 10k - junction - 2.2k - GND; junction to GPIO "
                            f"{P['PIN_BUS_SENSE']}"),
    ("2 x 330", "in series with the green and red LEDs"),
    ("3 rows of linked pins", "3V3 bus, GND bus, 5 V (from WAGO +5 V)"),
]


def parts():
    """Every part, in order of first appearance."""
    seen = []
    for _, a, _, b, _, _, _ in ALL:
        for p in (a, b):
            if p not in seen:
                seen.append(p)
    return seen


def of(part):
    """Connections of one part: (wire ID, its pin, other part, other pin, wire, note)."""
    rows = []
    for wid, a, pa, b, pb, wire, note in ALL:
        if a == part:
            rows.append((wid, pa, b, pb, wire, note))
        elif b == part:
            rows.append((wid, pb, a, pa, wire, note))
    return rows
