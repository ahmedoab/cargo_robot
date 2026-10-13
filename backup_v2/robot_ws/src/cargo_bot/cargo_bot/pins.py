"""Raspberry Pi wiring - BCM GPIO numbers (physical header pin in the comment).

This is the single source of truth for the wiring table in BUILD_GUIDE.md.
If you move a wire, change it here (or override the matching ROS parameter).
"""

PINS = {
    # BTS7960 motor drivers (both drivers' R_EN + L_EN tied together)
    "enable": 5,            # pin 29
    "left_fwd": 12,         # pin 32 -> left  BTS7960 RPWM
    "left_rev": 13,         # pin 33 -> left  BTS7960 LPWM
    "right_fwd": 18,        # pin 12 -> right BTS7960 RPWM
    "right_rev": 19,        # pin 35 -> right BTS7960 LPWM
    # Wheel encoders (powered from 3.3 V!)
    "left_enc_a": 17,       # pin 11
    "left_enc_b": 27,       # pin 13
    "right_enc_a": 22,      # pin 15
    "right_enc_b": 23,      # pin 16
    # HX711 load-cell amplifiers: one shared clock, one data line each
    "hx711_sck": 24,        # pin 18 -> SCK of all four boards
    "hx711_dout": [25, 16, 20, 21],   # pins 22, 36, 38, 40 -> front-left, front-right, rear-left, rear-right
    # Motor-power sense (10k / 3.3k divider after the E-stop): 1 = motors powered, 0 = E-stop pressed
    "estop_sense": 26,      # pin 37
}

CORNERS = ["front_left", "front_right", "rear_left", "rear_right"]
