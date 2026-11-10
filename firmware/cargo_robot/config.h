// Cargo robot - ESP32 wiring and fixed numbers.
//
// This file is the single source of truth for the wiring: the wire list in BUILD_GUIDE.md (week 3) and the
// schematics (cad/make_schematics.py reads this file) follow it. If you move a wire, change it here too.
// Numbers are the GPIO numbers printed on the ESP32 DevKit / screw-terminal breakout ("GPIO25" or just "25").
// Everything is wired straight to the ESP32: there is no I/O expander and no I2C.
#pragma once

// ---------------- ESP32 pins ----------------
// Drive: 2 x BTS7960 (IBT-2). RPWM = forward, LPWM = reverse. R_EN + L_EN of both come from PIN_MOTOR_EN.
constexpr int PIN_L_RPWM = 25;
constexpr int PIN_L_LPWM = 26;
constexpr int PIN_R_RPWM = 27;
constexpr int PIN_R_LPWM = 14;
constexpr int PIN_MOTOR_EN = 13;         // -> R_EN + L_EN of both drivers; 10k pull-down to GND: off at power-up
// Wheel encoders (JGB37-520 Hall, powered from 3.3 V). Input-only pins: each needs a 10k pull-up to 3.3 V.
constexpr int PIN_ENC_L_A = 34;
constexpr int PIN_ENC_L_B = 35;
constexpr int PIN_ENC_R_A = 36;          // marked "VP" on most boards
constexpr int PIN_ENC_R_B = 39;          // marked "VN"
// HC-SR04 ultrasonic sensor, looking straight ahead; ECHO through a 1k / 2k divider (5 V -> 3.3 V)
constexpr int PIN_SONAR_TRIG = 4;
constexpr int PIN_SONAR_ECHO = 23;
// Bed servos: signal wire only (their power comes from the 7.4 V servo buck converter)
constexpr int PIN_SERVO_FRONT = 2;
constexpr int PIN_SERVO_REAR = 12;       // GPIO12 must be LOW while the ESP32 starts: connect nothing else to it
// 3 IR line sensors (3-pin obstacle-avoidance modules, powered from 3.3 V), left to right
constexpr int LINE_N = 3;
constexpr int PIN_LINE[LINE_N] = {16, 17, 18};   // sensor 1 = left, 2 = middle, 3 = right
// Panel
constexpr int PIN_BUTTON = 19;           // <- start button to GND (internal pull-up)
constexpr int PIN_LED_GREEN = 21;        // -> green LED + 330 ohm to GND
constexpr int PIN_LED_RED = 22;          // -> red LED + 330 ohm to GND
constexpr int PIN_BUZZER = 32;           // -> active buzzer module, I/O pin
// Motor 12 V (after the E-stop) through a 10k / 2.2k divider: battery voltage, about 0 V = E-stop pressed
constexpr int PIN_BUS_SENSE = 33;
// Free: GPIO 0, 5 and 15. They are start-up (strapping) pins: only use them for something that can't pull them
// the wrong way while the ESP32 starts.

// ---------------- fixed numbers ----------------
constexpr float BUS_DIVIDER = (10.0f + 2.2f) / 2.2f;   // motor-bus volts per volt at PIN_BUS_SENSE
constexpr float BUS_ON_V = 7.0f;         // below this the motors have no power: E-stop pressed or battery out
constexpr int CONTROL_HZ = 100;          // wheel speed control and line following
constexpr int DRIVE_PWM_HZ = 20000;      // BTS7960 PWM, above hearing
constexpr int SONAR_PERIOD_MS = 60;      // one ultrasonic ping
constexpr float SONAR_MAX_CM = 400.0f;   // reported when nothing echoes back
