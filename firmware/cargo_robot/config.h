// Cargo robot - ESP32 wiring and fixed numbers.
//
// This file is the single source of truth for the wiring: the wire list in BUILD_GUIDE.md (week 3) and the
// schematics (cad/make_schematics.py reads this file) follow it. If you move a wire, change it here too.
// Numbers are the GPIO numbers printed on the ESP32 DevKit / screw-terminal breakout ("GPIO25" or just "25").
#pragma once

// ---------------- ESP32 pins ----------------
// Drive: 2 x BTS7960 (IBT-2). RPWM = forward, LPWM = reverse. R_EN + L_EN come from the MCP23017 (XP_MOTOR_EN).
constexpr int PIN_L_RPWM = 25;
constexpr int PIN_L_LPWM = 26;
constexpr int PIN_R_RPWM = 27;
constexpr int PIN_R_LPWM = 14;
// Wheel encoders (JGB37-520 Hall, powered from 3.3 V). Input-only pins: each needs a 10k pull-up to 3.3 V.
constexpr int PIN_ENC_L_A = 34;
constexpr int PIN_ENC_L_B = 35;
constexpr int PIN_ENC_R_A = 36;          // marked "VP" on most boards
constexpr int PIN_ENC_R_B = 39;          // marked "VN"
// 4 x HX711 load-cell amplifiers: one shared clock, one data line each
constexpr int PIN_HX_SCK = 5;
constexpr int PIN_HX_DT[4] = {16, 17, 18, 19};   // front-left, front-right, rear-left, rear-right
// HC-SR04 ultrasonic sensor, looking straight ahead; ECHO through a 1k / 2k divider (5 V -> 3.3 V)
constexpr int PIN_SONAR_TRIG = 4;
constexpr int PIN_SONAR_ECHO = 23;
// Bed servos: signal wire only (their power comes from the 7.4 V servo buck converter)
constexpr int PIN_SERVO_FRONT = 2;
constexpr int PIN_SERVO_REAR = 12;       // GPIO12 must be LOW while the ESP32 starts: connect nothing else to it
// I2C: MCP23017 I/O expander (0x20) + VL53L0X side distance sensor (0x29), 4.7k pull-ups to 3.3 V
constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;
// Motor 12 V (after the E-stop) through a 10k / 2.2k divider: battery voltage, about 0 V = E-stop pressed
constexpr int PIN_BUS_SENSE = 33;

// ---------------- MCP23017 pins (0-7 = GPA0-GPA7, 8-15 = GPB0-GPB7) ----------------
constexpr int MCP_ADDR = 0x20;           // A0, A1, A2 tied to GND
constexpr int XP_LINE_FIRST = 0;         // GPA0..GPA7 <- line sensors 1 (leftmost) .. 8 (rightmost)
constexpr int XP_MOTOR_EN = 8;           // GPB0 -> R_EN + L_EN of both BTS7960 (10k pull-down: off at power-up)
constexpr int XP_BUZZER = 9;             // GPB1 -> active buzzer module
constexpr int XP_LED_GREEN = 10;         // GPB2 -> green LED + 330 ohm
constexpr int XP_LED_RED = 11;           // GPB3 -> red LED + 330 ohm
constexpr int XP_BUTTON = 12;            // GPB4 <- start button to GND (internal pull-up)

// ---------------- fixed numbers ----------------
constexpr float BUS_DIVIDER = (10.0f + 2.2f) / 2.2f;   // motor-bus volts per volt at PIN_BUS_SENSE
constexpr float BUS_ON_V = 7.0f;         // below this the motors have no power: E-stop pressed or battery out
constexpr int CONTROL_HZ = 100;          // wheel speed control and line following
constexpr int DRIVE_PWM_HZ = 20000;      // BTS7960 PWM, above hearing
constexpr int SONAR_PERIOD_MS = 60;      // one ultrasonic ping
constexpr float SONAR_MAX_CM = 400.0f;   // reported when nothing echoes back
