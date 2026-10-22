// Tunable numbers, kept in the ESP32's flash so they survive a power cycle.
//
// Change them in the Serial Monitor:   set kp 0.03   (takes effect at once)   then   save   (keeps it).
// "list" prints them all, "defaults" goes back to the values below. Switches use 1 = yes, 0 = no.
#pragma once
#include <Arduino.h>

struct Settings {
  // ---- drive (week 3 bench tests, week 5) ----
  float wheel_radius = 0.05f;     // m - fine-tune with the 3 m straight-line test
  float wheel_sep = 0.33f;        // m, middle of the left tyre to middle of the right tyre - 5-turn spin test
  float ticks_per_rev = 2464;     // encoder counts per wheel turn - measure with "enc"
  float max_wheel = 12.0f;        // rad/s (12 x 0.05 m = 0.6 m/s)
  float kf = 0.05f;               // duty per rad/s - from "maxtest"
  float kp = 0.02f;
  float ki = 0.10f;
  float min_duty = 0.10f;         // duty that just starts a wheel - from "mintest"
  float accel = 0.4f;             // m/s^2: gentle, so the package does not slide
  float inv_l_motor = 0, inv_r_motor = 0, inv_l_enc = 0, inv_r_enc = 0;
  // ---- payload (week 3 calibration, week 7) ----
  float scale = 22000;            // raw counts (sum of the 4 cells) per kg - from "cal"
  float sign_fl = 1, sign_fr = 1, sign_rl = 1, sign_rr = 1;   // -1 for a cell wired "upside down"
  float max_kg = 3.0f;            // the speed has dropped to min_factor at this load
  float min_factor = 0.5f;
  float overload = 1.2f;          // refuse to move above max_kg x overload
  float load_kg = 0.15f;          // this much on the bed counts as "a package is on"
  float hold_s = 2.0f;            // ... for this long
  // ---- line following (week 6) ----
  float line_black = 1;           // 1 = a sensor output is HIGH over black tape, 0 = LOW over black
  float follow_speed = 0.20f;     // m/s, empty bed
  float k_line = 2.5f;            // rad/s of turning per unit of line error (-1..+1)
  float kd_line = 0.15f;
  float lost_s = 0.5f;            // stop if the line is gone this long
  float mark_min = 6;             // this many sensors on black at once = a station mark
  float stop_mm = 214;            // drive on this far after a mark: sensor bar -> middle of the bed
  // ---- obstacles (week 7) ----
  float stop_cm = 25;             // centre sensor: stop below this
  float slow_cm = 60;             // all sensors: slow down below this
  float clear_s = 1.0f;           // the way must be clear this long before driving on
  // ---- tilting bed (week 4, week 8) ----
  float tip_deg = 40;             // how far the bed tips to drop the package
  float tip_speed = 20;           // deg/s
  float f_flat_us = 1500, f_tip_us = 1944;   // front servo pulse, bed flat / bed at tip_deg - set with "servo"
  float r_flat_us = 1500, r_tip_us = 1056;   // rear servo: a mirror image, so it turns the other way
  float tip_hold_s = 2;           // seconds the bed stays tipped before it comes down
  float tof_check = 1;            // 1 = only tip when something (the belt) is beside the robot's right side
  float tof_min_mm = 20, tof_max_mm = 150;
};

extern Settings cfg;

void settingsLoad();
bool settingsSave();
void settingsDefaults();
bool settingsSet(const char *name, float value);
bool settingsGet(const char *name, float &value);
void settingsList(Print &out);
