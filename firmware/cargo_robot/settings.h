// Tunable numbers, kept in the ESP32's flash so they survive a power cycle.
//
// Change them in the Serial Monitor:   set kp 0.03   (takes effect at once)   then   save   (keeps it).
// "list" prints them all, "defaults" goes back to the values below. Switches use 1 = yes, 0 = no.
#pragma once
#include <Arduino.h>

struct Settings {
  // ---- drive (week 3 bench tests, week 5) ----
  float wheel_radius = 0.0325f;   // m (65 mm wheels) - fine-tune with the 3 m straight-line test
  float wheel_sep = 0.356f;       // m, middle of the left tyre to middle of the right tyre - 5-turn spin test
  float ticks_per_rev = 3960;     // encoder counts per wheel turn (about 11 x 4 x 90) - measure with "enc"
  float max_wheel = 8.5f;         // rad/s: what the 110 RPM motors manage loaded (8.5 x 0.0325 m = 0.28 m/s)
  float kf = 0.09f;               // duty per rad/s - from "maxtest"
  float kp = 0.02f;
  float ki = 0.10f;
  float min_duty = 0.10f;         // duty that just starts a wheel - from "mintest"
  float accel = 0.4f;             // m/s^2: gentle, so the package does not slide
  float inv_l_motor = 0, inv_r_motor = 0, inv_l_enc = 0, inv_r_enc = 0;
  // ---- line following (week 6) ----
  float line_black = 1;           // 1 = a sensor output is HIGH over black tape, 0 = LOW over black
  float follow_speed = 0.20f;     // m/s, empty bed
  float k_line = 2.5f;            // rad/s of turning per unit of line error (-1..+1)
  float kd_line = 0.15f;
  float lost_s = 0.5f;            // stop if the line is gone this long
  float mark_min = 3;             // this many sensors on black at once = a station mark (all 3)
  float stop_mm = 98;             // drive on this far after a mark: sensor bar -> middle of the bed
  // ---- obstacles (week 7) ----
  float stop_cm = 25;             // something closer than this ahead: stop
  float slow_cm = 60;             // ... closer than this: slow down
  float clear_s = 1.0f;           // the way must be clear this long before driving on
  // ---- tilting bed (week 4, week 8) ----
  float tip_deg = 40;             // how far the bed tips to drop the package
  float tip_speed = 20;           // deg/s
  float f_flat_us = 1500, f_tip_us = 1944;   // front servo pulse, bed flat / bed at tip_deg - set with "servo"
  float r_flat_us = 1500, r_tip_us = 1056;   // rear servo: a mirror image, so it turns the other way
  float tip_hold_s = 2;           // seconds the bed stays tipped before it comes down
};

extern Settings cfg;

void settingsLoad();
bool settingsSave();
void settingsDefaults();
bool settingsSet(const char *name, float value);
bool settingsGet(const char *name, float &value);
void settingsList(Print &out);
