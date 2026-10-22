#include "settings.h"

#include <Preferences.h>

Settings cfg;

namespace {

struct Entry {
  const char *name;   // also the key in flash (max 15 characters)
  float *value;
};

Entry entries[] = {
    {"wheel_radius", &cfg.wheel_radius}, {"wheel_sep", &cfg.wheel_sep}, {"ticks_per_rev", &cfg.ticks_per_rev},
    {"max_wheel", &cfg.max_wheel},       {"kf", &cfg.kf},               {"kp", &cfg.kp},
    {"ki", &cfg.ki},                     {"min_duty", &cfg.min_duty},   {"accel", &cfg.accel},
    {"inv_l_motor", &cfg.inv_l_motor},   {"inv_r_motor", &cfg.inv_r_motor},
    {"inv_l_enc", &cfg.inv_l_enc},       {"inv_r_enc", &cfg.inv_r_enc},
    {"scale", &cfg.scale},               {"sign_fl", &cfg.sign_fl},     {"sign_fr", &cfg.sign_fr},
    {"sign_rl", &cfg.sign_rl},           {"sign_rr", &cfg.sign_rr},     {"max_kg", &cfg.max_kg},
    {"min_factor", &cfg.min_factor},     {"overload", &cfg.overload},   {"load_kg", &cfg.load_kg},
    {"hold_s", &cfg.hold_s},
    {"line_black", &cfg.line_black},     {"follow_speed", &cfg.follow_speed}, {"k_line", &cfg.k_line},
    {"kd_line", &cfg.kd_line},           {"lost_s", &cfg.lost_s},       {"mark_min", &cfg.mark_min},
    {"stop_mm", &cfg.stop_mm},
    {"stop_cm", &cfg.stop_cm},           {"slow_cm", &cfg.slow_cm},     {"clear_s", &cfg.clear_s},
    {"tip_deg", &cfg.tip_deg},           {"tip_speed", &cfg.tip_speed},
    {"f_flat_us", &cfg.f_flat_us},       {"f_tip_us", &cfg.f_tip_us},
    {"r_flat_us", &cfg.r_flat_us},       {"r_tip_us", &cfg.r_tip_us},
    {"tip_hold_s", &cfg.tip_hold_s},     {"tof_check", &cfg.tof_check},
    {"tof_min_mm", &cfg.tof_min_mm},     {"tof_max_mm", &cfg.tof_max_mm},
};

Entry *find(const char *name) {
  for (Entry &e : entries)
    if (strcmp(e.name, name) == 0) return &e;
  return nullptr;
}

}  // namespace

void settingsLoad() {
  Preferences p;
  p.begin("cargo", true);                 // read-only; missing keys keep their default
  for (Entry &e : entries) *e.value = p.getFloat(e.name, *e.value);
  p.end();
}

bool settingsSave() {
  Preferences p;
  if (!p.begin("cargo", false)) return false;
  bool ok = true;
  for (Entry &e : entries) ok &= p.putFloat(e.name, *e.value) == sizeof(float);
  p.end();
  return ok;
}

void settingsDefaults() {
  cfg = Settings();
  Preferences p;
  p.begin("cargo", false);
  p.clear();
  p.end();
}

bool settingsSet(const char *name, float value) {
  Entry *e = find(name);
  if (!e) return false;
  *e->value = value;
  return true;
}

bool settingsGet(const char *name, float &value) {
  Entry *e = find(name);
  if (!e) return false;
  value = *e->value;
  return true;
}

void settingsList(Print &out) {
  for (Entry &e : entries) out.printf("  %-14s %g\n", e.name, *e.value);
}
