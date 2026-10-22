// Hardware drivers for the cargo robot's ESP32.
//
//   Motor       one BTS7960 channel (RPWM = forward, LPWM = reverse), hardware PWM
//   Encoder     quadrature counter in the ESP32's pulse-counter hardware (x4 decoding, no CPU load)
//   HX711Array  the 4 HX711 boards on one shared clock line
//   Expander    MCP23017 over I2C: line sensors, motor enable, buzzer, LEDs, start button
//   sonar::     the HC-SR04 in front; echo time measured with an interrupt
//   BedServo    one servo signal (50 Hz pulses); no pulses = the servo goes limp
//   sideMm()    VL53L0X distance to whatever is beside the robot's right side
//   busVolts()  motor 12 V bus, through the divider
//
// Pin numbers come from config.h.
#pragma once
#include <Arduino.h>
#include "driver/pulse_cnt.h"

class Motor {
 public:
  void begin(int pinFwd, int pinRev);
  void set(float duty);            // -1..1; 0 = brake
  bool invert = false;

 private:
  int fwd_ = -1, rev_ = -1;
};

class Encoder {
 public:
  bool begin(int pinA, int pinB);
  long count();                    // x4 counts since start, negative when turning backwards
  bool invert = false;

 private:
  pcnt_unit_handle_t unit_ = nullptr;
};

class HX711Array {
 public:
  void begin();
  int notReady();                  // bit i set = board i has no new reading yet
  bool read(long out[4], uint32_t timeoutMs = 300);   // signed 24-bit reading of every board
};

class Expander {
 public:
  bool begin();                    // Wire must already be running
  uint8_t lineBits();              // bit i = output level of line sensor i + 1
  bool buttonPressed();
  void out(int pin, bool on);      // XP_MOTOR_EN, XP_BUZZER, XP_LED_GREEN, XP_LED_RED
  bool ok = false;

 private:
  bool write(uint8_t reg, uint8_t value);
  int read(uint8_t reg);
  uint8_t olatB_ = 0;
};

namespace sonar {
void begin();
void update();                     // call often: pings every SONAR_PERIOD_MS
float cm();                        // distance straight ahead; SONAR_MAX_CM = nothing there
}  // namespace sonar

class BedServo {
 public:
  void begin(int pin);
  void pulse(float us);            // 500..2500 us
  void off();
  bool on() const { return on_; }

 private:
  int pin_ = -1;
  bool on_ = false;
};

bool sideBegin();
int sideMm();                      // latest reading, -1 = no sensor or no reading
float busVolts();
