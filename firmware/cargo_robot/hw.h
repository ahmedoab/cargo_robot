// Hardware drivers for the cargo robot's ESP32.
//
//   Motor       one BTS7960 channel (RPWM = forward, LPWM = reverse), hardware PWM
//   Encoder     quadrature counter in the ESP32's pulse-counter hardware (x4 decoding, no CPU load)
//   panel*()    the 3 line sensors, start button, LEDs, buzzer and motor enable: plain ESP32 pins
//   sonar::     the HC-SR04 in front; echo time measured with an interrupt
//   BedServo    one servo signal (50 Hz pulses); no pulses = the servo goes limp
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

void panelBegin();                 // set the pins up: motors disabled, LEDs and buzzer off
uint8_t lineBits();                // bit i = output level of line sensor i + 1
bool buttonDown();                 // the start button is held
void panelOut(int pin, bool on);   // PIN_MOTOR_EN, PIN_BUZZER, PIN_LED_GREEN, PIN_LED_RED

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

float busVolts();
