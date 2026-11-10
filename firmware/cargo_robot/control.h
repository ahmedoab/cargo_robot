// Maths used by the main program (no hardware in here).
#pragma once
#include <Arduino.h>

// Wheel speed controller: feed-forward + PI. Target and measurement in rad/s, output = motor duty -1..1.
//   duty = sign(target) * (min_duty + kf * |target|) + kp * error + integral
struct WheelPID {
  float integral = 0;
  float update(float target, float measured, float dt, float kf, float kp, float ki, float minDuty);
  void reset() { integral = 0; }
};

struct LowPass {
  float alpha, value = 0;
  explicit LowPass(float a) : alpha(a) {}
  float update(float x) { return value += alpha * (x - value); }
};

// Distance travelled (m) and heading change (rad, + = turned left) from the distance each wheel rolled.
struct Odometry {
  float distance = 0, heading = 0;
  void update(float dLeft, float dRight, float wheelSep);
};

// Robot speed (v m/s forward, w rad/s turning left) -> wheel speeds (rad/s), scaled down together if too fast.
void twistToWheels(float v, float w, float wheelSep, float wheelRadius, float maxWheel, float &left, float &right);

// 1.0 when the way ahead is clear (beyond slowCm) -> 0.0 at stopCm and closer.
float obstacleFactor(float cm, float stopCm, float slowCm);

// The line sensors as a position: +1 = line under the leftmost sensor, -1 = rightmost, 0 = centred.
struct Line {
  int count = 0;                   // sensors on black
  float pos = 0;
  uint8_t black = 0;               // bit i = sensor i + 1 is on black
};
Line readLine(uint8_t bits, int n, bool blackIsHigh);   // n sensors, bit i = sensor i + 1
