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

// 1.0 empty -> minFactor at maxKg -> 0.0 (refuse to move) above maxKg * overload.
float payloadFactor(float kg, float maxKg, float minFactor, float overload);

// 1.0 when the way is clear -> 0.0 at stopCm. Side sensors only slow the robot (to 50 %), never stop it,
// so the conveyor and walls beside the line do not stop it.
float obstacleFactor(const float cm[3], float stopCm, float slowCm);

// The 8 line sensors as a position: +1 = line under the leftmost sensor, -1 = rightmost, 0 = centred.
struct Line {
  int count = 0;                   // sensors on black
  float pos = 0;
  uint8_t black = 0;               // bit i = sensor i + 1 is on black
};
Line readLine(uint8_t bits, bool blackIsHigh);

// Median of the last n values (n <= 9): removes the odd wild load-cell reading.
class Median {
 public:
  explicit Median(int n) : n_(n) {}
  float update(float x);
  void clear() { count_ = 0; }

 private:
  float buf_[9];
  int n_, count_ = 0, next_ = 0;
};
