#include "control.h"

float WheelPID::update(float target, float measured, float dt, float kf, float kp, float ki, float minDuty) {
  if (fabsf(target) < 1e-3f) {
    reset();
    return 0;
  }
  float error = target - measured;
  integral = constrain(integral + ki * error * dt, -0.3f, 0.3f);
  float ff = copysignf(minDuty + kf * fabsf(target), target);
  return constrain(ff + kp * error + integral, -1.0f, 1.0f);
}

void Odometry::update(float dLeft, float dRight, float wheelSep) {
  distance += (dLeft + dRight) / 2;
  heading += (dRight - dLeft) / wheelSep;
}

void twistToWheels(float v, float w, float wheelSep, float wheelRadius, float maxWheel, float &left, float &right) {
  left = (v - w * wheelSep / 2) / wheelRadius;
  right = (v + w * wheelSep / 2) / wheelRadius;
  float biggest = max(fabsf(left), fabsf(right));
  if (biggest > maxWheel) {                       // keep the curve, just slower
    left *= maxWheel / biggest;
    right *= maxWheel / biggest;
  }
}

float obstacleFactor(float cm, float stopCm, float slowCm) {
  return constrain((cm - stopCm) / (slowCm - stopCm), 0.0f, 1.0f);
}

Line readLine(uint8_t bits, int n, bool blackIsHigh) {
  Line l;
  l.black = (blackIsHigh ? bits : (uint8_t)~bits) & ((1 << n) - 1);
  float mid = (n - 1) / 2.0f, sum = 0;
  for (int i = 0; i < n; i++) {
    if (l.black & (1 << i)) {
      sum += (mid - i) / mid;                     // sensor 1 -> +1 ... sensor n -> -1
      l.count++;
    }
  }
  if (l.count) l.pos = sum / l.count;
  return l;
}
