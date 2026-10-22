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

float payloadFactor(float kg, float maxKg, float minFactor, float overload) {
  if (kg <= 0) return 1;
  if (kg > maxKg * overload) return 0;
  return 1 - (1 - minFactor) * min(kg / maxKg, 1.0f);
}

float obstacleFactor(const float cm[3], float stopCm, float slowCm) {
  auto ramp = [&](float d) { return constrain((d - stopCm) / (slowCm - stopCm), 0.0f, 1.0f); };
  float centre = ramp(cm[1]);
  float sides = max(0.5f, ramp(min(cm[0], cm[2])));
  return min(centre, sides);
}

Line readLine(uint8_t bits, bool blackIsHigh) {
  Line l;
  l.black = blackIsHigh ? bits : (uint8_t)~bits;
  float sum = 0;
  for (int i = 0; i < 8; i++) {
    if (l.black & (1 << i)) {
      sum += (3.5f - i) / 3.5f;                   // sensor 1 -> +1 ... sensor 8 -> -1
      l.count++;
    }
  }
  if (l.count) l.pos = sum / l.count;
  return l;
}

float Median::update(float x) {
  buf_[next_] = x;
  next_ = (next_ + 1) % n_;
  if (count_ < n_) count_++;
  float s[9];
  memcpy(s, buf_, count_ * sizeof(float));
  for (int i = 1; i < count_; i++)                // insertion sort: at most 9 values
    for (int j = i; j > 0 && s[j - 1] > s[j]; j--) std::swap(s[j - 1], s[j]);
  return count_ % 2 ? s[count_ / 2] : (s[count_ / 2 - 1] + s[count_ / 2]) / 2;
}
