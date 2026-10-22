#include "robot.h"

#include <Wire.h>

#include "config.h"
#include "settings.h"

Motor motorL, motorR;
Encoder encL, encR;
HX711Array cells;
Expander io;
BedServo servoFront, servoRear;
bool manualMotors = false;
Readings rd;

namespace {
WheelPID pidL, pidR;
LowPass speedL(0.5f), speedR(0.5f);
Median weightFilter(5);
long offsets[4] = {0, 0, 0, 0};
long lastCountL = 0, lastCountR = 0;
uint32_t lastTick = 0, lastCellRead = 0, lastSide = 0, beepUntil = 0;
float targetV = 0, targetW = 0, rampV = 0;
float bedTarget = 0;
bool motorsOn = false;
Light greenMode = Light::Off, redMode = Light::Off;

float sign(int i) {
  const float s[4] = {cfg.sign_fl, cfg.sign_fr, cfg.sign_rl, cfg.sign_rr};
  return s[i] < 0 ? -1.0f : 1.0f;
}

bool lit(Light mode) {
  switch (mode) {
    case Light::On: return true;
    case Light::Slow: return millis() % 1000 < 150;
    case Light::Fast: return millis() % 300 < 150;
    default: return false;
  }
}

void writeServos() {
  float k = cfg.tip_deg != 0 ? rd.bedDeg / cfg.tip_deg : 0;
  servoFront.pulse(cfg.f_flat_us + (cfg.f_tip_us - cfg.f_flat_us) * k);
  servoRear.pulse(cfg.r_flat_us + (cfg.r_tip_us - cfg.r_flat_us) * k);
}

void controlTick(float dt) {
  // wheels: encoder counts -> speeds and odometry
  long cl = encL.count(), cr = encR.count();
  float radPerTick = 2 * PI / cfg.ticks_per_rev;
  float dl = (cl - lastCountL) * radPerTick, dr = (cr - lastCountR) * radPerTick;
  lastCountL = cl;
  lastCountR = cr;
  rd.wheelL = speedL.update(dl / dt);
  rd.wheelR = speedR.update(dr / dt);
  rd.odom.update(dl * cfg.wheel_radius, dr * cfg.wheel_radius, cfg.wheel_sep);

  // speed ramp: gentle starts and stops, so the package does not slide
  float step = cfg.accel * dt;
  rampV += constrain(targetV - rampV, -step, step);
  if (!manualMotors) {
    float left = 0, right = 0;
    if (motorsOn) twistToWheels(rampV, targetW, cfg.wheel_sep, cfg.wheel_radius, cfg.max_wheel, left, right);
    motorL.set(pidL.update(left, rd.wheelL, dt, cfg.kf, cfg.kp, cfg.ki, cfg.min_duty));
    motorR.set(pidR.update(right, rd.wheelR, dt, cfg.kf, cfg.kp, cfg.ki, cfg.min_duty));
  }

  // line sensors
  rd.line = readLine(io.lineBits(), cfg.line_black != 0);

  // bed: move towards its target at tip_speed
  if (servoFront.on()) {
    float s = cfg.tip_speed * dt;
    rd.bedDeg += constrain(bedTarget - rd.bedDeg, -s, s);
    writeServos();
  }
}
}  // namespace

void robotBegin() {
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  if (!io.begin()) Serial.println("!! MCP23017 not found at 0x20: check SDA/SCL, 3.3 V and A0-A2 to GND");
  motorL.begin(PIN_L_RPWM, PIN_L_LPWM);
  motorR.begin(PIN_R_RPWM, PIN_R_LPWM);
  if (!encL.begin(PIN_ENC_L_A, PIN_ENC_L_B) || !encR.begin(PIN_ENC_R_A, PIN_ENC_R_B))
    Serial.println("!! encoder counter set-up failed");
  applyInversions();
  cells.begin();
  sonar::begin();
  if (!sideBegin()) Serial.println("!! VL53L0X side sensor not found: check its wires (or set tof_check 0)");
  servoFront.begin(PIN_SERVO_FRONT);
  servoRear.begin(PIN_SERVO_REAR);
  motorsEnable(false);
  lastTick = micros();
}

void applyInversions() {
  motorL.invert = cfg.inv_l_motor != 0;
  motorR.invert = cfg.inv_r_motor != 0;
  encL.invert = cfg.inv_l_enc != 0;
  encR.invert = cfg.inv_r_enc != 0;
  lastCountL = encL.count();
  lastCountR = encR.count();
}

void robotUpdate() {
  rd.busV = busVolts();

  // load cells: a new reading 10 times per second
  if (cells.notReady() == 0) {
    long raw[4];
    if (cells.read(raw, 0)) {
      float total = 0;
      for (int i = 0; i < 4; i++) {
        rd.raw[i] = raw[i];
        rd.cornerKg[i] = sign(i) * (raw[i] - offsets[i]) / cfg.scale;
        total += rd.cornerKg[i];
      }
      rd.kg = weightFilter.update(total);
      lastCellRead = millis();
    }
  }
  rd.cellsOk = millis() - lastCellRead < 1000;

  sonar::update();
  rd.sonarCm = sonar::cm();
  if (millis() - lastSide >= 20) {
    lastSide = millis();
    rd.sideMm = sideMm();
  }

  uint32_t now = micros();
  if (now - lastTick >= 1000000 / CONTROL_HZ) {
    float dt = (now - lastTick) * 1e-6f;
    lastTick = now;
    controlTick(dt);
  }

  io.out(XP_LED_GREEN, lit(greenMode));
  io.out(XP_LED_RED, lit(redMode));
  io.out(XP_BUZZER, (int32_t)(beepUntil - millis()) > 0);
}

void driveTarget(float v, float w) {
  targetV = v;
  targetW = w;
}

void driveStop() {
  targetV = targetW = rampV = 0;
  pidL.reset();
  pidR.reset();
  motorL.set(0);
  motorR.set(0);
}

void motorsEnable(bool on) {
  if (!on) driveStop();
  motorsOn = on;
  io.out(XP_MOTOR_EN, on);
}

bool tare() {
  long sum[4] = {0, 0, 0, 0};
  int n = 0;
  for (int k = 0; k < 15; k++) {
    long raw[4];
    if (!cells.read(raw)) continue;
    for (int i = 0; i < 4; i++) sum[i] += raw[i];
    n++;
  }
  if (n < 8) return false;
  for (int i = 0; i < 4; i++) offsets[i] = sum[i] / n;
  weightFilter.clear();
  rd.kg = 0;
  return true;
}

void bedMove(float deg) {
  if (!servoFront.on()) {                         // waking up: the bed is resting flat on its posts
    rd.bedDeg = 0;
    writeServos();
  }
  bedTarget = constrain(deg, 0.0f, 45.0f);
}

bool bedMoving() { return servoFront.on() && fabsf(rd.bedDeg - bedTarget) > 0.1f; }

void bedRelax() {
  servoFront.off();
  servoRear.off();
  rd.bedDeg = bedTarget = 0;
}

void lights(Light green, Light red) {
  greenMode = green;
  redMode = red;
}

void beep(int ms) { beepUntil = millis() + ms; }
