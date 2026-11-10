#include "robot.h"

#include "config.h"
#include "settings.h"

Motor motorL, motorR;
Encoder encL, encR;
BedServo servoFront, servoRear;
bool manualMotors = false;
Readings rd;

namespace {
WheelPID pidL, pidR;
LowPass speedL(0.5f), speedR(0.5f);
long lastCountL = 0, lastCountR = 0;
uint32_t lastTick = 0, beepUntil = 0;
float targetV = 0, targetW = 0, rampV = 0;
float bedTarget = 0;
bool motorsOn = false;
Light greenMode = Light::Off, redMode = Light::Off;

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
  rd.line = readLine(lineBits(), LINE_N, cfg.line_black != 0);

  // bed: move towards its target at tip_speed
  if (servoFront.on()) {
    float s = cfg.tip_speed * dt;
    rd.bedDeg += constrain(bedTarget - rd.bedDeg, -s, s);
    writeServos();
  }
}
}  // namespace

void robotBegin() {
  panelBegin();
  motorL.begin(PIN_L_RPWM, PIN_L_LPWM);
  motorR.begin(PIN_R_RPWM, PIN_R_LPWM);
  if (!encL.begin(PIN_ENC_L_A, PIN_ENC_L_B) || !encR.begin(PIN_ENC_R_A, PIN_ENC_R_B))
    Serial.println("!! encoder counter set-up failed");
  applyInversions();
  sonar::begin();
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
  sonar::update();
  rd.sonarCm = sonar::cm();

  uint32_t now = micros();
  if (now - lastTick >= 1000000 / CONTROL_HZ) {
    float dt = (now - lastTick) * 1e-6f;
    lastTick = now;
    controlTick(dt);
  }

  panelOut(PIN_LED_GREEN, lit(greenMode));
  panelOut(PIN_LED_RED, lit(redMode));
  panelOut(PIN_BUZZER, (int32_t)(beepUntil - millis()) > 0);
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
  panelOut(PIN_MOTOR_EN, on);
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
