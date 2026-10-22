// Cargo robot - main program for the ESP32 (Arduino IDE 2, board "ESP32 Dev Module").
//
// The robot follows a black tape line. Stations are short strips of tape across the line ("station marks"):
// the pickup and the drop-off (the conveyor, on the robot's RIGHT). It carries one package on its tilting bed.
//
//   LOAD     at the pickup: wait until a package is on the bed (weight > load_kg for hold_s)
//   FOLLOW   follow the line to the next station mark: slower when loaded, slow or stop for obstacles
//   DOCK     drive on stop_mm so the middle of the bed is level with the mark, then stop
//   CHECK    at the drop-off: the side sensor must see the belt beside the robot (tof_check)
//   TIP      tip the bed to tip_deg (at tip_speed); the package slides off to the right
//   HOLD     keep it tipped for tip_hold_s
//   LOWER    lower the bed, switch the servos off, weigh: empty -> zero the scale and FOLLOW on to the pickup
// Always:  E-stop pressed -> everything stops.  Overload -> refuse to move.  Line lost, no belt or package still
//          on the bed -> ALARM: red LED + beeps; fix it, then press the start button to carry on.
//
// Start a mission with the start button or "run" in the Serial Monitor (115200 baud, Newline).
// "help" lists the test and calibration commands. Wiring: config.h. Settings: settings.h.
#include "config.h"
#include "console.h"
#include "robot.h"
#include "settings.h"

enum State { IDLE, LOAD, FOLLOW, DOCK, CHECK, TIP, HOLD, LOWER, ALARM, ESTOP };
const char *NAMES[] = {"IDLE", "LOAD", "FOLLOW", "DOCK", "CHECK", "TIP", "HOLD", "LOWER", "ALARM", "ESTOP"};

State state = IDLE, resumeTo = IDLE;    // resumeTo: where ALARM goes back to when the button is pressed
bool toDropoff = true;                  // the next station mark is the drop-off (else the pickup)
int deliveriesLeft = 0;                 // 0 = until stopped
uint32_t since = 0, condSince = 0;      // when the state started / when the condition we wait for became true
float dockStart = 0;
bool buttonWas = false;
bool relaxWhenFlat = false;             // after "stop": lower the bed, then switch the servos off

const char *stateName() { return NAMES[state]; }

void go(State s) {
  state = s;
  since = millis();
  condSince = 0;
  Serial.printf("[%s]\n", NAMES[s]);
}

// true once cond has held for seconds without a break
bool heldFor(bool cond, float seconds) {
  if (!cond) {
    condSince = 0;
    return false;
  }
  if (!condSince) condSince = millis();
  return millis() - condSince >= seconds * 1000;
}

void alarm(const char *why, State retry) {
  driveStop();
  motorsEnable(false);
  resumeTo = retry;
  Serial.printf("!! ALARM: %s - fix it, then press the start button\n", why);
  go(ALARM);
}

void startMission(int deliveries) {
  if (rd.busV < BUS_ON_V) {
    Serial.println("motor power is off: release the E-stop first");
    return;
  }
  deliveriesLeft = deliveries;
  bedRelax();
  Serial.println(deliveries ? "mission started" : "mission started (until stopped)");
  go(LOAD);
}

void stopMission() {
  driveStop();
  motorsEnable(false);
  if (servoFront.on()) {
    bedMove(0);
    relaxWhenFlat = true;
  }
  go(IDLE);
}

bool buttonPressed() {                  // one press = one event
  bool now = io.buttonPressed();
  bool edge = now && !buttonWas;
  buttonWas = now;
  return edge;
}

// ---------------- line following ----------------
bool markArmed = true;
int markTicks = 0;
float lastPos = 0, lastPosTime = 0;
uint32_t lostSince = 0, obstacleSince = 0;

// Steer along the line at the speed allowed by the load and the obstacles. Returns true when a station mark
// has just been crossed (6 or more sensors on black for 2 readings in a row).
bool follow() {
  const Line &l = rd.line;
  if (l.count == 0) {
    if (!lostSince) lostSince = millis();
    if (millis() - lostSince > cfg.lost_s * 1000) {
      alarm("line lost", state);
      return false;
    }
  } else {
    lostSince = 0;
  }

  float load = payloadFactor(rd.kg, cfg.max_kg, cfg.min_factor, cfg.overload);
  if (load == 0) {
    alarm("overload - take some weight off", state);
    return false;
  }
  float clear = obstacleFactor(rd.sonarCm, cfg.stop_cm, cfg.slow_cm);
  if (clear == 0) {                     // something in front: stop and ask for it to be moved
    if (!obstacleSince) Serial.println("obstacle - please remove it");
    obstacleSince = millis();
    lights(Light::On, Light::Fast);
    if (millis() % 1000 < 30) beep(120);
  } else if (obstacleSince && millis() - obstacleSince < cfg.clear_s * 1000) {
    clear = 0;                          // wait until the way has been clear for clear_s
  } else {
    obstacleSince = 0;
    lights(Light::On, Light::Off);
  }

  float pos = l.count ? l.pos : lastPos;  // line just lost: keep turning the way it went
  float now = millis() / 1000.0f;
  float dpos = (now > lastPosTime && lastPosTime > 0) ? (pos - lastPos) / (now - lastPosTime) : 0;
  lastPos = pos;
  lastPosTime = now;
  float v = cfg.follow_speed * load * clear;
  float w = (cfg.k_line * pos + cfg.kd_line * dpos) * (v / cfg.follow_speed);
  driveTarget(v, w);

  if (l.count >= cfg.mark_min) {
    if (markArmed && ++markTicks >= 2) {
      markArmed = false;
      return true;
    }
  } else {
    markTicks = 0;
    if (l.count <= 3) markArmed = true;
  }
  return false;
}

// ---------------- the mission ----------------
void step() {
  bool power = rd.busV > BUS_ON_V;
  if (!power && state != ESTOP && state != IDLE) {
    driveStop();
    motorsEnable(false);
    bedRelax();                         // the servos have lost power too; never jump back to an old angle
    Serial.println("E-STOP: motor power is off");
    lights(Light::Off, Light::On);
    go(ESTOP);
    return;
  }
  bool button = buttonPressed();

  switch (state) {
    case IDLE:
      lights(Light::Slow, power ? Light::Off : Light::On);
      if (relaxWhenFlat && !bedMoving()) {
        bedRelax();
        relaxWhenFlat = false;
      }
      if (button) startMission(0);
      break;

    case ESTOP:
      if (heldFor(power, 1.0f)) {
        Serial.println("motor power is back - press the start button or type run");
        go(IDLE);
      }
      break;

    case LOAD:
      lights(Light::On, Light::Off);
      motorsEnable(false);
      if (heldFor(rd.cellsOk && rd.kg > cfg.load_kg, cfg.hold_s)) {
        Serial.printf("package on: %.2f kg\n", rd.kg);
        beep(200);
        toDropoff = true;
        motorsEnable(true);
        markArmed = true;
        go(FOLLOW);
      }
      break;

    case FOLLOW:
      if (follow()) {
        dockStart = rd.odom.distance;
        go(DOCK);
      }
      break;

    case DOCK:
      follow();
      if (state == DOCK && rd.odom.distance - dockStart >= cfg.stop_mm / 1000) {
        driveStop();
        motorsEnable(false);
        if (toDropoff) {
          go(CHECK);
        } else if (deliveriesLeft && --deliveriesLeft == 0) {
          Serial.println("all deliveries done");
          go(IDLE);
        } else {
          go(LOAD);
        }
      }
      break;

    case CHECK:
      if (millis() - since < 500) break;  // let the robot settle
      if (cfg.tof_check != 0 && !(rd.sideMm >= cfg.tof_min_mm && rd.sideMm <= cfg.tof_max_mm)) {
        alarm("nothing beside the robot's right side - not at the conveyor?", CHECK);
        break;
      }
      bedMove(cfg.tip_deg);
      lights(Light::Fast, Light::Off);
      go(TIP);
      break;

    case TIP:
      if (!bedMoving()) go(HOLD);
      break;

    case HOLD:
      if (millis() - since >= cfg.tip_hold_s * 1000) {
        bedMove(0);
        go(LOWER);
      }
      break;

    case LOWER:                         // weigh only with the bed flat: tipped, the cells feel less weight
      if (bedMoving()) break;
      if (servoFront.on()) {
        bedRelax();                     // resting on its posts now
        since = millis();
      } else if (millis() - since > 1000) {
        if (rd.cellsOk && rd.kg > cfg.load_kg / 2) {
          alarm("the package is still on the bed - press the button to tip again", CHECK);
          break;
        }
        Serial.println("package delivered");
        if (!tare()) Serial.println("!! could not zero the scale");
        toDropoff = false;
        motorsEnable(true);
        markArmed = true;
        lights(Light::On, Light::Off);
        go(FOLLOW);
      }
      break;

    case ALARM:
      lights(Light::Off, Light::On);
      if (millis() % 2000 < 30) beep(150);
      if (button) {
        Serial.println("carrying on");
        if (resumeTo == FOLLOW || resumeTo == DOCK) motorsEnable(true);
        go(resumeTo);
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nCargo robot (ESP32) - type help");
  settingsLoad();
  robotBegin();
  delay(500);                           // let the HX711s settle
  if (!tare()) Serial.println("!! HX711 boards not answering - the scale is not zeroed");
  else Serial.println("scale zeroed (the bed must be empty at power-on)");
  robotUpdate();
  Serial.printf("motor power %s (%.1f V)\n", rd.busV > BUS_ON_V ? "ON" : "OFF", rd.busV);
  go(IDLE);
}

void loop() {
  robotUpdate();
  consolePoll();
  step();
}
