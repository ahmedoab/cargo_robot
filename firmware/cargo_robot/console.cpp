#include "console.h"

#include "config.h"
#include "robot.h"
#include "settings.h"

namespace {

String lineBuf;

// ---------------- helpers for the tests ----------------
bool keyPressed() {
  if (!Serial.available()) return false;
  while (Serial.available()) Serial.read();
  return true;
}

void waitEnter(const char *prompt) {
  Serial.print(prompt);
  Serial.println("  (press Enter)");
  while (!Serial.available()) robotUpdate();
  while (Serial.available()) {
    Serial.read();
    delay(2);
  }
}

// Repeat print() 5 times a second until a key is pressed.
template <typename F>
void monitor(F print, int periodMs = 200) {
  Serial.println("(press Enter to stop)");
  uint32_t last = 0;
  while (!keyPressed()) {
    robotUpdate();
    if (millis() - last >= (uint32_t)periodMs) {
      last = millis();
      print();
    }
  }
}

void runFor(float seconds) {
  uint32_t t0 = millis();
  while (millis() - t0 < seconds * 1000) robotUpdate();
}

const char *SIDES[2] = {"left", "right"};
Motor *motors[2] = {&motorL, &motorR};
Encoder *encs[2] = {&encL, &encR};

void powerCheck() {
  robotUpdate();
  Serial.printf("motor power: %s (%.1f V)\n", rd.busV > BUS_ON_V ? "ON" : "OFF - E-stop pressed or battery out",
                rd.busV);
}

// ---------------- motor and encoder tests (wheels in the air!) ----------------
void testEncoders() {
  Serial.println("Turn each wheel FORWARD by hand: its count must go UP.");
  monitor([] {
    Serial.printf("left %8ld (%+6.2f turns)    right %8ld (%+6.2f turns)\n", encL.count(),
                  encL.count() / cfg.ticks_per_rev, encR.count(), encR.count() / cfg.ticks_per_rev);
  });
}

void testMotors(float duty) {
  powerCheck();
  manualMotors = true;
  motorsEnable(true);
  for (int s = 0; s < 2; s++) {
    for (int dir : {1, -1}) {
      Serial.printf("\n[%s wheel %s]", SIDES[s], dir > 0 ? "FORWARD" : "BACKWARD");
      waitEnter(" watch the wheel");
      long c0 = encs[s]->count();
      motors[s]->set(dir * duty);
      runFor(2.0f);
      motors[s]->set(0);
      runFor(0.5f);
      long delta = encs[s]->count() - c0;
      Serial.printf("  encoder change: %+ld counts (%+.2f wheel turns)\n", delta, delta / cfg.ticks_per_rev);
      if (delta == 0)
        Serial.println("  !! the encoder did not count: check its 3.3 V, GND, A/B wires and the 10k pull-ups");
      else if ((delta > 0) != (dir > 0))
        Serial.printf("  !! count has the wrong sign: type   set inv_%c_enc %d   then run motortest again\n",
                      SIDES[s][0], (s == 0 ? cfg.inv_l_enc : cfg.inv_r_enc) == 0 ? 1 : 0);
    }
  }
  motorsEnable(false);
  manualMotors = false;
  Serial.println("\nBy eye: on FORWARD, did each wheel turn the way that drives the robot FORWARD?");
  Serial.println("If not: set inv_l_motor 1 (or inv_r_motor 1), then run motortest again. Type save when done.");
}

void testMaxSpeed() {
  powerCheck();
  manualMotors = true;
  motorsEnable(true);
  float slowest = 1e9;
  for (int s = 0; s < 2; s++) {
    Serial.printf("\n[%s wheel 100 %%]", SIDES[s]);
    waitEnter("");
    motors[s]->set(1.0f);
    runFor(1.0f);                                 // let it spin up
    long c0 = encs[s]->count();
    runFor(2.0f);
    float w = (encs[s]->count() - c0) / cfg.ticks_per_rev * 2 * PI / 2.0f;
    motors[s]->set(0);
    runFor(0.5f);
    Serial.printf("  top speed %.1f rad/s = %.0f rpm\n", w, w * 30 / PI);
    slowest = min(slowest, fabsf(w));
  }
  motorsEnable(false);
  manualMotors = false;
  if (slowest > 0.1f) Serial.printf("\nType:   set kf %.3f   then   save\n", 1.0f / slowest);
}

void testMinDuty() {
  powerCheck();
  manualMotors = true;
  motorsEnable(true);
  float found = 0;
  for (int s = 0; s < 2; s++) {
    Serial.printf("\n[%s wheel slow ramp]", SIDES[s]);
    waitEnter("");
    float duty = 0;
    long c0 = encs[s]->count();
    while (duty < 0.5f && labs(encs[s]->count() - c0) < 20) {
      duty += 0.01f;
      motors[s]->set(duty);
      runFor(0.3f);
    }
    motors[s]->set(0);
    Serial.printf("  started moving at %.2f\n", duty);
    found = max(found, duty);
  }
  motorsEnable(false);
  manualMotors = false;
  Serial.printf("\nType:   set min_duty %.2f   then   save\n", 0.9f * found);
}

void testOneMotor(char side, float duty, float seconds) {
  powerCheck();
  manualMotors = true;
  motorsEnable(true);
  Motor *m = side == 'r' ? &motorR : &motorL;
  m->set(duty);
  uint32_t t0 = millis();
  while (millis() - t0 < seconds * 1000 && !keyPressed()) robotUpdate();
  m->set(0);
  motorsEnable(false);
  manualMotors = false;
}

void testDrive(float v, float w, float seconds) {
  powerCheck();
  Odometry start = rd.odom;
  motorsEnable(true);
  driveTarget(v, w);
  uint32_t t0 = millis(), last = 0;
  Serial.println("(press Enter to stop early)");
  while (millis() - t0 < seconds * 1000 && !keyPressed()) {
    robotUpdate();
    if (millis() - last >= 250) {
      last = millis();
      Serial.printf("wheels L %+5.2f R %+5.2f rad/s\n", rd.wheelL, rd.wheelR);
    }
  }
  driveTarget(0, 0);
  runFor(1.5f);                                   // ramp down
  motorsEnable(false);
  Serial.printf("odometry: %.3f m forward, turned %.1f deg\n", rd.odom.distance - start.distance,
                (rd.odom.heading - start.heading) * 180 / PI);
}

// ---------------- load cells ----------------
const char *CORNERS[4] = {"front-left", "front-right", "rear-left", "rear-right"};

void testCells() {
  if (!tare()) {
    Serial.println("!! HX711 boards not answering: check their VCC, GND, SCK and DT wires");
    return;
  }
  Serial.println("zeroed - corners in kg, then the total (uses scale and sign_* from the settings)");
  monitor([] {
    for (int i = 0; i < 4; i++) Serial.printf("%s %7.3f   ", CORNERS[i], rd.cornerKg[i]);
    Serial.printf("| total %7.3f kg\n", rd.kg);
  });
}

bool average(float out[4]) {                      // 1.5 s of readings, per cell
  long raw[4];
  float sum[4] = {0, 0, 0, 0};
  int n = 0;
  for (int k = 0; k < 15; k++) {
    if (!cells.read(raw)) continue;
    for (int i = 0; i < 4; i++) sum[i] += raw[i];
    n++;
  }
  for (int i = 0; i < 4; i++) out[i] = sum[i] / max(n, 1);
  if (n < 8) Serial.println("!! HX711 boards not answering: check their VCC, GND, SCK and DT wires");
  return n >= 8;
}

void calibrate(float kg) {
  float zero[4], loaded[4];
  waitEnter("1) EMPTY the cargo bed");
  if (!average(zero)) return;
  Serial.printf("2) put the %.2f kg weight in the MIDDLE of the bed", kg);
  waitEnter("");
  if (!average(loaded)) return;
  float delta[4], total = 0;
  for (int i = 0; i < 4; i++) {
    delta[i] = loaded[i] - zero[i];
    total += delta[i];
  }
  float majority = total >= 0 ? 1 : -1, scale = 0;
  float *signs[4] = {&cfg.sign_fl, &cfg.sign_fr, &cfg.sign_rl, &cfg.sign_rr};
  for (int i = 0; i < 4; i++) {
    bool same = delta[i] * majority >= 0 || fabsf(delta[i]) < 0.05f * fabsf(total) / 4;
    *signs[i] = same ? 1 : -1;
    scale += *signs[i] * delta[i];
    Serial.printf("   %-12s change %+10.0f%s\n", CORNERS[i], delta[i],
                  same ? "" : "   <-- opposite sign: swap this cell's A+/A- wires, or keep sign -1");
  }
  cfg.scale = scale / kg;
  Serial.printf("\nscale %.1f, signs %g %g %g %g - now in use. Type save to keep them.\n", cfg.scale, cfg.sign_fl,
                cfg.sign_fr, cfg.sign_rl, cfg.sign_rr);
  if (fabsf(cfg.scale) < 1000) Serial.println("!! very small change - check the load-cell wiring (E+/E-/A+/A-)");
  tare();
}

// ---------------- sensors ----------------
void testLine() {
  Serial.println("sensor 1 (left) .. 8 (right): # = black");
  monitor([] {
    char s[9];
    for (int i = 0; i < 8; i++) s[i] = rd.line.black & (1 << i) ? '#' : '.';
    s[8] = 0;
    Serial.printf("%s   on black %d   position %+.2f%s\n", s, rd.line.count, rd.line.pos,
                  rd.line.count >= cfg.mark_min ? "   STATION MARK" : "");
  }, 100);
}

void testSonar() {
  monitor([] {
    Serial.printf("left %5.0f cm   centre %5.0f cm   right %5.0f cm   (%.0f = nothing)\n", rd.sonarCm[0],
                  rd.sonarCm[1], rd.sonarCm[2], SONAR_MAX_CM);
  });
}

void testSide() {
  monitor([] {
    if (rd.sideMm < 0) Serial.println("side: nothing in range (or no sensor)");
    else Serial.printf("side: %d mm%s\n", rd.sideMm,
                       rd.sideMm >= cfg.tof_min_mm && rd.sideMm <= cfg.tof_max_mm ? "   OK to tip" : "");
  });
}

// ---------------- bed ----------------
void testServo(char which, const String &arg) {
  BedServo *s = which == 'r' ? &servoRear : &servoFront;
  if (arg == "off") {
    s->off();
    return;
  }
  powerCheck();
  s->pulse(arg.toFloat());
  Serial.printf("%s servo at %.0f us. Change it in small steps (10-20 us); \"servo %c off\" to release.\n",
                which == 'r' ? "rear" : "front", arg.toFloat(), which);
}

void testTip() {
  powerCheck();
  Serial.printf("on the bed: %.2f kg - tipping to %.0f deg ...\n", rd.kg, cfg.tip_deg);
  uint32_t t0 = millis();
  bedMove(cfg.tip_deg);
  while (bedMoving()) robotUpdate();
  Serial.printf("tipped after %.1f s, holding %.1f s\n", (millis() - t0) / 1000.0f, cfg.tip_hold_s);
  runFor(cfg.tip_hold_s);
  bedMove(0);
  while (bedMoving()) robotUpdate();
  runFor(0.5f);
  bedRelax();
  runFor(1.0f);                                   // weigh with the bed flat: tipped, the cells feel less weight
  Serial.printf("bed flat again: %.2f kg on it - %s\n", rd.kg,
                rd.kg < cfg.load_kg / 2 ? "delivered" : "the package did NOT slide off");
}

// ---------------- status ----------------
void status() {
  char bits[9];
  for (int i = 0; i < 8; i++) bits[i] = rd.line.black & (1 << i) ? '#' : '.';
  bits[8] = 0;
  Serial.printf("%-7s | %4.1f V | %6.3f kg%s | line %s | sonar %3.0f %3.0f %3.0f cm | side %4d mm | bed %4.1f deg\n",
                stateName(), rd.busV, rd.kg, rd.cellsOk ? "" : " (no HX711)", bits, rd.sonarCm[0], rd.sonarCm[1],
                rd.sonarCm[2], rd.sideMm, rd.bedDeg);
}

void help() {
  Serial.println(R"(
MISSION    run [n]          start (n deliveries, 0 = until stopped); the start button does the same
           stop             stop everything
           status | watch   one line of readings | the same, twice a second
DRIVE      enc              encoder counts (turn the wheels by hand)          WHEELS IN THE AIR for:
           motortest        each wheel forward/back 30 %, checks directions
           mintest | maxtest   measure min_duty | kf
           motor l|r <duty> [s]   one motor open loop, duty -1..1
           drive <v> <w> [s]      closed loop: v m/s, w rad/s, default 2 s; prints odometry
LOAD CELLS hx               live corners + total kg (zeroes first: bed empty)
           tare             zero the scale (bed empty)
           cal <kg>         calibrate with a known weight
SENSORS    line | sonar | tof   live line sensors | ultrasonic | side distance
BED        servo f|r <us>   one servo to a raw pulse (calibration); servo f|r off
           bed <deg>        both servos smoothly to deg (0 = flat); bed off = release
           tip              tip, wait until empty, lower
SETTINGS   list | get <name> | set <name> <value> | save | defaults)");
}

void settingsCommand(const String &cmd, const String &a1, const String &a2) {
  if (cmd == "list") {
    settingsList(Serial);
  } else if (cmd == "get") {
    float v;
    if (settingsGet(a1.c_str(), v)) Serial.printf("%s = %g\n", a1.c_str(), v);
    else Serial.println("unknown setting (type list)");
  } else if (cmd == "set") {
    if (!settingsSet(a1.c_str(), a2.toFloat())) {
      Serial.println("unknown setting (type list)");
      return;
    }
    applyInversions();
    Serial.printf("%s = %g (in use now; type save to keep it after power-off)\n", a1.c_str(), a2.toFloat());
  } else if (cmd == "save") {
    Serial.println(settingsSave() ? "saved" : "!! save failed");
  } else if (cmd == "defaults") {
    settingsDefaults();
    applyInversions();
    Serial.println("back to the defaults (also in flash)");
  }
}

void runCommand(String line) {
  line.trim();
  line.toLowerCase();
  if (!line.length()) return;
  String w[4];
  int n = 0;
  for (int start = 0; n < 4 && start < (int)line.length();) {
    int sp = line.indexOf(' ', start);
    if (sp < 0) sp = line.length();
    if (sp > start) w[n++] = line.substring(start, sp);
    start = sp + 1;
  }
  const String &c = w[0];
  if (c == "stop") return stopMission();
  if (c == "status") return status();
  if (c == "help") return help();
  if (strcmp(stateName(), "IDLE") != 0) {
    Serial.println("busy: type stop first");
    return;
  }
  if (c == "run") return startMission(w[1].toInt());
  if (c == "watch") return monitor(status, 500);
  if (c == "enc") return testEncoders();
  if (c == "motortest") return testMotors(w[1].length() ? w[1].toFloat() : 0.3f);
  if (c == "mintest") return testMinDuty();
  if (c == "maxtest") return testMaxSpeed();
  if (c == "motor") return testOneMotor(w[1][0], w[2].toFloat(), w[3].length() ? w[3].toFloat() : 2.0f);
  if (c == "drive") return testDrive(w[1].toFloat(), w[2].toFloat(), w[3].length() ? w[3].toFloat() : 2.0f);
  if (c == "hx") return testCells();
  if (c == "tare") return (void)Serial.println(tare() ? "zeroed" : "!! HX711 boards not answering");
  if (c == "cal") return calibrate(w[1].toFloat() > 0 ? w[1].toFloat() : 2.0f);
  if (c == "line") return testLine();
  if (c == "sonar") return testSonar();
  if (c == "tof") return testSide();
  if (c == "servo") return testServo(w[1][0], w[2]);
  if (c == "bed") {
    if (w[1] == "off") return bedRelax();
    bedMove(w[1].toFloat());
    while (bedMoving()) robotUpdate();
    return;
  }
  if (c == "tip") return testTip();
  if (c == "list" || c == "get" || c == "set" || c == "save" || c == "defaults") return settingsCommand(c, w[1], w[2]);
  Serial.println("unknown command - type help");
}

}  // namespace

void consolePoll() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (lineBuf.length()) {
        Serial.printf("> %s\n", lineBuf.c_str());
        runCommand(lineBuf);
      }
      lineBuf = "";
    } else if (lineBuf.length() < 80) {
      lineBuf += ch;
    }
  }
}
