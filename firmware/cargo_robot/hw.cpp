#include "hw.h"

#include "config.h"

// ---------------- Motor ----------------
void Motor::begin(int pinFwd, int pinRev) {
  fwd_ = pinFwd;
  rev_ = pinRev;
  ledcAttach(fwd_, DRIVE_PWM_HZ, 10);
  ledcAttach(rev_, DRIVE_PWM_HZ, 10);
  set(0);
}

void Motor::set(float duty) {
  duty = constrain(duty, -1.0f, 1.0f);
  if (invert) duty = -duty;
  uint32_t d = (uint32_t)(fabsf(duty) * 1023.0f + 0.5f);
  ledcWrite(duty >= 0 ? rev_ : fwd_, 0);          // both inputs low = both low-side switches on = brake
  ledcWrite(duty >= 0 ? fwd_ : rev_, d);
}

// ---------------- Encoder ----------------
bool Encoder::begin(int pinA, int pinB) {
  pcnt_unit_config_t unit = {};
  unit.low_limit = -30000;
  unit.high_limit = 30000;
  unit.flags.accum_count = 1;                     // keep counting past the limits
  if (pcnt_new_unit(&unit, &unit_) != ESP_OK) return false;
  pcnt_glitch_filter_config_t filter = {};
  filter.max_glitch_ns = 1000;                    // ignore spikes shorter than 1 us
  pcnt_unit_set_glitch_filter(unit_, &filter);
  pcnt_chan_config_t a = {}, b = {};
  a.edge_gpio_num = pinA;
  a.level_gpio_num = pinB;
  b.edge_gpio_num = pinB;
  b.level_gpio_num = pinA;
  pcnt_channel_handle_t chA, chB;
  pcnt_new_channel(unit_, &a, &chA);
  pcnt_new_channel(unit_, &b, &chB);
  // x4 decoding: count every edge of A and of B, direction from the level of the other channel
  pcnt_channel_set_edge_action(chA, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(chA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_channel_set_edge_action(chB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(chB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_unit_add_watch_point(unit_, unit.high_limit);
  pcnt_unit_add_watch_point(unit_, unit.low_limit);
  pcnt_unit_enable(unit_);
  pcnt_unit_clear_count(unit_);
  pcnt_unit_start(unit_);
  return true;
}

long Encoder::count() {
  int c = 0;
  if (unit_) pcnt_unit_get_count(unit_, &c);
  return invert ? -c : c;
}

// ---------------- panel and line sensors (plain ESP32 pins) ----------------
void panelBegin() {
  for (int pin : {PIN_MOTOR_EN, PIN_BUZZER, PIN_LED_GREEN, PIN_LED_RED}) {
    digitalWrite(pin, LOW);                       // motors disabled, buzzer and LEDs off
    pinMode(pin, OUTPUT);
  }
  for (int pin : PIN_LINE) pinMode(pin, INPUT);   // the modules drive their outputs
  pinMode(PIN_BUTTON, INPUT_PULLUP);
}

uint8_t lineBits() {
  uint8_t bits = 0;
  for (int i = 0; i < LINE_N; i++)
    if (digitalRead(PIN_LINE[i])) bits |= 1 << i;
  return bits;
}

bool buttonDown() { return digitalRead(PIN_BUTTON) == LOW; }   // pressed = pulled to GND

void panelOut(int pin, bool on) { digitalWrite(pin, on ? HIGH : LOW); }

// ---------------- sonar ----------------
namespace sonar {
namespace {
volatile uint32_t riseUs = 0, widthUs = 0;
volatile bool fresh = false;
float lastCm = SONAR_MAX_CM;
uint32_t lastPing = 0;

void IRAM_ATTR onEcho() {
  uint32_t now = micros();
  if (digitalRead(PIN_SONAR_ECHO)) {
    riseUs = now;
  } else {
    widthUs = now - riseUs;
    fresh = true;
  }
}
}  // namespace

void begin() {
  pinMode(PIN_SONAR_TRIG, OUTPUT);
  digitalWrite(PIN_SONAR_TRIG, LOW);
  pinMode(PIN_SONAR_ECHO, INPUT);
  attachInterrupt(PIN_SONAR_ECHO, onEcho, CHANGE);
}

void update() {
  if (millis() - lastPing < SONAR_PERIOD_MS) return;
  lastPing = millis();
  lastCm = fresh ? min(widthUs / 58.0f, SONAR_MAX_CM) : SONAR_MAX_CM;   // result of the previous ping
  fresh = false;
  digitalWrite(PIN_SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SONAR_TRIG, LOW);
}

float cm() { return lastCm; }
}  // namespace sonar

// ---------------- BedServo ----------------
void BedServo::begin(int pin) {
  pin_ = pin;
  ledcAttach(pin_, 50, 16);
  off();
}

void BedServo::pulse(float us) {
  ledcWrite(pin_, (uint32_t)(constrain(us, 500.0f, 2500.0f) * 65536.0f / 20000.0f));
  on_ = true;
}

void BedServo::off() {
  ledcWrite(pin_, 0);
  on_ = false;
}

// ---------------- motor bus voltage ----------------
float busVolts() { return analogReadMilliVolts(PIN_BUS_SENSE) / 1000.0f * BUS_DIVIDER; }
