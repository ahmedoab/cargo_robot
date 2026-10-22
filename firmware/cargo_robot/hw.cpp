#include "hw.h"

#include <VL53L0X.h>
#include <Wire.h>

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

// ---------------- HX711Array ----------------
static portMUX_TYPE hxMux = portMUX_INITIALIZER_UNLOCKED;

void HX711Array::begin() {
  pinMode(PIN_HX_SCK, OUTPUT);
  digitalWrite(PIN_HX_SCK, LOW);                  // SCK low = the chips are powered up
  for (int pin : PIN_HX_DT) pinMode(pin, INPUT_PULLUP);
}

int HX711Array::notReady() {
  int bits = 0;
  for (int i = 0; i < 4; i++)
    if (digitalRead(PIN_HX_DT[i])) bits |= 1 << i;   // DOUT high = no new reading yet
  return bits;
}

bool HX711Array::read(long out[4], uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (notReady()) {
    if (millis() - t0 > timeoutMs) return false;
    delay(1);
  }
  uint32_t v[4] = {0, 0, 0, 0};
  portENTER_CRITICAL(&hxMux);                     // keep SCK high short: > 60 us high = the HX711 powers down
  for (int bit = 0; bit < 24; bit++) {
    digitalWrite(PIN_HX_SCK, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_HX_SCK, LOW);
    for (int i = 0; i < 4; i++) v[i] = (v[i] << 1) | digitalRead(PIN_HX_DT[i]);
    delayMicroseconds(1);
  }
  digitalWrite(PIN_HX_SCK, HIGH);                 // 25th pulse: next reading = channel A, gain 128
  delayMicroseconds(1);
  digitalWrite(PIN_HX_SCK, LOW);
  portEXIT_CRITICAL(&hxMux);
  for (int i = 0; i < 4; i++) out[i] = (v[i] & 0x800000) ? (long)v[i] - 0x1000000L : (long)v[i];
  return true;
}

// ---------------- Expander (MCP23017, registers with IOCON.BANK = 0) ----------------
namespace {
constexpr uint8_t IODIRA = 0x00, IODIRB = 0x01, GPPUA = 0x0C, GPPUB = 0x0D, GPIOA = 0x12, GPIOB = 0x13,
                  OLATB = 0x15;
constexpr uint8_t PORTB_INPUTS = 0xF0;           // GPB0-3 outputs, GPB4-7 inputs
}  // namespace

bool Expander::write(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MCP_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

int Expander::read(uint8_t reg) {
  Wire.beginTransmission(MCP_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint8_t)MCP_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

bool Expander::begin() {
  olatB_ = 0;                                     // motors disabled, buzzer and LEDs off
  write(OLATB, olatB_);
  write(IODIRA, 0xFF);                            // port A: 8 line-sensor inputs (the board drives them)
  write(GPPUA, 0x00);
  write(IODIRB, PORTB_INPUTS);
  write(GPPUB, PORTB_INPUTS);                     // pull-ups on the button and the spare inputs
  ok = read(IODIRB) == PORTB_INPUTS;
  return ok;
}

uint8_t Expander::lineBits() {
  int v = read(GPIOA);
  return v < 0 ? 0 : (uint8_t)v;
}

bool Expander::buttonPressed() {
  int v = read(GPIOB);
  return v >= 0 && !(v & (1 << (XP_BUTTON - 8)));  // pressed = pulled to GND
}

void Expander::out(int pin, bool on) {
  uint8_t mask = 1 << (pin - 8);
  uint8_t next = on ? (olatB_ | mask) : (olatB_ & ~mask);
  if (next != olatB_ && write(OLATB, next)) olatB_ = next;
}

// ---------------- sonar ----------------
namespace sonar {
namespace {
volatile uint32_t riseUs[3];
volatile uint32_t widthUs[3];
volatile bool fresh[3];
float lastCm[3] = {SONAR_MAX_CM, SONAR_MAX_CM, SONAR_MAX_CM};
uint32_t lastPing = 0;

void IRAM_ATTR onEcho(void *arg) {
  int i = (int)(intptr_t)arg;
  uint32_t now = micros();
  if (digitalRead(PIN_ECHO[i])) {
    riseUs[i] = now;
  } else {
    widthUs[i] = now - riseUs[i];
    fresh[i] = true;
  }
}
}  // namespace

void begin() {
  pinMode(PIN_SONAR_TRIG, OUTPUT);
  digitalWrite(PIN_SONAR_TRIG, LOW);
  for (int i = 0; i < 3; i++) {
    pinMode(PIN_ECHO[i], INPUT);
    attachInterruptArg(PIN_ECHO[i], onEcho, (void *)(intptr_t)i, CHANGE);
  }
}

void update() {
  if (millis() - lastPing < SONAR_PERIOD_MS) return;
  lastPing = millis();
  for (int i = 0; i < 3; i++) {                   // results of the previous ping
    lastCm[i] = fresh[i] ? min(widthUs[i] / 58.0f, SONAR_MAX_CM) : SONAR_MAX_CM;
    fresh[i] = false;
  }
  digitalWrite(PIN_SONAR_TRIG, HIGH);             // all 3 sensors ping at once
  delayMicroseconds(10);
  digitalWrite(PIN_SONAR_TRIG, LOW);
}

float cm(int i) { return lastCm[i]; }
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

// ---------------- side distance sensor (VL53L0X) ----------------
static VL53L0X side;
static bool sideOk = false;
static int sideLast = -1;

bool sideBegin() {
  side.setBus(&Wire);
  side.setTimeout(50);
  sideOk = side.init();
  if (sideOk) side.startContinuous(50);
  return sideOk;
}

int sideMm() {
  if (!sideOk) return -1;
  if (side.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) {    // a new reading is waiting
    uint16_t mm = side.readRangeContinuousMillimeters();
    sideLast = (side.timeoutOccurred() || mm >= 8000) ? -1 : mm;
  }
  return sideLast;
}

// ---------------- motor bus voltage ----------------
float busVolts() { return analogReadMilliVolts(PIN_BUS_SENSE) / 1000.0f * BUS_DIVIDER; }
