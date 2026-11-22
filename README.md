# JarSense---Smart-Mixer-Grinder
This respository is about the JarSense, an Adaptive Resonance-Control Smart Mixer Grinder which can be used for Noise reduction
 * Smart Mixer Grinder - ESP32 firmware (Rev A prototype)
 *
 * Driver: BTS7960 / IBT-2 module (one direction only)
 *   GPIO25 -> RPWM            (speed PWM)
 *   GPIO26 -> R_EN AND L_EN   (both joined together; active HIGH)
 *   LPWM   -> GND             (tie directly to GND; reverse is never used)
 *   IBT-2 VCC -> +5V (buck)   IBT-2 GND -> logic 0V (star point)
 *   B+ / B-   -> 24 V after F1 + E-stop K1 / PSU-      M+ / M- -> motor
 *   R_IS / L_IS: unused (optional current sense, not read by this firmware)
 *
 * Other pins (unchanged from schematic):
 *   GPIO21 -> SDA (MPU6050)      GPIO22 -> SCL (MPU6050, addr 0x68)
 *   GPIO27 <- A3144 Hall output (open-collector, 10k pull-up to 3V3)
 *
 * Build: Arduino IDE (board "ESP32 Dev Module") or PlatformIO (rename to src/main.cpp).
 * No external libraries needed (MPU6050 is read directly over Wire).
 *
 * Serial monitor, 115200 baud, send a line ending with newline:
 *   d<0-100>   run open-loop at that duty %      e.g.  d30
 *   s<rpm>     run closed-loop at target RPM     e.g.  s3000
 *   x          stop (ramps down, then disables driver)
 *   c          clear a latched fault
 *   ?          print status
 *
 * SAFETY: the hardware E-stop (K1) is the real protection. Everything here is
 * a secondary software safeguard. First tests: no blade, guarded, no load.
 */

#include <Arduino.h>
#include <Wire.h>

// ---------------- Pins ----------------
static const int PIN_PWM   = 25;
static const int PIN_EN    = 26;
static const int PIN_SDA   = 21;
static const int PIN_SCL   = 22;
static const int PIN_HALL  = 27;

// ---------------- Configuration (tune these) ----------------
static const bool     EN_ACTIVE_HIGH   = true;    // set false if your driver enables on LOW
static const uint32_t PWM_FREQ_HZ      = 20000;   // above audible range
static const uint8_t  PWM_BITS         = 10;      // 0..1023
static const float    MAX_DUTY_PCT     = 60.0f;   // hard cap while prototyping; raise gradually
static const float    RAMP_PCT_PER_SEC = 25.0f;   // soft start / soft stop slope
static const float    MAGNETS_PER_REV  = 1.0f;    // per schematic
static const float    MAX_RPM          = 12000.0f;// overspeed fault; set from your mechanical rating

// PID (closed-loop RPM). Output is duty %. Start small and tune.
static const float KP = 0.010f;
static const float KI = 0.012f;
static const float KD = 0.000f;

// Stall / jam detection: driving hard but no Hall pulses
static const float    STALL_DUTY_PCT = 15.0f;
static const float    STALL_RPM      = 30.0f;
static const uint32_t STALL_MS       = 2000;

// Vibration / unbalance detection (MPU6050)
static const bool     USE_IMU        = true;
static const bool     REQUIRE_IMU    = true;      // refuse to run if IMU not found
static const float    VIB_LIMIT_G    = 0.80f;     // tune with your real jar/motor
static const uint32_t VIB_MS         = 1000;

// ---------------- State ----------------
enum RunState { ST_IDLE, ST_RUN, ST_STOPPING, ST_FAULT };
enum Mode     { MODE_OPEN, MODE_CLOSED };

static RunState state = ST_IDLE;
static Mode     mode  = MODE_OPEN;

static float targetDutyPct = 0.0f;   // open-loop request
static float targetRpm     = 0.0f;   // closed-loop request
static float dutyPct       = 0.0f;   // actual duty being output
static float rpmNow        = 0.0f;
static String faultReason  = "";

// PID
static float pidIntegral = 0.0f;
static float pidPrevErr  = 0.0f;

// Hall
static volatile uint32_t hallLastUs   = 0;
static volatile uint32_t hallPeriodUs = 0;
static portMUX_TYPE hallMux = portMUX_INITIALIZER_UNLOCKED;
static const uint32_t HALL_MIN_PERIOD_US = 2000;   // ignore glitches (>30000 RPM)
static const uint32_t HALL_TIMEOUT_US    = 500000; // no pulse for 0.5 s -> RPM = 0

// IMU
static bool   imuOk      = false;
static float  vibEma     = 0.0f;
static float  magAvg     = 1.0f;
static uint32_t vibStartMs = 0;
static bool   vibActive  = false;

// Timing
static uint32_t lastControlMs = 0;
static uint32_t lastImuMs     = 0;
static uint32_t lastPrintMs   = 0;
static uint32_t stallStartMs  = 0;
static bool     stallActive   = false;

// ---------------- PWM helpers (works on Arduino-ESP32 core 2.x and 3.x) ----------------
static void pwmBegin() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_PWM, PWM_FREQ_HZ, PWM_BITS);
#else
  ledcSetup(0, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(PIN_PWM, 0);
#endif
}

static void pwmWriteDuty(float pct) {
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  uint32_t maxVal = (1u << PWM_BITS) - 1;
  uint32_t duty = (uint32_t)(pct * maxVal / 100.0f + 0.5f);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_PWM, duty);
#else
  ledcWrite(0, duty);
#endif
}

static void driverEnable(bool on) {
  digitalWrite(PIN_EN, (on == EN_ACTIVE_HIGH) ? HIGH : LOW);
}

// Immediate, unconditional shutdown
static void hardStop() {
  pwmWriteDuty(0);
  driverEnable(false);
  dutyPct = 0;
  pidIntegral = 0;
  pidPrevErr = 0;
}

static void latchFault(const String &why) {
  hardStop();
  state = ST_FAULT;
  faultReason = why;
  Serial.print("FAULT: ");
  Serial.println(why);
}

// ---------------- Hall sensor ----------------
void IRAM_ATTR hallISR() {
  uint32_t now = micros();
  uint32_t dt = now - hallLastUs;
  if (dt > HALL_MIN_PERIOD_US) {
    hallPeriodUs = dt;
    hallLastUs = now;
  }
}

static float readRpm() {
  uint32_t period, last;
  portENTER_CRITICAL(&hallMux);
  period = hallPeriodUs;
  last = hallLastUs;
  portEXIT_CRITICAL(&hallMux);

  if (period == 0 || (micros() - last) > HALL_TIMEOUT_US) return 0.0f;
  return (60.0e6f / (float)period) / MAGNETS_PER_REV;
}

// ---------------- MPU6050 (raw I2C) ----------------
static const uint8_t MPU_ADDR = 0x68;

static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;
  if (!mpuWrite(0x6B, 0x00)) return false;  // wake up
  mpuWrite(0x1C, 0x08);                      // accel range +/-4 g (8192 LSB/g)
  mpuWrite(0x1A, 0x03);                      // digital low-pass ~44 Hz
  return true;
}

static bool mpuReadAccelG(float &mag) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 6) != 6) return false;
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  const float s = 1.0f / 8192.0f;
  float x = ax * s, y = ay * s, z = az * s;
  mag = sqrtf(x * x + y * y + z * z);
  return true;
}

static void imuUpdate() {
  if (!imuOk) return;
  float mag;
  if (!mpuReadAccelG(mag)) {
    if (state == ST_RUN) latchFault("IMU read failed");
    return;
  }
  // slow average tracks gravity/orientation; deviation from it = vibration
  magAvg += 0.02f * (mag - magAvg);
  float dev = fabsf(mag - magAvg);
  vibEma += 0.1f * (dev - vibEma);

  if (state == ST_RUN && vibEma > VIB_LIMIT_G) {
    if (!vibActive) { vibActive = true; vibStartMs = millis(); }
    else if (millis() - vibStartMs > VIB_MS) latchFault("Excess vibration");
  } else {
    vibActive = false;
  }
}

// ---------------- Control ----------------
static void startRun() {
  if (state == ST_FAULT) { Serial.println("Fault latched. Send 'c' to clear."); return; }
  if (USE_IMU && REQUIRE_IMU && !imuOk) { Serial.println("IMU not found; refusing to run."); return; }
  if (state == ST_IDLE) {
    pwmWriteDuty(0);
    dutyPct = 0;
    pidIntegral = 0;
    pidPrevErr = 0;
    driverEnable(true);
  }
  stallActive = false;
  vibActive = false;
  state = ST_RUN;
}

static void controlStep(float dt) {
  rpmNow = readRpm();

  // overspeed
  if (rpmNow > MAX_RPM) { latchFault("Overspeed"); return; }

  float desiredDuty = 0.0f;

  if (state == ST_RUN) {
    if (mode == MODE_OPEN) {
      desiredDuty = targetDutyPct;
    } else {
      float err = targetRpm - rpmNow;
      pidIntegral += err * dt;
      // anti-windup: limit integral contribution to the duty range
      float iMax = MAX_DUTY_PCT / (KI > 0 ? KI : 1.0f);
      if (pidIntegral > iMax) pidIntegral = iMax;
      if (pidIntegral < 0) pidIntegral = 0;
      float deriv = (err - pidPrevErr) / dt;
      pidPrevErr = err;
      desiredDuty = KP * err + KI * pidIntegral + KD * deriv;
    }
  } else if (state == ST_STOPPING) {
    desiredDuty = 0.0f;
  }

  if (desiredDuty > MAX_DUTY_PCT) desiredDuty = MAX_DUTY_PCT;
  if (desiredDuty < 0) desiredDuty = 0;

  // slew-rate limit (soft start / stop)
  float step = RAMP_PCT_PER_SEC * dt;
  if (desiredDuty > dutyPct + step) dutyPct += step;
  else if (desiredDuty < dutyPct - step) dutyPct -= step;
  else dutyPct = desiredDuty;

  pwmWriteDuty(dutyPct);

  if (state == ST_STOPPING && dutyPct <= 0.5f) {
    hardStop();
    state = ST_IDLE;
    Serial.println("Stopped.");
    return;
  }

  // stall / jam detection
  if (state == ST_RUN && dutyPct > STALL_DUTY_PCT && rpmNow < STALL_RPM) {
    if (!stallActive) { stallActive = true; stallStartMs = millis(); }
    else if (millis() - stallStartMs > STALL_MS) latchFault("Stall / no Hall pulses");
  } else {
    stallActive = false;
  }
}

// ---------------- Serial commands ----------------
static void printStatus() {
  const char *names[] = {"IDLE", "RUN", "STOPPING", "FAULT"};
  Serial.printf("state=%s mode=%s duty=%.1f%% rpm=%.0f target=%.0f vib=%.2fg%s%s\n",
                names[state], mode == MODE_OPEN ? "open" : "closed",
                dutyPct, rpmNow,
                mode == MODE_OPEN ? targetDutyPct : targetRpm,
                vibEma,
                state == ST_FAULT ? " reason=" : "",
                state == ST_FAULT ? faultReason.c_str() : "");
}

static void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  char c = line.charAt(0);
  float v = line.substring(1).toFloat();

  switch (c) {
    case 'd':
      if (v < 0 || v > 100) { Serial.println("Duty must be 0-100"); break; }
      mode = MODE_OPEN;
      targetDutyPct = min(v, MAX_DUTY_PCT);
      if (v > MAX_DUTY_PCT) Serial.printf("Capped to MAX_DUTY_PCT=%.0f\n", MAX_DUTY_PCT);
      startRun();
      break;
    case 's':
      if (v <= 0 || v > MAX_RPM) { Serial.println("RPM out of range"); break; }
      mode = MODE_CLOSED;
      targetRpm = v;
      pidIntegral = 0;
      startRun();
      break;
    case 'x':
      if (state == ST_RUN) state = ST_STOPPING;
      break;
    case 'c':
      if (state == ST_FAULT) { state = ST_IDLE; faultReason = ""; Serial.println("Fault cleared."); }
      break;
    case '?':
      printStatus();
      break;
    default:
      Serial.println("Commands: d<0-100>  s<rpm>  x  c  ?");
  }
}

static String rxLine;
static void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') { handleCommand(rxLine); rxLine = ""; }
    else if (rxLine.length() < 32) rxLine += ch;
  }
}

// ---------------- Arduino entry points ----------------
void setup() {
  // Outputs safe first
  pinMode(PIN_EN, OUTPUT);
  driverEnable(false);
  pwmBegin();
  pwmWriteDuty(0);

  Serial.begin(115200);
  delay(200);
  Serial.println("\nSmart Mixer Grinder firmware - Rev A");

  pinMode(PIN_HALL, INPUT);  // external 10k pull-up to 3V3 per schematic
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallISR, FALLING);  // A3144 pulls LOW at magnet

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  if (USE_IMU) {
    imuOk = mpuInit();
    Serial.println(imuOk ? "MPU6050 OK" : "MPU6050 NOT FOUND (check wiring, addr 0x68)");
  }

  Serial.println("Commands: d<0-100>  s<rpm>  x  c  ?");
  lastControlMs = lastImuMs = lastPrintMs = millis();
}

void loop() {
  uint32_t now = millis();

  pollSerial();

  if (now - lastImuMs >= 10) {          // 100 Hz
    lastImuMs = now;
    imuUpdate();
  }

  if (now - lastControlMs >= 20) {      // 50 Hz
    float dt = (now - lastControlMs) / 1000.0f;
    lastControlMs = now;
    if (state == ST_RUN || state == ST_STOPPING) controlStep(dt);
    else rpmNow = readRpm();
  }

  if (state != ST_IDLE && now - lastPrintMs >= 500) {
    lastPrintMs = now;
    printStatus();
  }
}
