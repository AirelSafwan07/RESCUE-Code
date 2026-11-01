/**
 * RESCUE - 3rd Prototype: In-car child detection
 *
 * Follows the flow chart:
 *   1. Engine ON  (voltage detected)      -> stay in STANDBY, keep checking.
 *   2. Engine OFF (no voltage)            -> MONITORING: sound / motion / weight sensors active.
 *   3. 2 of 3 sensors activated           -> DETECTED: GSM sends SMS + buzzer sounds.
 *   4. Reading continues after 5 minutes  -> PANIC: multiple warning calls, buzzer keeps sounding.
 *      Reading stops before 5 minutes     -> back to MONITORING (buzzer off).
 *   5. Engine starts at any time          -> back to STANDBY (everything cancelled).
 *
 * Hardware:
 * - Arduino Uno
 * - SIM900A GSM module
 * - RCWL-0516 (microwave motion)
 * - HX711 + 50 kg load cell
 * - Sound sensor (W104 / KY-037 or similar)
 * - Active buzzer
 * - Voltage sensor (digital output)
 *
 * HARDWARE NOTES
 * - The voltage sensor must watch a line that is 0 V when the engine is OFF
 *   (ignition / ACC line, or a charging-voltage detector). The constant
 *   battery line is ~12 V even with the engine off, so it can't tell you
 *   whether the engine is running.
 * - The Arduino and SIM900A must be powered from a permanent (non-ignition)
 *   supply through a buck converter, or they turn off with the engine.
 * - SIM900A needs its own ~4 V supply (up to 2 A bursts), not the Arduino 5V pin.
 * - Check that the SIM900A RX pin tolerates the Uno's 5 V TX (else use a divider).
 * - Tare happens at power-up: the seat must be empty at that moment.
 */

#include "HX711.h"
#include <SoftwareSerial.h>

// --- PIN DEFINITIONS ---
#define VOLTAGE_PIN       2   // HIGH = engine/voltage present, LOW = engine off
#define MOTION_PIN        3   // RCWL-0516 output
#define SOUND_PIN         4   // Sound sensor digital output
#define BUZZER_PIN        5   // Active buzzer
#define LOADCELL_DOUT_PIN 6
#define LOADCELL_SCK_PIN  7
#define GSM_RX_PIN        9   // Connect to SIM900A TX
#define GSM_TX_PIN        10  // Connect to SIM900A RX

// Some sound modules output HIGH on sound, others LOW. Flip if needed.
#define SOUND_ACTIVE_LEVEL HIGH

// --- CONFIGURATION ---
const float WEIGHT_THRESHOLD = 5.0;               // kg (only meaningful after calibration)
float calibration_factor = 2280.0;                // MUST CALIBRATE YOUR SPECIFIC CELL

const uint8_t TRIGGERS_REQUIRED = 2;              // "2 sensors activated?"
const unsigned long TRIGGER_WINDOW = 5000UL;      // sensors firing within this window count together
const unsigned long ENGINE_DEBOUNCE = 2000UL;     // voltage state must be stable this long
const unsigned long ARM_DELAY = 30000UL;          // grace time after engine-off so the driver can leave (0 = off)
const unsigned long WEIGHT_INTERVAL = 300UL;      // how often to read the load cell

const unsigned long PANIC_DELAY = 300000UL;       // 5 minutes of continued detection -> panic mode
const unsigned long CLEAR_DELAY = 30000UL;        // reading must be gone this long to count as "stopped"
const unsigned long BUZZER_MAX_ON = 1800000UL;    // safety cut-off to protect the car battery (0 = never)

const unsigned long SMS_COOLDOWN = 120000UL;      // min gap between SMS rounds after a successful send
const unsigned long SMS_RETRY_INTERVAL = 30000UL; // retry gap after a failed send

const uint8_t CALL_ROUNDS = 3;                    // how many times to cycle through all contacts
const unsigned long RING_TIME = 20000UL;          // ring time per call
const unsigned long CALL_GAP = 5000UL;            // pause between calls

// Contacts for SMS and panic calls (max 8). Delete one entry if you only have one.
const char *const CONTACTS[] = { "+60XXXXXXXXX", "+60YYYYYYYYY" };
const uint8_t NUM_CONTACTS = sizeof(CONTACTS) / sizeof(CONTACTS[0]);

// --- OBJECTS ---
HX711 scale;
SoftwareSerial gsm(GSM_RX_PIN, GSM_TX_PIN);

// --- STATE ---
enum State { ENGINE_ON, MONITORING, DETECTED, PANIC };
State state = ENGINE_ON;

bool scaleOk = false;
bool powerLost = false;       // true = engine off
bool prevPowerLost = false;

bool buzzerOn = false;
bool panicCallsDone = false;
unsigned long monitoringStart = 0;
unsigned long detectedAt = 0;
unsigned long lastReadingAt = 0;
unsigned long lastWeightRead = 0;

uint8_t smsDoneMask = 0;
bool haveSmsAttempt = false;
unsigned long lastSmsAttemptAt = 0;
unsigned long smsGap = 0;

bool motionSeen = false, soundSeen = false, weightSeen = false;
unsigned long motionAt = 0, soundAt = 0, weightAt = 0;

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  gsm.begin(9600);

  pinMode(VOLTAGE_PIN, INPUT);   // use INPUT_PULLUP if your sensor output is open-collector
  pinMode(MOTION_PIN, INPUT);
  pinMode(SOUND_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println(F("Initializing scale (keep seat EMPTY)..."));
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  if (scale.wait_ready_timeout(2000)) {
    scale.set_scale(calibration_factor);
    scale.tare();
    scaleOk = true;
  } else {
    Serial.println(F("WARNING: HX711 not found - weight sensor disabled."));
  }

  Serial.println(F("Initializing GSM..."));
  if (gsmInit()) {
    Serial.println(F("GSM registered on network."));
  } else {
    Serial.println(F("WARNING: GSM not ready (will retry when sending)."));
  }

  Serial.println(F("System ready."));
}

// ---------------------------------------------------------------------------
// MAIN LOOP
// ---------------------------------------------------------------------------
void loop() {
  updatePowerState();

  // Engine state changed (this also catches changes seen during blocking calls)
  if (powerLost != prevPowerLost) {
    prevPowerLost = powerLost;
    enterState(powerLost ? MONITORING : ENGINE_ON);
  }

  switch (state) {
    case ENGINE_ON:   delay(100);       break;   // "Is voltage detected? YES" -> keep checking
    case MONITORING:  runMonitoring();  break;
    case DETECTED:    runDetected();    break;
    case PANIC:       runPanic();       break;
  }
}

// ---------------------------------------------------------------------------
// STATE MACHINE
// ---------------------------------------------------------------------------
void enterState(uint8_t s) {
  state = (State)s;
  switch (state) {
    case ENGINE_ON:
      Serial.println(F("State: ENGINE ON (standby)"));
      stopAlarm();
      haveSmsAttempt = false;
      break;

    case MONITORING:
      Serial.println(F("State: MONITORING"));
      stopAlarm();
      monitoringStart = millis();
      break;

    case DETECTED:
      Serial.println(F("State: DETECTED - SMS + buzzer"));
      detectedAt = millis();
      lastReadingAt = detectedAt;
      smsDoneMask = 0;
      buzzerOn = true;
      digitalWrite(BUZZER_PIN, HIGH);
      break;

    case PANIC:
      Serial.println(F("State: PANIC MODE"));
      panicCallsDone = false;
      break;
  }
}

void stopAlarm() {
  digitalWrite(BUZZER_PIN, LOW);
  buzzerOn = false;
  smsDoneMask = 0;
  panicCallsDone = false;
  clearSensorHistory();
}

// Engine off, waiting for 2 of 3 sensors
void runMonitoring() {
  unsigned long now = millis();

  // Let the driver get out of the car before arming
  if (ARM_DELAY > 0 && (now - monitoringStart) < ARM_DELAY) {
    clearSensorHistory();
    delay(50);
    return;
  }

  readSensors();
  if (countTriggers(millis()) >= TRIGGERS_REQUIRED) {
    enterState(DETECTED);
  }
}

// "Baby is detected": SMS + buzzer, then check if reading continues after 5 min
void runDetected() {
  readSensors();
  unsigned long now = millis();
  if (countTriggers(now) >= TRIGGERS_REQUIRED) lastReadingAt = now;

  trySendSms();                 // may block for a while
  now = millis();

  updateBuzzerTimeout(now);

  // Reading stopped -> back to monitoring
  if ((now - lastReadingAt) >= CLEAR_DELAY) {
    Serial.println(F("Reading stopped - back to monitoring."));
    enterState(MONITORING);
    monitoringStart = now - ARM_DELAY;   // already armed, no new grace period
    return;
  }

  // Reading continues after 5 minutes -> panic mode
  if ((now - detectedAt) >= PANIC_DELAY) {
    enterState(PANIC);
  }
}

// Panic: multiple warning calls, buzzer keeps sounding until the engine starts
void runPanic() {
  if (!panicCallsDone) {
    doPanicCalls();
    panicCallsDone = true;
    if (powerLost) Serial.println(F("Panic sequence complete. Buzzer on until engine starts."));
    return;
  }
  updateBuzzerTimeout(millis());
  delay(100);
}

void updateBuzzerTimeout(unsigned long now) {
  if (buzzerOn && BUZZER_MAX_ON > 0 && (now - detectedAt) >= BUZZER_MAX_ON) {
    buzzerOn = false;
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println(F("Buzzer safety timeout."));
  }
}

// ---------------------------------------------------------------------------
// ENGINE / VOLTAGE
// ---------------------------------------------------------------------------
// HIGH = voltage present (engine on). LOW = no voltage (engine off).
void updatePowerState() {
  static bool lastRaw = false;
  static unsigned long changeStart = 0;

  bool raw = (digitalRead(VOLTAGE_PIN) == LOW);
  if (raw != lastRaw) {
    lastRaw = raw;
    changeStart = millis();
  }
  if (raw != powerLost && (millis() - changeStart) >= ENGINE_DEBOUNCE) {
    powerLost = raw;
  }
}

// Wait while still watching the engine. Returns false if the engine started.
bool waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    updatePowerState();
    if (!powerLost) return false;
    delay(20);
  }
  return true;
}

// ---------------------------------------------------------------------------
// SENSORS
// ---------------------------------------------------------------------------
void readSensors() {
  unsigned long now = millis();

  if (digitalRead(MOTION_PIN) == HIGH) { motionSeen = true; motionAt = now; }
  if (digitalRead(SOUND_PIN) == SOUND_ACTIVE_LEVEL) { soundSeen = true; soundAt = now; }

  if (scaleOk && (now - lastWeightRead) >= WEIGHT_INTERVAL) {
    lastWeightRead = now;
    float w = scale.get_units(1);
    if (fabs(w) > WEIGHT_THRESHOLD) { weightSeen = true; weightAt = millis(); }
  }
}

bool isRecent(bool seen, unsigned long at, unsigned long now) {
  return seen && (now - at) <= TRIGGER_WINDOW;
}

uint8_t countTriggers(unsigned long now) {
  uint8_t t = 0;
  if (isRecent(motionSeen, motionAt, now)) t++;
  if (isRecent(soundSeen,  soundAt,  now)) t++;
  if (isRecent(weightSeen, weightAt, now)) t++;

  static uint8_t lastT = 255;
  if (t != lastT) {
    lastT = t;
    Serial.print(F("Triggers: "));
    Serial.println(t);
  }
  return t;
}

void clearSensorHistory() {
  motionSeen = soundSeen = weightSeen = false;
}

// ---------------------------------------------------------------------------
// ALERTS
// ---------------------------------------------------------------------------
void trySendSms() {
  const uint8_t fullMask = (uint8_t)((1 << NUM_CONTACTS) - 1);
  if (smsDoneMask == fullMask) return;

  unsigned long now = millis();
  if (haveSmsAttempt && (now - lastSmsAttemptAt) < smsGap) return;

  bool allOk = true;
  for (uint8_t i = 0; i < NUM_CONTACTS; i++) {
    if (smsDoneMask & (1 << i)) continue;
    if (sendSMS(CONTACTS[i], F("RESCUE ALERT: Child possibly detected in the car with the engine off. Check immediately!"))) {
      smsDoneMask |= (1 << i);
    } else {
      allOk = false;
    }
  }

  haveSmsAttempt = true;
  lastSmsAttemptAt = millis();
  smsGap = allOk ? SMS_COOLDOWN : SMS_RETRY_INTERVAL;
  lastReadingAt = millis();     // time spent sending isn't "reading lost"
}

void doPanicCalls() {
  for (uint8_t r = 0; r < CALL_ROUNDS; r++) {
    for (uint8_t i = 0; i < NUM_CONTACTS; i++) {
      Serial.print(F("Panic call round "));
      Serial.print(r + 1);
      Serial.print(F(" -> contact "));
      Serial.println(i + 1);

      gsmFlush();
      gsm.print(F("ATD"));
      gsm.print(CONTACTS[i]);
      gsm.println(';');                  // semicolon = voice call

      bool engineStillOff = waitMs(RING_TIME);
      gsm.println(F("ATH"));             // hang up
      if (!engineStillOff) return;       // engine started: abort

      if (!waitMs(CALL_GAP)) return;
    }
  }
}

// ---------------------------------------------------------------------------
// GSM FUNCTIONS
// ---------------------------------------------------------------------------
void gsmFlush() {
  while (gsm.available()) gsm.read();
}

// Wait until `expected` appears in the module's reply, or timeout.
bool gsmWaitFor(const char *expected, unsigned long timeoutMs) {
  char buf[48];
  uint8_t len = 0;
  buf[0] = '\0';
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (gsm.available()) {
      char c = gsm.read();
      if (len == sizeof(buf) - 1) {      // sliding window
        memmove(buf, buf + 1, len - 1);
        len--;
      }
      buf[len++] = c;
      buf[len] = '\0';
      if (strstr(buf, expected)) return true;
    }
  }
  return false;
}

bool gsmCommand(const __FlashStringHelper *cmd, const char *expected, unsigned long timeoutMs) {
  gsmFlush();
  gsm.println(cmd);
  return gsmWaitFor(expected, timeoutMs);
}

bool gsmRegistered() {
  gsmFlush();
  gsm.println(F("AT+CREG?"));
  char buf[40];
  uint8_t n = 0;
  unsigned long start = millis();
  while (millis() - start < 1500 && n < sizeof(buf) - 1) {
    if (gsm.available()) buf[n++] = gsm.read();
  }
  buf[n] = '\0';
  return strstr(buf, ",1") || strstr(buf, ",5");   // home or roaming
}

bool gsmInit() {
  bool alive = false;
  for (uint8_t i = 0; i < 10 && !alive; i++) {
    alive = gsmCommand(F("AT"), "OK", 1000);
  }
  if (!alive) return false;

  gsmCommand(F("ATE0"), "OK", 1000);   // echo off

  for (uint8_t i = 0; i < 30; i++) {   // wait up to ~45 s for the network
    if (gsmRegistered()) return true;
    delay(500);
  }
  return false;
}

bool sendSMS(const char *number, const __FlashStringHelper *message) {
  Serial.print(F("Sending SMS to "));
  Serial.println(number);

  if (!gsmCommand(F("AT+CMGF=1"), "OK", 2000)) {
    Serial.println(F("SMS failed: module not responding."));
    return false;
  }

  gsmFlush();
  gsm.print(F("AT+CMGS=\""));
  gsm.print(number);
  gsm.println('"');

  if (!gsmWaitFor(">", 5000)) {
    gsm.write(27);                     // ESC: cancel
    Serial.println(F("SMS failed: no prompt."));
    return false;
  }

  gsm.print(message);
  gsm.write(26);                       // CTRL+Z to send

  if (gsmWaitFor("+CMGS:", 20000)) {
    Serial.println(F("SMS sent."));
    return true;
  }
  Serial.println(F("SMS failed: no confirmation."));
  return false;
}
