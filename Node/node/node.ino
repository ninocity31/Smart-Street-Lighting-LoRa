// =============================================================================
// Smart street lighting - lamp node firmware
// Board: TTGO LoRa32 V2.1 (ESP32 + SX1276), Arduino IDE
// Libraries: LoRa by Sandeep Mistry
//
// The same code runs on every lamp post. Choose the role before uploading:
//   'M' = street master (has the HB-100 motion sensor, drives its slaves)
//   'E' = slave        (no motion sensor, follows the gateway and its master)
// =============================================================================

#include <SPI.h>
#include <LoRa.h>
#include <esp_task_wdt.h>

// -----------------------------------------------------------------------------
// 1. Settings
// -----------------------------------------------------------------------------
#define NODE_ROLE   'M'     // 'M' (master) or 'E' (slave)
#define NODE_ID     1       // 1..20, unique on the network
#define SLAVE_COUNT 4       // a master drives nodes NODE_ID+1 .. NODE_ID+SLAVE_COUNT

// LoRa radio (must be the same on the gateway)
#define LORA_FREQ      868E6
#define LORA_SF        9
#define LORA_BW        125E3
#define LORA_CR        5      // coding rate 4/5
#define LORA_TX_POWER  14     // dBm
#define LORA_SYNC_WORD 0x12   // private network

// TTGO LoRa32 V2.1 (T3 V1.6) pins
#define LORA_SCK  5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS   18
#define LORA_RST  23
#define LORA_DIO0 26

#define PIN_LED_PWM 25    // PWM to the LED driver (also the on-board LED)
#define PIN_LDR     34    // light sensor (ADC1, input only)
#define PIN_HB100   36    // amplified HB-100 output (ADC1, input only). GPIO35 is used by the battery divider.

// PWM
#define PWM_CHANNEL    0
#define PWM_FREQ       1000
#define PWM_RESOLUTION 8
#define PWM_OFF        0
#define PWM_LOW        77     // about 30 %
#define PWM_HIGH       255    // 100 %
#define PWM_STEP_MS    8      // smooth ramp: 1 step every 8 ms

// Timings
#define STATUS_INTERVAL_MS  300000UL   // status report every 5 min
#define COMMS_TIMEOUT_MS    7200000UL  // 2 h without a command = communication lost
#define WDT_TIMEOUT_S       10

// Lamp check with the LDR (ADC value goes up with light)
#define LDR_LIGHT_THRESHOLD 500
#define LDR_CHECK_DELAY_MS  3000
#define LDR_CHECK_RETRIES   3
#define LDR_DEAD_VALUE      5           // reading this low in daytime = sensor fault
#define LDR_DEAD_TIME_MS    600000UL    // for 10 min

// Motion (master only)
#define MOTION_THRESHOLD    300         // ADC counts away from the resting level
#define MOTION_HOLD_MS      2000        // one hit keeps "motion" for 2 s
#define MOTION_TIMEOUT_MS   60000UL     // light stays HIGH 60 s after the last motion
#define HIGH_REFRESH_MS     30000UL     // re-send HIGH to the slaves at most every 30 s
#define RELAY_INTERVAL_MS   1800000UL   // master repeats ON/OFF to its slaves every 30 min
#define HB100_RAIL_LOW      50
#define HB100_RAIL_HIGH     4045
#define HB100_DEAD_TIME_MS  10000UL

// Slave: falls back to LOW if the master's LOW command is lost
#define SLAVE_HIGH_HOLD_MS  (MOTION_TIMEOUT_MS + 30000UL)

// Error flags (same bits in the gateway)
#define ERR_NONE        0x00
#define ERR_HB100_FAIL  0x01
#define ERR_LDR_FAIL    0x02
#define ERR_COMMS_LOSS  0x04
#define ERR_LAMP_FAIL   0x08

// -----------------------------------------------------------------------------
// 2. Global variables
// -----------------------------------------------------------------------------
bool isNightMode = false;
int currentPwm = 0;
int targetPwm = 0;
int ldrValue = 0;
uint8_t errorFlags = ERR_NONE;

unsigned long lastStatusTime = 0;
unsigned long nextStatusDelay = 0;
unsigned long lastCommandTime = 0;
unsigned long lastPwmStep = 0;
unsigned long ldrLowSince = 0;

enum LdrCheckState { IDLE, WAITING };
LdrCheckState ldrCheckState = IDLE;
unsigned long ldrCheckStartTime = 0;
int ldrCheckRetriesLeft = 0;

#if NODE_ROLE == 'M'
bool motionState = false;
bool hadMotion = false;
bool slavesActivated = false;
unsigned long lastMotionTime = 0;
unsigned long lastHitTime = 0;
unsigned long lastHighSent = 0;
unsigned long hb100BadSince = 0;
float hb100Baseline = -1;
#else
bool highActive = false;
unsigned long highSince = 0;
#endif

// -----------------------------------------------------------------------------
// 3. Helpers
// -----------------------------------------------------------------------------
void pwmInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_LED_PWM, PWM_FREQ, PWM_RESOLUTION);
#else
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_LED_PWM, PWM_CHANNEL);
#endif
}

void pwmWrite(int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_LED_PWM, duty);
#else
  ledcWrite(PWM_CHANNEL, duty);
#endif
}

void watchdogInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t config = {};
  config.timeout_ms = WDT_TIMEOUT_S * 1000;
  config.idle_core_mask = 0;
  config.trigger_panic = true;
  esp_task_wdt_reconfigure(&config);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);
}

void sendLoRaMessage(const char* message) {
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();          // blocking, then the radio goes back to standby
  Serial.print("LoRa TX: ");
  Serial.println(message);
}

void sendStatus() {
  char msg[64];
#if NODE_ROLE == 'M'
  int motion = motionState ? 1 : 0;
#else
  int motion = 0;            // a slave has no motion sensor
#endif
  snprintf(msg, sizeof(msg), "STATUS,%d,%c,%d,%d,%d,%d",
           NODE_ID, NODE_ROLE, currentPwm, ldrValue, motion, errorFlags);
  sendLoRaMessage(msg);
  lastStatusTime = millis();
  nextStatusDelay = STATUS_INTERVAL_MS + random(0, 10000);   // avoid collisions
}

void setError(uint8_t flag, bool active) {
  uint8_t old = errorFlags;
  if (active) errorFlags |= flag;
  else        errorFlags &= ~flag;
  if (errorFlags != old) {
    Serial.print("Error flags: ");
    Serial.println(errorFlags);
    sendStatus();            // report a new fault right away
  }
}

void enterNight() {
  if (isNightMode) return;
  isNightMode = true;
#if NODE_ROLE == 'E'
  targetPwm = PWM_LOW;
#endif
  Serial.println("Night mode.");
}

void enterDay() {
  if (!isNightMode) return;
  isNightMode = false;
  targetPwm = PWM_OFF;
#if NODE_ROLE == 'M'
  slavesActivated = false;
#else
  highActive = false;
#endif
  Serial.println("Day mode.");
}

#if NODE_ROLE == 'M'
void sendCommandToSlaves(const char* command) {
  char msg[32];
  for (int i = 1; i <= SLAVE_COUNT; i++) {
    snprintf(msg, sizeof(msg), "CMD,%d,%s", NODE_ID + i, command);
    sendLoRaMessage(msg);
  }
}
#endif

// Smooth change of the light level
void applyPwmSmooth() {
  if (millis() - lastPwmStep < PWM_STEP_MS) return;
  lastPwmStep = millis();
  if (currentPwm < targetPwm)      currentPwm++;
  else if (currentPwm > targetPwm) currentPwm--;
  else return;
  pwmWrite(currentPwm);
}

void startLdrCheck() {
  ldrCheckState = WAITING;
  ldrCheckStartTime = millis();
  ldrCheckRetriesLeft = LDR_CHECK_RETRIES;
}

// After the lamp is switched on, the LDR must see its light
void handleLdrCheck() {
  if (ldrCheckState == IDLE) return;
  if (targetPwm == PWM_OFF) { ldrCheckState = IDLE; return; }
  if (millis() - ldrCheckStartTime < LDR_CHECK_DELAY_MS) return;

  if (ldrValue >= LDR_LIGHT_THRESHOLD) {
    setError(ERR_LAMP_FAIL, false);
    ldrCheckState = IDLE;
  } else if (--ldrCheckRetriesLeft <= 0) {
    Serial.println("Lamp fault: no light seen by the LDR.");
    setError(ERR_LAMP_FAIL, true);
    ldrCheckState = IDLE;
  } else {
    ldrCheckStartTime = millis();          // try again
  }
}

// -----------------------------------------------------------------------------
// 4. Main handlers (called in loop)
// -----------------------------------------------------------------------------
void handleSensors() {
  unsigned long now = millis();

  // LDR (all roles). In daytime it can never read 0, unless it is disconnected.
  ldrValue = analogRead(PIN_LDR);
  if (!isNightMode && ldrValue <= LDR_DEAD_VALUE) {
    if (ldrLowSince == 0) ldrLowSince = now;
    if (now - ldrLowSince > LDR_DEAD_TIME_MS) setError(ERR_LDR_FAIL, true);
  } else {
    ldrLowSince = 0;
    if (!isNightMode) setError(ERR_LDR_FAIL, false);
  }

#if NODE_ROLE == 'M'
  // The HB-100 gives an AC signal around a resting level (mid-supply).
  // Motion = the signal moves far enough away from that level.
  int x = analogRead(PIN_HB100);
  if (hb100Baseline < 0) hb100Baseline = x;
  hb100Baseline += (x - hb100Baseline) * 0.001f;        // slow average

  if (abs(x - (int)hb100Baseline) > MOTION_THRESHOLD) lastHitTime = now;
  bool reading = (lastHitTime != 0) && (now - lastHitTime < MOTION_HOLD_MS);

  // Sensor fault: the amplifier output stays stuck at a supply rail
  if (hb100Baseline < HB100_RAIL_LOW || hb100Baseline > HB100_RAIL_HIGH) {
    if (hb100BadSince == 0) hb100BadSince = now;
    if (now - hb100BadSince > HB100_DEAD_TIME_MS) setError(ERR_HB100_FAIL, true);
  } else {
    hb100BadSince = 0;
    setError(ERR_HB100_FAIL, false);
  }

  if (reading != motionState) {
    motionState = reading;
    Serial.println(motionState ? "Motion DETECTED" : "Motion ENDED");
  }
  if (motionState) {
    lastMotionTime = now;
    hadMotion = true;
  }
#endif
}

void runStateMachine() {
  unsigned long now = millis();

  // Fail-safe: no news from the gateway for too long -> keep a low light on
  bool commsLost = (now - lastCommandTime > COMMS_TIMEOUT_MS);
  if (commsLost && !(errorFlags & ERR_COMMS_LOSS)) {
    Serial.println("Communication lost: safe mode (light ON).");
    enterNight();
  }
  setError(ERR_COMMS_LOSS, commsLost);

#if NODE_ROLE == 'M'
  if (isNightMode) {
    bool motionActive = motionState || (hadMotion && now - lastMotionTime < MOTION_TIMEOUT_MS);
    if (motionActive) {
      targetPwm = PWM_HIGH;
      // tell the slaves once, then refresh only while there is real motion
      if (!slavesActivated || (motionState && now - lastHighSent > HIGH_REFRESH_MS)) {
        sendCommandToSlaves("HIGH");
        slavesActivated = true;
        lastHighSent = now;
      }
    } else {
      targetPwm = PWM_LOW;
      if (slavesActivated) {
        sendCommandToSlaves("LOW");
        slavesActivated = false;
      }
    }
  } else {
    targetPwm = PWM_OFF;
  }
#else
  // Slave: go back to LOW if the master's LOW command was lost
  if (highActive && now - highSince > SLAVE_HIGH_HOLD_MS) {
    highActive = false;
    if (isNightMode) targetPwm = PWM_LOW;
  }
#endif

  // Start a lamp check each time the lamp is switched on
  static int lastTarget = -1;
  if (targetPwm != lastTarget) {
    if (lastTarget <= PWM_OFF && targetPwm > PWM_OFF) startLdrCheck();
    lastTarget = targetPwm;
  }
}

void handleCommand(const char* target, const char* command) {
  bool isForMe = (atoi(target) == NODE_ID);
  bool isBroadcast = (strcmp(target, "ALL") == 0);
  if (!isForMe && !isBroadcast) return;

  // ON / OFF: night or day (from the gateway, or relayed by the master)
  if (strcmp(command, "ON") == 0 || strcmp(command, "OFF") == 0) {
    lastCommandTime = millis();          // the link with the gateway is alive
#if NODE_ROLE == 'M'
    bool wasNight = isNightMode;
#endif
    if (command[1] == 'N') enterNight();
    else                   enterDay();
#if NODE_ROLE == 'M'
    // Relay to the slaves (some may be out of the gateway's range)
    static unsigned long lastRelay = 0;
    if (isNightMode != wasNight || lastRelay == 0 || millis() - lastRelay > RELAY_INTERVAL_MS) {
      sendCommandToSlaves(command);
      lastRelay = millis();
    }
#endif
    return;
  }

#if NODE_ROLE == 'E'
  // HIGH / LOW: sent by the street master when it sees motion
  if (isForMe && isNightMode) {
    if (strcmp(command, "HIGH") == 0) {
      targetPwm = PWM_HIGH;
      highActive = true;
      highSince = millis();
      Serial.println("HIGH received.");
    } else if (strcmp(command, "LOW") == 0) {
      targetPwm = PWM_LOW;
      highActive = false;
      Serial.println("LOW received.");
    }
  }
#endif
}

// LoRa reception by polling (no SPI access inside an interrupt)
void handleLoRa() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;

  char buffer[64];
  int len = 0;
  while (LoRa.available()) {
    char c = (char)LoRa.read();
    if (len < (int)sizeof(buffer) - 1) buffer[len++] = c;
  }
  buffer[len] = '\0';
  Serial.print("LoRa RX: ");
  Serial.println(buffer);

  // Format: CMD,<ID|ALL>,<ON|OFF|HIGH|LOW>
  char* token = strtok(buffer, ",");
  if (token == NULL || strcmp(token, "CMD") != 0) return;   // ignore STATUS from other nodes
  char* target = strtok(NULL, ",");
  char* command = strtok(NULL, ",\r\n");
  if (target == NULL || command == NULL) return;
  handleCommand(target, command);
}

void handleReporting() {
  if (millis() - lastStatusTime >= nextStatusDelay) sendStatus();
}

// -----------------------------------------------------------------------------
// 5. setup / loop
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n--- Street light node ---");
  Serial.print("Node ID: ");   Serial.println(NODE_ID);
  Serial.print("Node role: "); Serial.println((char)NODE_ROLE);

  analogReadResolution(12);
  pinMode(PIN_LDR, INPUT);
#if NODE_ROLE == 'M'
  pinMode(PIN_HB100, INPUT);
#endif

  pwmInit();
  pwmWrite(0);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    while (1) delay(1000);
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();
  Serial.println("LoRa OK.");

  watchdogInit();

  randomSeed(esp_random());
  lastCommandTime = millis();
  lastStatusTime = millis();
  nextStatusDelay = 5000UL + NODE_ID * 2000UL;    // first report, spread by ID
}

void loop() {
  handleLoRa();
  handleSensors();
  runStateMachine();
  applyPwmSmooth();
  handleLdrCheck();
  handleReporting();
  esp_task_wdt_reset();
}
