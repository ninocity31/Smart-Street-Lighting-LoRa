// =============================================================================
// Smart street lighting - central gateway firmware
// Board: Arduino Uno + RFM95W (LoRa 868 MHz) + SIM800L (GPRS)
// Libraries: LoRa by Sandeep Mistry, TinyGSM
//
// What it does:
//   - gets the time from the network and computes sunrise / sunset
//   - sends "CMD,ALL,ON" at night and "CMD,ALL,OFF" during the day
//   - receives the STATUS reports of the lamp nodes
//   - checks if a lamp is too dark at full power (brightness fault)
//   - every 5 minutes, sends a small binary frame to the server over GPRS
// =============================================================================

#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 64

#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <TinyGsmClient.h>
#include <avr/wdt.h>

// -----------------------------------------------------------------------------
// 1. Settings
// -----------------------------------------------------------------------------
#define MAX_NODES 20

// LoRa radio (must be the same on the nodes)
#define LORA_FREQ      868E6
#define LORA_SF        9
#define LORA_BW        125E3
#define LORA_CR        5
#define LORA_TX_POWER  14
#define LORA_SYNC_WORD 0x12

// Pins (Arduino Uno)
#define LORA_SS     10
#define LORA_RST    9
#define LORA_DIO0   2
#define SIM_RX_PIN  7     // Uno receives on 7  <- SIM800L TXD
#define SIM_TX_PIN  8     // Uno sends on 8     -> SIM800L RXD (through a divider)

// GPRS and server (fill in your own values)
const char APN[]         = "your.apn";
const char GPRS_USER[]   = "";
const char GPRS_PASS[]   = "";
const char SERVER_HOST[] = "your.server.com";
const uint16_t SERVER_PORT = 5000;

// Site position for the sunrise / sunset calculation (example: Paris)
#define SITE_LAT  48.8566
#define SITE_LON  2.3522

// Timings
#define CMD_INTERVAL_MS        60000UL      // ON/OFF broadcast every minute
#define SEND_INTERVAL_MS       300000UL     // data to the server every 5 min
#define TIME_SYNC_INTERVAL_MS  21600000UL   // new time sync every 6 h
#define TIME_RETRY_MS          60000UL

// Brightness check: at full power the LDR must see enough light
#define PWM_THRESHOLD_BRIGHT   200
#define LDR_MIN_AT_FULL_POWER  1500

// Error flags (bits 0-3 come from the nodes)
#define ERROR_HB100_FAIL         0x01
#define ERROR_LDR_FAIL           0x02
#define ERROR_COMMS_LOSS         0x04
#define ERROR_LAMP_FAIL          0x08
#define ERROR_BRIGHTNESS_FAILURE 0x80   // found by the gateway

// -----------------------------------------------------------------------------
// 2. Objects and variables
// -----------------------------------------------------------------------------
SoftwareSerial simSerial(SIM_RX_PIN, SIM_TX_PIN);
TinyGsm modem(simSerial);
TinyGsmClient client(modem);

struct NodeStatus {
  uint8_t nodeId;
  uint8_t errorFlags;
  uint8_t lampState;   // 0 = OFF, 1 = ON
  char    nodeRole;    // 'M' or 'E'
  bool    updated;
};
NodeStatus nodeData[MAX_NODES];

bool timeValid = false;
bool isNightGlobal = false;
long syncMinuteOfDay = 0;           // UTC minutes at the last sync
unsigned long syncMillis = 0;
float sunriseMin = 0, sunsetMin = 0;

unsigned long lastCmdTime = 0;
unsigned long lastSendTime = 0;
unsigned long lastSyncTry = 0;

// -----------------------------------------------------------------------------
// 3. Time and day / night
// -----------------------------------------------------------------------------
// Sunrise or sunset time in UTC minutes (simple almanac formula, about 1 min accuracy).
// Written here because the usual sunrise library no longer builds for AVR boards.
float sunTimeUTC(int year, int month, int day, float lat, float lon, bool rising) {
  int n1 = 275 * month / 9;
  int n2 = (month + 9) / 12;
  int n3 = 1 + (year - 4 * (year / 4) + 2) / 3;
  int n = n1 - n2 * n3 + day - 30;                      // day of the year

  float lngHour = lon / 15.0;
  float t = n + ((rising ? 6.0 : 18.0) - lngHour) / 24.0;
  float m = 0.9856 * t - 3.289;
  float l = m + 1.916 * sin(radians(m)) + 0.020 * sin(radians(2 * m)) + 282.634;
  l = fmod(l + 360.0, 360.0);

  float ra = fmod(degrees(atan(0.91764 * tan(radians(l)))) + 360.0, 360.0);
  ra += floor(l / 90.0) * 90.0 - floor(ra / 90.0) * 90.0;
  ra /= 15.0;

  float sinDec = 0.39782 * sin(radians(l));
  float cosDec = cos(asin(sinDec));
  float cosH = (cos(radians(90.833)) - sinDec * sin(radians(lat))) / (cosDec * cos(radians(lat)));
  if (cosH > 1.0) cosH = 1.0;                           // polar cases, not used here
  if (cosH < -1.0) cosH = -1.0;

  float h = rising ? 360.0 - degrees(acos(cosH)) : degrees(acos(cosH));
  float ut = h / 15.0 + ra - 0.06571 * t - 6.622 - lngHour;
  ut = fmod(ut + 48.0, 24.0);
  return ut * 60.0;
}

// The modem can block for a long time: stop the watchdog during these calls.
void watchdogPause()  { wdt_disable(); }
void watchdogResume() { wdt_enable(WDTO_8S); }

bool connectGprs() {
  if (modem.isGprsConnected()) return true;
  Serial.println(F("GPRS: connecting..."));
  if (!modem.waitForNetwork(60000L)) return false;
  return modem.gprsConnect(APN, GPRS_USER, GPRS_PASS);
}

bool syncTime() {
  watchdogPause();
  bool ok = false;
  int y, mo, d, h, mi, s;
  float tz;
  if (connectGprs()) {
    modem.NTPServerSync("pool.ntp.org", 0);            // modem clock in UTC
    if (modem.getNetworkTime(&y, &mo, &d, &h, &mi, &s, &tz) && y >= 2024) {
      syncMinuteOfDay = h * 60L + mi;
      syncMillis = millis();
      sunriseMin = sunTimeUTC(y, mo, d, SITE_LAT, SITE_LON, true);    // everything in UTC
      sunsetMin  = sunTimeUTC(y, mo, d, SITE_LAT, SITE_LON, false);
      ok = true;
      Serial.print(F("Time OK (UTC min): ")); Serial.println(syncMinuteOfDay);
      Serial.print(F("Sunrise: ")); Serial.print(sunriseMin);
      Serial.print(F(" Sunset: ")); Serial.println(sunsetMin);
    }
  }
  watchdogResume();
  if (!ok) Serial.println(F("Time sync failed."));
  return ok;
}

long minuteOfDayUTC() {
  return (syncMinuteOfDay + (millis() - syncMillis) / 60000UL) % 1440;
}

void updateDayNight() {
  if (!timeValid) return;
  long now = minuteOfDayUTC();
  if (sunriseMin < sunsetMin) isNightGlobal = (now >= sunsetMin) || (now < sunriseMin);
  else                        isNightGlobal = (now >= sunsetMin) && (now < sunriseMin);   // sunset after 00:00 UTC
}

// -----------------------------------------------------------------------------
// 4. LoRa communication
// -----------------------------------------------------------------------------
void sendLoRaMessage(const char* message) {
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();
  Serial.print(F("LoRa TX: "));
  Serial.println(message);
}

// Format: STATUS,<ID>,<ROLE>,<PWM>,<LDR>,<MOTION>,<ERRORS>
void handleStatus(char* buffer) {
  strtok(buffer, ",");                       // "STATUS"
  char* id_str     = strtok(NULL, ",");
  char* role_str   = strtok(NULL, ",");
  char* pwm_str    = strtok(NULL, ",");
  char* ldr_str    = strtok(NULL, ",");
  char* motion_str = strtok(NULL, ",");
  char* error_str  = strtok(NULL, ",\r\n");
  if (!(id_str && role_str && pwm_str && ldr_str && motion_str && error_str)) return;

  int nodeId = atoi(id_str);
  if (nodeId <= 0 || nodeId > MAX_NODES) return;
  int index = nodeId - 1;

  int pwmValue = atoi(pwm_str);
  int ldrValue = atoi(ldr_str);
  uint8_t nodeErrors = (uint8_t)atoi(error_str);

  // Brightness analysis: full power at night but not enough light
  uint8_t gatewayErrors = 0;
  if (isNightGlobal && pwmValue > PWM_THRESHOLD_BRIGHT && ldrValue < LDR_MIN_AT_FULL_POWER) {
    gatewayErrors |= ERROR_BRIGHTNESS_FAILURE;
  }

  nodeData[index].lampState  = (pwmValue > 0) ? 1 : 0;
  nodeData[index].nodeRole   = role_str[0];
  nodeData[index].errorFlags = nodeErrors | gatewayErrors;
  nodeData[index].updated    = true;

  Serial.print(F("Status stored, node "));
  Serial.println(nodeId);
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
  Serial.print(F("LoRa RX: "));
  Serial.println(buffer);

  if (strncmp(buffer, "STATUS,", 7) == 0) handleStatus(buffer);
  // "CMD,..." packets from the street masters are ignored
}

void handleCommands() {
  if (!timeValid) return;                      // no time = no command
  if (millis() - lastCmdTime < CMD_INTERVAL_MS) return;
  lastCmdTime = millis();
  sendLoRaMessage(isNightGlobal ? "CMD,ALL,ON" : "CMD,ALL,OFF");
}

// -----------------------------------------------------------------------------
// 5. GPRS communication
// -----------------------------------------------------------------------------
// Binary frame: 4 bytes per updated node = ID, ERRORS, LAMP_STATE, ROLE
int buildGprsFrame(byte* buffer, int maxSize) {
  int bufferIndex = 0;
  for (int i = 0; i < MAX_NODES; i++) {
    if (!nodeData[i].updated) continue;
    if (bufferIndex + 4 > maxSize) break;
    buffer[bufferIndex++] = nodeData[i].nodeId;
    buffer[bufferIndex++] = nodeData[i].errorFlags;
    buffer[bufferIndex++] = nodeData[i].lampState;
    buffer[bufferIndex++] = (byte)nodeData[i].nodeRole;
  }
  return bufferIndex;
}

bool sendFrame(const byte* frame, int len) {
  watchdogPause();
  bool ok = false;
  if (connectGprs() && client.connect(SERVER_HOST, SERVER_PORT)) {
    ok = (client.write(frame, len) == (size_t)len);
    client.stop();
  }
  watchdogResume();
  return ok;
}

void handleServer() {
  if (millis() - lastSendTime < SEND_INTERVAL_MS) return;
  lastSendTime = millis();

  byte frame[MAX_NODES * 4];
  int len = buildGprsFrame(frame, sizeof(frame));
  if (len == 0) return;                        // nothing new

  if (sendFrame(frame, len)) {
    for (int i = 0; i < MAX_NODES; i++) nodeData[i].updated = false;   // clear only after success
    Serial.print(F("GPRS: sent ")); Serial.print(len); Serial.println(F(" bytes"));
  } else {
    Serial.println(F("GPRS: send failed, data kept for next time"));
  }
}

void handleTimeSync() {
  bool due = timeValid ? (millis() - syncMillis > TIME_SYNC_INTERVAL_MS)
                       : (millis() - lastSyncTry > TIME_RETRY_MS);
  if (!due) return;
  lastSyncTry = millis();
  if (syncTime()) timeValid = true;
  else if (timeValid) syncMillis = millis() - TIME_SYNC_INTERVAL_MS + TIME_RETRY_MS;   // retry in 1 min, keep old time
}

// -----------------------------------------------------------------------------
// 6. setup / loop
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n--- Street light gateway ---"));

  for (int i = 0; i < MAX_NODES; i++) {
    nodeData[i].nodeId = i + 1;
    nodeData[i].errorFlags = 0;
    nodeData[i].lampState = 0;
    nodeData[i].nodeRole = '?';
    nodeData[i].updated = false;
  }

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("LoRa init failed!"));
    while (1);
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();
  Serial.println(F("LoRa OK."));

  simSerial.begin(9600);
  Serial.println(F("Modem start..."));
  modem.restart();
  if (syncTime()) timeValid = true;
  lastSyncTry = millis();

  watchdogResume();
}

void loop() {
  handleLoRa();
  handleTimeSync();
  updateDayNight();
  handleCommands();
  handleServer();
  wdt_reset();
}
