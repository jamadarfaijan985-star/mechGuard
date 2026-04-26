/*
 ╔══════════════════════════════════════════════════════════════╗
 ║      LeakSense AI — ESP8266 (NodeMCU) Firmware  v2.2.0     ║
 ║      FLOW SENSORS ONLY  (No Pressure Sensor)               ║
 ║      2x YF-S201 Water Flow Sensors                         ║
 ╠══════════════════════════════════════════════════════════════╣
 ║  WIRING:                                                    ║
 ║  YF-S201 IN  (inlet)  YELLOW → D1 (GPIO 5)                ║
 ║  YF-S201 OUT (outlet) YELLOW → D2 (GPIO 4)                ║
 ║  Both sensors: RED → 5V,  BLACK → GND                      ║
 ║                                                             ║
 ║  ⚠ YF-S201 signal is 5V — use voltage divider:            ║
 ║    YELLOW ── 10kΩ ──┬── D1 or D2                          ║
 ║                     └── 20kΩ ── GND                        ║
 ║                                                             ║
 ║  Green  LED + 220Ω → D5 (GPIO 14)  NORMAL                 ║
 ║  Yellow LED + 220Ω → D6 (GPIO 12)  WARNING                ║
 ║  Red    LED + 220Ω → D7 (GPIO 13)  ALARM                  ║
 ║  Buzzer (optional) → D8 (GPIO 15)                          ║
 ║  Built-in LED      → D4 (GPIO  2)  heartbeat               ║
 ║                                                             ║
 ║  LIBRARIES (Arduino Library Manager):                       ║
 ║  • ESP8266WiFi      — bundled with ESP8266 board           ║
 ║  • ESP8266WebServer — bundled                              ║
 ║  • ArduinoJson v6.x — by Benoit Blanchon                  ║
 ║                                                             ║
 ║  Tools → Board → ESP8266 Boards → NodeMCU 1.0             ║
 ║  Tools → Upload Speed → 115200                            ║
 ╚══════════════════════════════════════════════════════════════╝
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>

// ══════════════════════════════════════════════
//  ①  EDIT — WiFi credentials
// ══════════════════════════════════════════════
const char* WIFI_SSID     = "Galaxy A13 3C43";
const char* WIFI_PASSWORD = "123456789";

// ══════════════════════════════════════════════
//  ②  EDIT — Sensor K-factor
//  YF-S201 = 7.5  |  YF-S401 = 5.5
//  YF-B1   = 3.0  |  YF-B5   = 1.8
// ══════════════════════════════════════════════
#define K_FACTOR_IN    7.5f
#define K_FACTOR_OUT   7.5f

// ══════════════════════════════════════════════
//  ③  EDIT — Leak thresholds (LOWERED for sensitivity)
// ══════════════════════════════════════════════
#define MIN_FLOW_TO_CHECK    0.5f   // L/min — below this = pipe idle (was 1.0)
#define WARNING_LOSS_PCT     3.0f   // 3%  loss → WARNING  (was 5%)
#define ALARM_LOSS_PCT       8.0f   // 8%  loss → ALARM    (was 15%)
#define CONFIRM_COUNT_WARN   2      // consecutive readings for WARNING
#define CONFIRM_COUNT_ALARM  3      // consecutive readings for ALARM
#define SPIKE_THRESHOLD     15.0f   // L/min sudden jump → WARNING (was 20)

// ══════════════════════════════════════════════
//  SIMULATION MODE
//  false = real sensors  |  true = fake data
// ══════════════════════════════════════════════
#define USE_SIMULATION  false

// ──────────────────────────────────────────────
//  PINS
// ──────────────────────────────────────────────
#define PIN_FLOW_IN    5    // D1
#define PIN_FLOW_OUT   4    // D2
#define PIN_LED_OK    14    // D5 green
#define PIN_LED_WARN  12    // D6 yellow
#define PIN_LED_ALARM 13    // D7 red
#define PIN_BUZZER    15    // D8  (set to -1 to disable)
#define PIN_LED_HB     2    // D4 built-in heartbeat

// ──────────────────────────────────────────────
//  TIMING
// ──────────────────────────────────────────────
#define MEASURE_MS  1000

// ──────────────────────────────────────────────
//  PULSE COUNTERS
// ──────────────────────────────────────────────
volatile unsigned long flowPulseIn  = 0;
volatile unsigned long flowPulseOut = 0;

ICACHE_RAM_ATTR void onFlowPulseIn()  { flowPulseIn++;  }
ICACHE_RAM_ATTR void onFlowPulseOut() { flowPulseOut++; }

// ──────────────────────────────────────────────
//  GLOBALS
// ──────────────────────────────────────────────
ESP8266WebServer server(80);

float flowIn      = 0.0f;
float flowOut     = 0.0f;
float flowInPrev  = 0.0f;
float lossLpm     = 0.0f;
float lossPct     = 0.0f;
float totalVolIn  = 0.0f;
float totalVolOut = 0.0f;

// ✅ FIX: Rolling average buffers to smooth noisy sensor readings
#define AVG_SIZE 5
float flowInBuf[AVG_SIZE]  = {0};
float flowOutBuf[AVG_SIZE] = {0};
int   avgIdx = 0;

int   alarmState    = 0;   // 0=NORMAL 1=WARNING 2=ALARM
int   warnCount     = 0;
int   alarmCount_c  = 0;
unsigned long alarmEvents = 0;

float simPhase      = 0.0f;
bool  simLeakActive = false;
int   simLeakTimer  = 0;

unsigned long readingCount  = 0;
unsigned long requestCount  = 0;
unsigned long startMs       = 0;
unsigned long lastMeasureMs = 0;

// ──────────────────────────────────────────────
//  FUNCTION PROTOTYPES
// ──────────────────────────────────────────────
void connectWiFi();
void ledTest();
void measureFlow(float dt);
void simulateFlow();
void detectLeak();
void updateOutputs();
void addCORS();
void handleCORS();
void handleData();
void handleStatus();
void handleRoot();
void handleReset();
void handleNotFound();
float rollingAvg(float* buf, float newVal);

// ──────────────────────────────────────────────
//  SETUP
// ──────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(150);
  Serial.println(F("\n╔══════════════════════════════════════╗"));
  Serial.println(F("║  LeakSense AI — ESP8266 v2.2.0       ║"));
  Serial.println(F("║  2x YF-S201 Flow Sensors             ║"));
  Serial.println(F("╚══════════════════════════════════════╝"));

  pinMode(PIN_LED_OK,    OUTPUT);
  pinMode(PIN_LED_WARN,  OUTPUT);
  pinMode(PIN_LED_ALARM, OUTPUT);
  pinMode(PIN_LED_HB,    OUTPUT);
  if (PIN_BUZZER >= 0) pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_LED_HB, HIGH); // built-in LED off (active LOW)

  ledTest();

  if (!USE_SIMULATION) {
    pinMode(PIN_FLOW_IN,  INPUT_PULLUP);
    pinMode(PIN_FLOW_OUT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_FLOW_IN),
                    onFlowPulseIn,  FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_FLOW_OUT),
                    onFlowPulseOut, FALLING);
    Serial.println(F("[SENSOR] Real YF-S201 on D1 + D2"));
  } else {
    Serial.println(F("[SENSOR] Simulation mode active"));
  }

  connectWiFi();

  server.on("/",       HTTP_GET,     handleRoot);
  server.on("/data",   HTTP_GET,     handleData);
  server.on("/data",   HTTP_OPTIONS, handleCORS);
  server.on("/status", HTTP_GET,     handleStatus);
  server.on("/reset",  HTTP_GET,     handleReset);
  server.onNotFound(handleNotFound);
  server.begin();

  startMs = millis();
  Serial.println(F("[INFO] System ready."));
  Serial.printf("[INFO] Data URL: http://%s/data\n",
                WiFi.localIP().toString().c_str());
  Serial.println(F("[INFO] Thresholds: WARN=3% ALARM=8% MIN_FLOW=0.5 L/min"));
  Serial.println();
}

// ──────────────────────────────────────────────
//  MAIN LOOP
// ──────────────────────────────────────────────
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[WIFI] Reconnecting..."));
    connectWiFi();
  }

  server.handleClient();

  unsigned long now = millis();
  if (now - lastMeasureMs >= MEASURE_MS) {
    float dt = (now - lastMeasureMs) / 1000.0f;
    lastMeasureMs = now;
    readingCount++;

    if (USE_SIMULATION) {
      simulateFlow();
    } else {
      measureFlow(dt);
    }

    // Accumulate total volume (L/min × min = Litres)
    totalVolIn  += flowIn  * (dt / 60.0f);
    totalVolOut += flowOut * (dt / 60.0f);

    // ✅ FIX: Compute loss WITHOUT clamping flowOut
    // This is the REAL loss — don't hide it
    if (flowIn >= MIN_FLOW_TO_CHECK) {
      lossLpm = flowIn - flowOut;          // can be negative (sensor noise)
      lossLpm = max(0.0f, lossLpm);        // floor at 0
      lossPct = (lossLpm / flowIn) * 100.0f;
      lossPct = constrain(lossPct, 0.0f, 100.0f);
    } else {
      lossLpm = 0.0f;
      lossPct = 0.0f;
    }

    detectLeak();
    updateOutputs();

    // Heartbeat blink
    digitalWrite(PIN_LED_HB, LOW);
    delay(25);
    digitalWrite(PIN_LED_HB, HIGH);

    // ✅ FIX: Enhanced serial debug — shows raw values for calibration
    Serial.printf(
      "[DATA] IN=%.3f OUT=%.3f LOSS=%.3f L/min (%.2f%%)  STATE=%s\n",
      flowIn, flowOut, lossLpm, lossPct,
      alarmState == 0 ? "NORMAL" :
      alarmState == 1 ? "WARNING" : "ALARM"
    );
  }
}

// ──────────────────────────────────────────────
//  ROLLING AVERAGE HELPER
// ──────────────────────────────────────────────
float rollingAvg(float* buf, float newVal) {
  buf[avgIdx % AVG_SIZE] = newVal;
  float sum = 0;
  for (int i = 0; i < AVG_SIZE; i++) sum += buf[i];
  return sum / AVG_SIZE;
}

// ──────────────────────────────────────────────
//  MEASURE REAL FLOW
//  ✅ FIX: Removed the flowOut clamping line that
//  was hiding real leaks:
//    REMOVED: flowOut = min(flowOut, flowIn * 1.02f)
//  ✅ FIX: Added rolling average to reduce noise
// ──────────────────────────────────────────────
void measureFlow(float dt) {
  // Read and reset pulse counters atomically
  noInterrupts();
  unsigned long pIn  = flowPulseIn;
  unsigned long pOut = flowPulseOut;
  flowPulseIn  = 0;
  flowPulseOut = 0;
  interrupts();

  // pulses/dt = Hz,  Hz / K_FACTOR = L/min
  flowInPrev = flowIn;

  float rawIn  = (pIn  / dt) / K_FACTOR_IN;
  float rawOut = (pOut / dt) / K_FACTOR_OUT;

  // Clamp negatives only
  rawIn  = max(0.0f, rawIn);
  rawOut = max(0.0f, rawOut);

  // Apply rolling average to smooth noise
  flowIn  = rollingAvg(flowInBuf,  rawIn);
  flowOut = rollingAvg(flowOutBuf, rawOut);

  avgIdx++;  // advance rolling buffer index

  // ❌ REMOVED: flowOut = min(flowOut, flowIn * 1.02f)
  // This line was HIDING leaks by capping outlet flow.
  // Real leaks show as flowOut < flowIn — we NEED that difference.
}

// ──────────────────────────────────────────────
//  SIMULATE FLOW (no real sensors needed)
// ──────────────────────────────────────────────
void simulateFlow() {
  simPhase += 0.05f;
  simLeakTimer++;

  if (!simLeakActive && random(1000) < 13) {
    simLeakActive = true;
    simLeakTimer  = 0;
    Serial.println(F("[SIM] Leak started!"));
  }
  if (simLeakActive && simLeakTimer > 50) {
    simLeakActive = false;
    Serial.println(F("[SIM] Leak ended."));
  }

  flowInPrev = flowIn;
  flowIn  = max(5.0f, 50.0f + sin(simPhase) * 8.0f + (random(40) / 10.0f));

  float leakFactor = simLeakActive
    ? 0.05f + (simLeakTimer / 50.0f) * 0.25f
    : random(20) / 1000.0f;

  flowOut = max(2.0f, flowIn * (1.0f - leakFactor));
}

// ──────────────────────────────────────────────
//  LEAK DETECTION (3 methods)
// ──────────────────────────────────────────────
void detectLeak() {
  int newState = 0;

  if (flowIn < MIN_FLOW_TO_CHECK) {
    warnCount    = 0;
    alarmCount_c = 0;
    alarmState   = 0;
    return;
  }

  // Method A — loss % threshold
  if (lossPct >= ALARM_LOSS_PCT) {
    alarmCount_c++;
    warnCount = CONFIRM_COUNT_WARN;
    newState = (alarmCount_c >= CONFIRM_COUNT_ALARM) ? 2 : 1;
    Serial.printf("[DETECT-A] ALARM level: %.2f%% loss, count=%d\n",
                  lossPct, alarmCount_c);
  } else if (lossPct >= WARNING_LOSS_PCT) {
    warnCount++;
    alarmCount_c = 0;
    if (warnCount >= CONFIRM_COUNT_WARN) newState = max(newState, 1);
    Serial.printf("[DETECT-A] WARN level: %.2f%% loss, count=%d\n",
                  lossPct, warnCount);
  } else {
    warnCount    = max(0, warnCount    - 1);
    alarmCount_c = max(0, alarmCount_c - 1);
  }

  // Method B — no outlet flow while inlet active (burst pipe)
  if (flowIn > MIN_FLOW_TO_CHECK * 2.0f && flowOut < 0.5f) {
    newState = max(newState, 2);
    Serial.printf("[DETECT-B] NO OUTLET! IN=%.2f OUT=%.2f\n", flowIn, flowOut);
  }

  // Method C — sudden inlet spike
  float spike = flowIn - flowInPrev;
  if (spike > SPIKE_THRESHOLD) {
    newState = max(newState, 1);
    Serial.printf("[DETECT-C] SPIKE +%.2f L/min\n", spike);
  }

  if (newState == 2 && alarmState < 2) {
    alarmEvents++;
    Serial.printf("[ALARM] *** LEAK CONFIRMED! %.2f%% (%.3f L/min) ***\n",
                  lossPct, lossLpm);
  }

  alarmState = newState;
}

// ──────────────────────────────────────────────
//  LEDs + BUZZER
// ──────────────────────────────────────────────
void updateOutputs() {
  digitalWrite(PIN_LED_OK,    LOW);
  digitalWrite(PIN_LED_WARN,  LOW);
  digitalWrite(PIN_LED_ALARM, LOW);
  if (PIN_BUZZER >= 0) digitalWrite(PIN_BUZZER, LOW);

  if (alarmState == 0) {
    digitalWrite(PIN_LED_OK, HIGH);
  } else if (alarmState == 1) {
    digitalWrite(PIN_LED_WARN,
      (millis() / 500) % 2 ? HIGH : LOW);
  } else {
    digitalWrite(PIN_LED_ALARM, HIGH);
    if (PIN_BUZZER >= 0)
      digitalWrite(PIN_BUZZER, (millis() / 250) % 2 ? HIGH : LOW);
  }
}

// ──────────────────────────────────────────────
//  LED SELF-TEST
// ──────────────────────────────────────────────
void ledTest() {
  digitalWrite(PIN_LED_OK,    HIGH);
  digitalWrite(PIN_LED_WARN,  HIGH);
  digitalWrite(PIN_LED_ALARM, HIGH);
  if (PIN_BUZZER >= 0) digitalWrite(PIN_BUZZER, HIGH);
  delay(600);
  digitalWrite(PIN_LED_OK,    LOW);
  digitalWrite(PIN_LED_WARN,  LOW);
  digitalWrite(PIN_LED_ALARM, LOW);
  if (PIN_BUZZER >= 0) digitalWrite(PIN_BUZZER, LOW);
  delay(200);
}

// ──────────────────────────────────────────────
//  WIFI
// ──────────────────────────────────────────────
void connectWiFi() {
  Serial.printf("[WIFI] Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("\n[WIFI] Connected!"));
    Serial.printf("[WIFI] IP  : %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WIFI] RSSI: %d dBm\n", WiFi.RSSI());
  } else {
    Serial.println(F("\n[WIFI] FAILED — check SSID/password"));
  }
}

// ──────────────────────────────────────────────
//  CORS
// ──────────────────────────────────────────────
void addCORS() {
  server.sendHeader("Access-Control-Allow-Origin",  "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.sendHeader("Cache-Control",                "no-cache");
}
void handleCORS() { addCORS(); server.send(200, "text/plain", "OK"); }

// ──────────────────────────────────────────────
//  GET /data
// ──────────────────────────────────────────────
void handleData() {
  addCORS();
  requestCount++;

  StaticJsonDocument<320> doc;
  doc["flowIn"]      = round(flowIn      * 100.0f) / 100.0f;
  doc["flowOut"]     = round(flowOut     * 100.0f) / 100.0f;
  doc["loss"]        = round(lossPct     * 100.0f) / 100.0f;
  doc["lossLpm"]     = round(lossLpm     * 100.0f) / 100.0f;
  doc["pressure"]    = 0;
  doc["totalVolIn"]  = round(totalVolIn  *  10.0f) /  10.0f;
  doc["totalVolOut"] = round(totalVolOut *  10.0f) /  10.0f;
  doc["alarmState"]  = alarmState;
  doc["alarmLabel"]  = alarmState == 0 ? "NORMAL"
                     : alarmState == 1 ? "WARNING" : "ALARM";
  doc["alarmEvents"] = alarmEvents;
  doc["uptime"]      = (millis() - startMs) / 1000UL;
  doc["readings"]    = readingCount;
  doc["rssi"]        = WiFi.RSSI();
  doc["simMode"]     = USE_SIMULATION;
  doc["timestamp"]   = millis();

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

// ──────────────────────────────────────────────
//  GET /status
// ──────────────────────────────────────────────
void handleStatus() {
  addCORS();
  StaticJsonDocument<256> doc;
  doc["device"]   = "LeakSense-ESP8266-FlowOnly";
  doc["firmware"] = "2.2.0";
  doc["uptime"]   = (millis() - startMs) / 1000UL;
  doc["requests"] = requestCount;
  doc["ip"]       = WiFi.localIP().toString();
  doc["rssi"]     = WiFi.RSSI();
  doc["freeHeap"] = ESP.getFreeHeap();
  doc["simMode"]  = USE_SIMULATION;
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

// ──────────────────────────────────────────────
//  GET /
// ──────────────────────────────────────────────
void handleRoot() {
  addCORS();
  String ip = WiFi.localIP().toString();
  String html = R"(<!DOCTYPE html>
<html><head><meta charset='UTF-8'/>
<title>LeakSense ESP8266</title>
<style>
  body{font-family:monospace;background:#020b14;color:#00d4ff;
       padding:30px;text-align:center;}
  h1{font-size:22px;} p{color:#7ab8d4;font-size:13px;} a{color:#00ff88;}
  .card{display:inline-block;margin:10px;padding:18px 28px;
        border:1px solid rgba(0,212,255,0.3);border-radius:10px;
        background:rgba(0,212,255,0.05);}
  .big{font-size:24px;font-weight:bold;color:#00ff88;
       margin-top:10px;display:block;}
</style></head><body>
<h1>💧 LeakSense AI v2.2.0 — Flow Sensor Node Online</h1>
<p>ESP8266 | 2x YF-S201 | No Pressure Sensor</p>
<p>IP: <strong>)" + ip + R"(</strong></p><br/>
<div class='card'><a href='/data'>📡 /data</a><br/><small>Sensor JSON</small></div>
<div class='card'><a href='/status'>⚡ /status</a><br/><small>Device Health</small></div>
<br/><p>Paste into LeakSense dashboard:</p>
<span class='big'>http://)" + ip + R"(/data</span>
</body></html>)";
  server.send(200, "text/html", html);
}

// ──────────────────────────────────────────────
//  GET /reset
// ──────────────────────────────────────────────
void handleReset() {
  addCORS();
  readingCount = requestCount = alarmEvents = 0;
  totalVolIn = totalVolOut = 0;
  startMs = millis();
  server.send(200, "application/json", "{\"reset\":true}");
}

void handleNotFound() {
  addCORS();
  server.send(404, "application/json", "{\"error\":\"Not found\"}");
}
