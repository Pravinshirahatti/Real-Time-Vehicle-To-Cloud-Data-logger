/*
  
  ================================================
  Dual publish: Blynk IoT dashboard + MQTT → Spring Boot backend → PostgreSQL

  Sensors  : HX710B (tire pressure) | LM35 (temperature) | INA219 (voltage/current)
             ADXL345 (accelerometer) | KY-024 Hall sensor (speed) | SIM808 (GPS + SMS)

  MQTT topic : ev/{VEHICLE_VIN}/telemetry  →  your website live dashboard
  Blynk      : virtual pins V1–V12         →  Blynk mobile/web dashboard

  Libraries (Arduino Library Manager):
    - PubSubClient          by Nick O'Leary
    - ArduinoJson           by Benoit Blanchon  (v6.x)
    - Blynk                 by Volodymyr Shymanskyy
    - Adafruit INA219
    - Adafruit ADXL345
    - Adafruit Unified Sensor
*/

// ================================================================
//  BLYNK — must be defined before any includes
// ================================================================
#define BLYNK_TEMPLATE_ID    "TMPL3VONicBeY"
#define BLYNK_TEMPLATE_NAME  "IOT vehicle data"
#define BLYNK_AUTH_TOKEN     "Daqm9oKgiPSJBAfVvQttBiHPc64j1oR5"
#define BLYNK_PRINT          Serial

// Blynk virtual pin map
#define VPIN_PRESSURE    V1   // tire pressure (bar)
#define VPIN_CURRENT     V2   // battery current (mA)
#define VPIN_ACCEL       V7   // accelerometer string
#define VPIN_CRASH_MSG   V8   // crash status label
#define VPIN_VOLTAGE     V9   // battery voltage (V)
#define VPIN_GPS_LINK    V10  // Google Maps link on crash
#define VPIN_TEMP        V11  // temperature (°C)
#define VPIN_SPEED       V12  // speed (km/h)

// ================================================================
//  CONFIG — edit before flashing
// ================================================================
#define WIFI_SSID           "VivoY725G"
#define WIFI_PASSWORD       "123456789"

// Laptop LAN IP — run ipconfig (Windows) to confirm before each session
#define MQTT_BROKER_IP      "10.249.211.199"
#define MQTT_PORT           1883
#define MQTT_CLIENT_ID      "esp32-ev-001"

// Must match a VIN in the backend vehicle table
#define VEHICLE_VIN         "1HGBH41JXMN109186"

#define PUBLISH_INTERVAL_MS  500    // normal telemetry interval
#define CRASH_G_THRESHOLD    12.0f  // must match TelemetryService.java
#define ALERT_PHONE         "+917588161021"

// ================================================================
//  PIN ASSIGNMENTS
// ================================================================
#define HX_DT      4    // HX710B data
#define HX_SCK     14   // HX710B clock
#define LM35_PIN   34   // LM35 analog (ADC1)
#define HALL_PIN   35   // KY-024 DO (digital output) — attach to interrupt-capable pin
// I2C shared bus : SDA=GPIO21, SCL=GPIO22  (INA219 + ADXL345)
// SIM808 UART    : RX=GPIO16,  TX=GPIO17

// ================================================================
//  LIBRARIES
// ================================================================
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_INA219.h>
#include <math.h>

// ================================================================
//  GLOBALS
// ================================================================

// --- MQTT ---
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);
char         mqttTopic[64];

// --- HX710B ---
long    hxOffset     = 0;
#define PRESSURE_SCALE  10000.0f   // calibrate: raw_at_known_PSI / known_PSI

// --- INA219 ---
Adafruit_INA219 ina219;

// --- ADXL345 ---
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// --- KY-024 Hall sensor speed calibration ---
// KY-024 DO goes LOW each time the magnet passes the sensor face.
// Tune these two values to match your physical setup:
#define MAGNETS_PER_REV   1        // number of magnets glued to the wheel rim
#define WHEEL_RADIUS_M    0.30f    // wheel radius in metres (measure your actual wheel)
// Debounce: KY-024 DO can glitch when the magnet edge exits the field.
// 5 ms is safe up to ~30 rev/s (> 200 km/h on a 0.3 m wheel).
#define HALL_DEBOUNCE_US  5000UL

volatile int           hallPulses   = 0;
volatile unsigned long lastPulseUs  = 0;
unsigned long          lastSpeedMs  = 0;
float                  speedKmh     = 0.0f;

// --- SIM808 GPS ---
HardwareSerial sim808(1);
float gpsLat = 0.0f;
float gpsLon = 0.0f;

// --- Crash flags ---
bool crashSmsSent        = false;
bool crashPublishPending = false;

// ================================================================
//  INTERRUPT
// ================================================================
void IRAM_ATTR onHallPulse() {
  unsigned long now = micros();
  if (now - lastPulseUs >= HALL_DEBOUNCE_US) {
    hallPulses++;
    lastPulseUs = now;
  }
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== NLPC EV Safety — Booting ===");

  // HX710B
  pinMode(HX_DT,  INPUT);
  pinMode(HX_SCK, OUTPUT);

  // KY-024 Hall sensor — DO is active-LOW (drops when magnet present)
  // INPUT_PULLUP adds ESP32's internal pull-up on top of KY-024's onboard resistor
  // for extra noise immunity, especially on long wire runs.
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), onHallPulse, FALLING);
  lastSpeedMs = millis();

  // HX710B zero calibration (no pressure applied during boot)
  Serial.println("[HX710B] Zero calibrating — keep sensor unloaded...");
  delay(2000);
  long sum = 0;
  for (int i = 0; i < 20; i++) { sum += hxRead(); delay(100); }
  hxOffset = sum / 20;
  Serial.print("[HX710B] Offset = "); Serial.println(hxOffset);

  // SIM808 init
  sim808.begin(9600, SERIAL_8N1, 16, 17);
  delay(2000);
  simCmd("AT",                "OK", 2000);
  simCmd("ATE0",              "OK", 1000);
  simCmd("AT+CMGF=1",         "OK", 1000);
  simCmd("AT+CNMI=1,2,0,0,0", "OK", 1000);
  simCmd("AT+CGNSPWR=1",      "OK", 1000);
  simCmd("AT+CGNSSEQ=RMC",    "OK", 1000);
  Serial.println("[SIM808] Ready");

  // I2C sensors
  Wire.begin(21, 22);
  ina219.begin();
  accel.begin();
  accel.setRange(ADXL345_RANGE_16_G);
  Serial.println("[I2C] INA219 + ADXL345 ready");

  // WiFi
  connectWiFi();

  // Blynk — config() reuses the WiFi we already connected
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(5000);   // 5 s timeout — continues even if Blynk is unreachable
  Serial.print("[Blynk] Connected: ");
  Serial.println(Blynk.connected() ? "YES" : "NO (offline mode)");

  // MQTT
  mqttClient.setServer(MQTT_BROKER_IP, MQTT_PORT);
  mqttClient.setBufferSize(512);
  snprintf(mqttTopic, sizeof(mqttTopic), "ev/%s/telemetry", VEHICLE_VIN);
  Serial.print("[MQTT] Topic: "); Serial.println(mqttTopic);

  Serial.println("=== Boot complete — publishing to BOTH Blynk + backend ===\n");
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  // --- Blynk keepalive (run() handles reconnect internally) ---
  Blynk.run();

  // --- MQTT keepalive ---
  if (!mqttClient.connected()) reconnectMQTT();
  mqttClient.loop();

  // --- Incoming SMS (location on demand) ---
  handleIncomingSMS();

  // --- Speed (every 1 s from hall pulses) ---
  updateSpeed();

  // --- GPS poll (every 5 s — slow sensor, no need every cycle) ---
  static unsigned long lastGpsPoll = 0;
  if (millis() - lastGpsPoll >= 5000) {
    lastGpsPoll = millis();
    tryGPSUpdate();
  }

  // --- Immediate crash publish (don't wait for 500 ms timer) ---
  if (crashPublishPending) {
    crashPublishPending = false;
    publishTelemetry();
  }

  // --- Normal 500 ms publish ---
  static unsigned long lastPublish = 0;
  if (millis() - lastPublish >= PUBLISH_INTERVAL_MS) {
    lastPublish = millis();
    publishTelemetry();
  }
}

// ================================================================
//  PUBLISH — reads sensors, sends to Blynk AND MQTT backend
// ================================================================
void publishTelemetry() {
  // --- Read all sensors ---
  float voltage   = ina219.getBusVoltage_V();
  float currentMa = ina219.getCurrent_mA();

  float adcV  = (analogRead(LM35_PIN) / 4095.0f) * 3.3f;
  float tempC = adcV * 100.0f;

  long  hxRaw      = hxRead();
  float pressurePSI = (float)(hxRaw - hxOffset) / PRESSURE_SCALE;
  if (pressurePSI < 0) pressurePSI = 0;
  float pressureBar = pressurePSI * 0.068948f;

  sensors_event_t evt;
  accel.getEvent(&evt);
  float ax = evt.acceleration.x;
  float ay = evt.acceleration.y;
  float az = evt.acceleration.z;
  float gf = sqrt(ax*ax + ay*ay + az*az);

  // ── 1. BLYNK publish ──────────────────────────────────────────
  if (Blynk.connected()) {
    Blynk.virtualWrite(VPIN_VOLTAGE,   round2(voltage));
    Blynk.virtualWrite(VPIN_CURRENT,   round2(currentMa));
    Blynk.virtualWrite(VPIN_TEMP,      round2(tempC));
    Blynk.virtualWrite(VPIN_PRESSURE,  round2(pressureBar));
    Blynk.virtualWrite(VPIN_SPEED,     round2(speedKmh));

    String accelStr = "X:" + String(ax,2) +
                      " Y:" + String(ay,2) +
                      " Z:" + String(az,2) +
                      " G:" + String(gf,2);
    Blynk.virtualWrite(VPIN_ACCEL, accelStr);

    if (gf > CRASH_G_THRESHOLD) {
      Blynk.virtualWrite(VPIN_CRASH_MSG, "CRASH DETECTED");
      String link = "https://maps.google.com/?q=" +
                    String(gpsLat, 6) + "," + String(gpsLon, 6);
      Blynk.virtualWrite(VPIN_GPS_LINK, link);
      Blynk.logEvent("impact_alert", "High G-force: " + String(gf, 2));
    } else {
      Blynk.virtualWrite(VPIN_CRASH_MSG, "Normal");
      Blynk.virtualWrite(VPIN_GPS_LINK, "-");
    }
  }

  // ── 2. MQTT publish → backend → PostgreSQL → your website ─────
  StaticJsonDocument<512> doc;
  doc["vehicleId"]       = VEHICLE_VIN;
  doc["batteryVoltage"]  = round2(voltage);
  doc["currentMa"]       = round2(currentMa);
  doc["temperatureC"]    = round2(tempC);
  doc["tirePressureBar"] = round2(pressureBar);
  doc["speedKmh"]        = round2(speedKmh);
  doc["accelX"]          = round2(ax);
  doc["accelY"]          = round2(ay);
  doc["accelZ"]          = round2(az);
  doc["gForce"]          = round2(gf);
  doc["latitude"]        = gpsLat;
  doc["longitude"]       = gpsLon;

  char payload[512];
  serializeJson(doc, payload);
  bool mqttOk = mqttClient.publish(mqttTopic, payload);

  Serial.printf("[DUAL] Blynk:%s MQTT:%s | gF=%.2f spd=%.1f tmp=%.1f pres=%.2fbar\n",
                Blynk.connected() ? "OK" : "--",
                mqttOk            ? "OK" : "--",
                gf, speedKmh, tempC, pressureBar);

  // ── 3. Crash alert ─────────────────────────────────────────────
  if (gf > CRASH_G_THRESHOLD && !crashSmsSent) {
    crashSmsSent        = true;
    crashPublishPending = true;
    Serial.println("[CRASH] Threshold exceeded — SMS + immediate re-publish");
    String link = "https://maps.google.com/?q=" +
                  String(gpsLat, 6) + "," + String(gpsLon, 6);
    sendSMS(ALERT_PHONE, "CRASH DETECTED! Vehicle: " VEHICLE_VIN " Location: " + link);
  }
  if (gf <= CRASH_G_THRESHOLD) crashSmsSent = false;
}

// ================================================================
//  SPEED
// ================================================================
void updateSpeed() {
  if (millis() - lastSpeedMs >= 1000) {
    noInterrupts();
    int pulses = hallPulses;
    hallPulses  = 0;
    interrupts();

    if (pulses == 0) {
      speedKmh = 0.0f;
    } else {
      // revolutions per second = pulses / magnets_per_rev (1 s window)
      float rps = (float)pulses / MAGNETS_PER_REV;
      speedKmh  = rps * (2.0f * 3.14159f * WHEEL_RADIUS_M) * 3.6f;
    }
    lastSpeedMs = millis();
  }
}

// ================================================================
//  HX710B RAW READ
// ================================================================
long hxRead() {
  while (digitalRead(HX_DT));
  long count = 0;
  for (int i = 0; i < 24; i++) {
    digitalWrite(HX_SCK, HIGH);
    count = count << 1;
    digitalWrite(HX_SCK, LOW);
    if (digitalRead(HX_DT)) count++;
  }
  digitalWrite(HX_SCK, HIGH);
  digitalWrite(HX_SCK, LOW);
  if (count & 0x800000) count |= ~0xFFFFFF;
  return count;
}

// ================================================================
//  GPS
// ================================================================
void tryGPSUpdate() {
  String resp = simCmd("AT+CGNSINF", "OK", 1500);
  int idx = resp.indexOf("+CGNSINF:");
  if (idx == -1) return;
  String s = resp.substring(idx);
  int c[10], n = 0;
  for (size_t i = 0; i < s.length() && n < 10; i++) if (s[i] == ',') c[n++] = i;
  if (n < 6) return;
  if (s.substring(c[0]+1, c[1]).toInt() != 1) return;
  float lat = s.substring(c[2]+1, c[3]).toFloat();
  float lon = s.substring(c[3]+1, c[4]).toFloat();
  if (lat != 0.0f && lon != 0.0f) { gpsLat = lat; gpsLon = lon; }
}

// ================================================================
//  SMS
// ================================================================
void handleIncomingSMS() {
  while (sim808.available()) {
    String r = sim808.readString();
    if (r.indexOf("+CMT:") != -1) {
      int s = r.indexOf("\"") + 1;
      int e = r.indexOf("\"", s);
      String sender = r.substring(s, e);
      tryGPSUpdate();
      String link = "https://maps.google.com/?q=" +
                    String(gpsLat, 6) + "," + String(gpsLon, 6);
      sendSMS(sender, link);
    }
  }
}

// ================================================================
//  WIFI
// ================================================================
void connectWiFi() {
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println();
  Serial.print("[WiFi] Connected — IP: "); Serial.println(WiFi.localIP());
}

// ================================================================
//  MQTT RECONNECT
// ================================================================
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("[MQTT] Reconnecting...");
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      Serial.println(" OK");
    } else {
      Serial.printf(" failed rc=%d — retry in 3s\n", mqttClient.state());
      delay(3000);
    }
  }
}

// ================================================================
//  SIM808 HELPERS
// ================================================================
String simCmd(String cmd, String expected, unsigned long timeoutMs) {
  sim808.println(cmd);
  String resp = "";
  unsigned long t = millis();
  while (millis() - t < timeoutMs) {
    while (sim808.available()) resp += (char)sim808.read();
    if (resp.indexOf(expected) != -1) break;
  }
  return resp;
}

bool sendSMS(String number, String msg) {
  sim808.print("AT+CMGS=\""); sim808.print(number); sim808.println("\"");
  delay(1000);
  sim808.print(msg);
  sim808.write(26);
  delay(3000);
  return true;
}

// ================================================================
//  UTILITY
// ================================================================
float round2(float v) { return roundf(v * 100.0f) / 100.0f; }