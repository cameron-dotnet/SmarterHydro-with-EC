/*
  Smart Hydroponics (Mega 2560 + ESP8266 via WiFiEsp)
  ---------------------------------------------------
  FIXES INCLUDED (minimal, targeted):
  1) Light sensor now returns a stable 0–100% brightness value (auto learns min/max).
  2) Slightly increased ADC sample spacing to behave better under indoor LED flicker.
  3) Fixed a compile-breaking extra '}' that was in your pasted code after getLightLevel().
*/

#include <WiFiEspUdp.h>
#include <SPI.h>               // SPI support (WiFiEsp may use it internally depending on setup)
#include <WiFiEsp.h>           // WiFiEsp core (ESP8266 AT firmware driver)
#include <WiFiEspClient.h>     // Client socket class
#include <WiFiEspServer.h>     // Server socket class
#include <DHT.h>               // DHT11 temp/humidity sensor library
#include "DFRobot_PH.h"        // DFRobot Gravity pH library
#include "DFRobot_EC10.h"      // DFRobot Gravity EC library (K=1 type boards)
#include <arduino-timer.h>     // Lightweight scheduling/timers
#include <math.h>              // isnan()

// ================= WIFI NETWORK SETTINGS =================
char ssid[] = "SmartHydro1";
char password[] = "Password123";

// message is used to build JSON responses
String message = "";

// ================= ML MODELS (RandomForest) =================
#include "EC.h"
#include "pH.h"
#include "Humidity.h"
#include "Temperature.h"

Eloquent::ML::Port::RandomForestEC ForestEC;
Eloquent::ML::Port::RandomForestpH ForestPH;
Eloquent::ML::Port::RandomForestHumidity ForestHumidity;
Eloquent::ML::Port::RandomForestTemperature ForestTemperature;

// ================= HTTP SERVER =================
WiFiEspServer server(80);
RingBuffer buf(16);

// ================= PINS (Mega 2560) =================
#define FLOW_PIN 2

#define LIGHT_PIN A7
#define EC_PIN A8
#define PH_PIN A9

#define DHTTYPE DHT11
#define DHT_PIN 8

#define LED_PIN 4
#define FAN_PIN 5
#define PUMP_PIN 6
#define EXTRACTOR_PIN 7

#define PH_UP_PIN 9
#define PH_DOWN_PIN 10
#define EC_UP_PIN 11
#define EC_DOWN_PIN 12

// ================= SENSOR OBJECTS =================
DFRobot_PH ph;
DFRobot_EC10 ec;
DHT dht = DHT(DHT_PIN, DHTTYPE);

// ================= TIMING CONSTANTS =================
const unsigned long SIXTEEN_HR = 57600000UL;
const unsigned long PUMP_INTERVAL = 5000UL;

const unsigned long EIGHT_HR = 28800000UL;
const unsigned long FOUR_HR = 14400000UL;

const unsigned long QUATER_HR = 900000UL;
const unsigned long FIVE_THREEQUATER_HR = 20700000UL;

const unsigned long SENSOR_INTERVAL = 5000UL;
const unsigned long DHT_MIN_INTERVAL = 2000UL;

// arduino-timer instance for scheduled events
auto timer = timer_create_default();

// ================= SENSOR READINGS (GLOBAL STATE) =================
float temperature = NAN;
float humidity = NAN;
float ecLevel = NAN;
float phLevel = NAN;
float lightLevel = NAN;  // NOW: 0–100% after fix
float flowRate = 0.0;

float lastGoodTemp = NAN;
float lastGoodHum = NAN;
float lastGoodEC = NAN;
float lastGoodPH = NAN;
float lastGoodLight = NAN;

// Light auto-calibration (raw ADC -> 0–100%)
int lightMin = 1023;  // darkest seen
int lightMax = 0;     // brightest seen

// Flow sensor uses interrupt pulses
volatile unsigned long pulseCount = 0;
unsigned long currentTime = 0, cloopTime = 0;

unsigned long lastSensorReadMs = 0;
unsigned long lastDhtReadMs = 0;

float lastFlowRate = 0.0;

// ================= FUNCTION DECLARATIONS =================
void togglePin(int pin);
void togglePin(int pin, int toggleValue);

void sendHttpResponse(WiFiEspClient client, String message);

int analogReadAvg(int pin, uint8_t samples);
float getLightLevel();
float getEC();
float getPH();
float getFlowRate();

void setComponent(int result, int pin, int status);
void setPump(int result, int pinUp, int pinDown, int statusUp, int statusDown);

void estimateTemperature();
void estimateHumidity();
void estimatePH();
void estimateEC();
void estimateFactors();

void disablePH();
void disableEC();

void incrementPulseCounter();

void toggleLightOn();
void toggleLightOff();
void togglePumpOn();
void togglePumpOff();

// ----------------------------------------------------------
// Utility: take an averaged ADC reading for smoother signals
// ----------------------------------------------------------
int analogReadAvg(int pin, uint8_t samples = 10) {
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(800); // was 250; helps indoors under LED flicker
  }
  return (int)(sum / samples);
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);

  WiFi.init(&Serial1);

  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi module not detected. Please check wiring and power.");
    while (true) { }
  }

  Serial.print("Attempting to start AP ");
  Serial.println(ssid);

  IPAddress localIp(192, 168, 8, 14);
  WiFi.configAP(localIp);

  WiFi.beginAP(ssid, 11, password, ENC_TYPE_WPA2_PSK);
  Serial.println("Access point started");

  server.begin();

  ec.begin();
  dht.begin();
  ph.begin();

  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), incrementPulseCounter, RISING);

  // NOTE: This toggling can cause random relay states at boot.
  // If you want safe startup, replace toggles with explicit OFF states.
  for (int i = 3; i < 13; i++) {
    if (i != 8) {
      pinMode(i, OUTPUT);
      togglePin(i);
    }
  }

  togglePin(LED_PIN);
  togglePin(FAN_PIN);
  togglePin(PUMP_PIN);
  togglePin(EXTRACTOR_PIN);

  Serial.println("Server started");

  timer.every(5000, estimateTemperature);
  timer.every(5000, estimateHumidity);
  timer.every(SIXTEEN_HR, estimateEC);
  timer.every(SIXTEEN_HR, estimatePH);

  toggleLightOn();
  togglePumpOn();
}

void loop() {
  WiFiEspClient client = server.available();

  unsigned long now = millis();

  if (now - lastSensorReadMs >= SENSOR_INTERVAL) {
    lastSensorReadMs = now;

    if (now - lastDhtReadMs >= DHT_MIN_INTERVAL) {
      lastDhtReadMs = now;

      float t = dht.readTemperature();
      float h = dht.readHumidity();

      if (!isnan(t)) { temperature = t; lastGoodTemp = t; }
      else temperature = lastGoodTemp;

      if (!isnan(h)) { humidity = h; lastGoodHum = h; }
      else humidity = lastGoodHum;
    }

    // Light (0–100%)
    lightLevel = getLightLevel();
    if (!isnan(lightLevel)) lastGoodLight = lightLevel;
    else lightLevel = lastGoodLight;

    ecLevel = getEC();
    if (!isnan(ecLevel)) lastGoodEC = ecLevel;
    else ecLevel = lastGoodEC;

    phLevel = getPH();
    if (!isnan(phLevel)) lastGoodPH = phLevel;
    else phLevel = lastGoodPH;

    flowRate = getFlowRate();
  }

  timer.tick();

  if (client) {
    buf.init();
    message = "";

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        buf.push(c);

        if (buf.endsWith("\r\n\r\n")) {
          message =
            "{\n  \"PH\": \"" + String(phLevel) +
            "\",\n  \"Light\": \"" + String(lightLevel) +
            "\",\n  \"EC\": \"" + String(ecLevel) +
            "\",\n  \"FlowRate\": \"" + String(flowRate) +
            "\",\n  \"Humidity\": \"" + String(humidity) +
            "\",\n  \"Temperature\": \"" + String(temperature) +
            "\"\n }";

          sendHttpResponse(client, message);
          break;
        }

        if (buf.endsWith("/light")) togglePin(LED_PIN);
        if (buf.endsWith("/fan")) togglePin(FAN_PIN);
        if (buf.endsWith("/extract")) togglePin(EXTRACTOR_PIN);
        if (buf.endsWith("/pump")) togglePin(PUMP_PIN);

        if (buf.endsWith("/phUp")) {
          togglePin(PH_DOWN_PIN, LOW);
          togglePin(PH_UP_PIN, HIGH);
          timer.in(PUMP_INTERVAL, disablePH);
        }

        if (buf.endsWith("/phDown")) {
          togglePin(PH_UP_PIN, LOW);
          togglePin(PH_DOWN_PIN, HIGH);
          timer.in(PUMP_INTERVAL, disablePH);
        }

        if (buf.endsWith("/ecUp")) {
          togglePin(EC_DOWN_PIN, LOW);
          togglePin(EC_UP_PIN, HIGH);
          timer.in(PUMP_INTERVAL, disableEC);
        }

        if (buf.endsWith("/ecDown")) {
          togglePin(EC_UP_PIN, LOW);
          togglePin(EC_DOWN_PIN, HIGH);
          timer.in(PUMP_INTERVAL, disableEC);
        }

        if (buf.endsWith("/ph")) disablePH();
        if (buf.endsWith("/ec")) disableEC();

        if (buf.endsWith("/components")) {
          message =
            "{\n  \"PHPump\": \"" + String(digitalRead(PH_UP_PIN)) +
            "\",\n  \"LightRaw\": \"" + String(analogRead(LIGHT_PIN)) +
            "\",\n  \"LightPct\": \"" + String(lightLevel) +
            "\",\n  \"ECPump\": \"" + String(digitalRead(EC_UP_PIN)) +
            "\",\n  \"WaterPump\": \"" + String(digitalRead(PUMP_PIN)) +
            "\",\n  \"Exctractor\": \"" + String(digitalRead(EXTRACTOR_PIN)) +
            "\",\n  \"Fan\": \"" + String(digitalRead(FAN_PIN)) +
            "\"\n }";

          sendHttpResponse(client, message);
          break;
        }
      }
    }

    Serial.println("Client disconnected");
    client.stop();
  }
}

void togglePin(int pin) {
  digitalWrite(pin, !(digitalRead(pin)));
}

void togglePin(int pin, int toggleValue) {
  digitalWrite(pin, toggleValue);
}

void sendHttpResponse(WiFiEspClient client, String message) {
  client.print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: application/json\r\n"
    "Connection: close\r\n"
  );

  if (message.length() > 0) {
    client.print("Content-Length:" + String(message.length()) + "\r\n\r\n");
    client.print(message);
  }
}

/**
 * getLightLevel()
 * --------------
 * NOW: returns 0–100% brightness (auto learns min/max).
 * If it hasn't learned enough range yet, returns raw ADC briefly.
 */
float getLightLevel() {
  int raw = analogReadAvg(LIGHT_PIN, 10);

  // learn min/max
  if (raw < lightMin) lightMin = raw;
  if (raw > lightMax) lightMax = raw;

  // not enough range learned yet
  if (lightMax - lightMin < 10) {
    return (float)raw; // temporary
  }

  float pct = (raw - lightMin) * 100.0 / (float)(lightMax - lightMin);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

float getEC() {
  if (isnan(temperature)) return lastGoodEC;

  int raw = analogReadAvg(EC_PIN, 10);
  float ecVoltage = (float)raw / 1023.0 * 5000.0;

  float val = ec.readEC(ecVoltage, temperature);
  return val;
}

float getPH() {
  if (isnan(temperature)) return lastGoodPH;

  int raw = analogReadAvg(PH_PIN, 10);
  float phVoltage = (float)raw / 1023.0 * 5000.0;

  float val = ph.readPH(phVoltage, temperature);
  return val;
}

void setComponent(int result, int pin, int status) {
  if (result == 0) {
    if (status == 1) digitalWrite(pin, LOW);
  } else if (result == 1) {
    if (status == 0) digitalWrite(pin, HIGH);
  } else {
    if (status == 0) {
      Serial.println(String("Component: ") + digitalRead(pin));
      togglePin(pin);
    }
  }
}

void setPump(int result, int pinUp, int pinDown, int statusUp, int statusDown) {
  if (result == 0) {
    if (statusUp == 1 || statusDown == 0) {
      digitalWrite(pinUp, LOW);
      digitalWrite(pinDown, HIGH);
    }
  } else if (result == 1) {
    if (statusUp == 0 || statusDown == 1) {
      digitalWrite(pinUp, HIGH);
      digitalWrite(pinDown, LOW);
    }
  } else {
    Serial.println(String("Up: ") + digitalRead(pinUp));
    Serial.println(String("Down: ") + digitalRead(pinDown));
    togglePin(pinUp, HIGH);
    togglePin(pinDown, HIGH);
  }
}

void estimateTemperature() {
  if (!isnan(temperature)) {
    int result = ForestTemperature.predict(&temperature);
    int fanStatus = digitalRead(FAN_PIN);

    int lightStatus = digitalRead(LED_PIN);
    (void)lightStatus;

    Serial.println(result);
    setComponent(result, FAN_PIN, fanStatus);
  }
}

void estimateHumidity() {
  if (!isnan(humidity)) {
    int result = ForestHumidity.predict(&humidity);
    int extractorStatus = digitalRead(EXTRACTOR_PIN);
    setComponent(result, EXTRACTOR_PIN, extractorStatus);
  }
}

void estimatePH() {
  if (!isnan(phLevel)) {
    int result = ForestPH.predict(&phLevel);
    int phUpStatus = digitalRead(PH_UP_PIN);
    int phDownStatus = digitalRead(PH_DOWN_PIN);

    setPump(result, PH_UP_PIN, PH_DOWN_PIN, phUpStatus, phDownStatus);
    timer.in(PUMP_INTERVAL, disablePH);
  }
}

void estimateEC() {
  if (!isnan(ecLevel)) {
    int result = ForestEC.predict(&ecLevel);
    int ecUpStatus = digitalRead(EC_UP_PIN);
    int ecDownStatus = digitalRead(EC_DOWN_PIN);

    setPump(result, EC_UP_PIN, EC_DOWN_PIN, ecUpStatus, ecDownStatus);
    timer.in(PUMP_INTERVAL, disableEC);
  }
}

void estimateFactors() {
  estimatePH();
  estimateTemperature();
  estimateHumidity();
  estimateEC();
}

void disablePH() {
  digitalWrite(PH_UP_PIN, HIGH);
  digitalWrite(PH_DOWN_PIN, HIGH);
}

void disableEC() {
  digitalWrite(EC_UP_PIN, HIGH);
  digitalWrite(EC_DOWN_PIN, HIGH);
  Serial.println("EC HIT");
}

void incrementPulseCounter() {
  pulseCount++;
}

float getFlowRate() {
  currentTime = millis();

  if (currentTime >= (cloopTime + 1000)) {
    cloopTime = currentTime;

    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float flowRatePerHr = (pulses * 60.0 / 7.5);
    lastFlowRate = flowRatePerHr;
    return flowRatePerHr;
  }

  return lastFlowRate;
}

void toggleLightOn() {
  togglePin(LED_PIN, LOW);
  timer.in(EIGHT_HR, toggleLightOff);
}

void toggleLightOff() {
  togglePin(LED_PIN, HIGH);
  timer.in(FOUR_HR, toggleLightOn);
}

void togglePumpOn() {
  togglePin(PUMP_PIN, LOW);
  timer.in(QUATER_HR, togglePumpOff);
}

void togglePumpOff() {
  togglePin(PUMP_PIN, HIGH);
  timer.in(FIVE_THREEQUATER_HR, togglePumpOn);
}