// ESP32 UDP receiver + PCA9685 pan/tilt servo controller
// Channels: 0 = Eye Pan (L/R), 1 = Eye Tilt (U/D)
// Limits: PAN_MIN=40, PAN_MAX=140 ; TILT_MIN=45, TILT_MAX=90

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const char* ssid = "........";
const char* password = ".......";

const unsigned int UDP_PORT = 4210;
WiFiUDP Udp;

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

// PCA9685 pulse mapping
const int PCA_MIN = 102; // tune if necessary
const int PCA_MAX = 512;

int PAN_CHANNEL = 0;
int TILT_CHANNEL = 1;

// Mechanical limits (from your table)
const float PAN_MIN = 40.0;
const float PAN_MAX = 140.0;
const float TILT_MIN = 45.0;
const float TILT_MAX = 90.0;

// Current and target angles
volatile float currentPan = 90.0;
volatile float currentTilt = 70.0;
volatile float targetPan = 90.0;
volatile float targetTilt = 70.0;

// Smoothing parameters
const unsigned long SMOOTH_DT_MS = 20; // update interval
const float MAX_DEG_PER_SEC = 120.0;   // max speed degrees/second
unsigned long lastSmoothMillis = 0;

void setServoAngle(uint8_t channel, float angle) {
  angle = constrain(angle, 0.0, 180.0);
  int pulse = map((int)angle, 0, 180, PCA_MIN, PCA_MAX);
  pca.setPWM(channel, 0, pulse);
}

void applyServoAngles(float pan, float tilt) {
  setServoAngle(PAN_CHANNEL, pan);
  setServoAngle(TILT_CHANNEL, tilt);
}

// Non-blocking smooth step toward target
void smoothStep() {
  unsigned long now = millis();
  unsigned long dt = now - lastSmoothMillis;
  if (dt < SMOOTH_DT_MS) return;
  lastSmoothMillis = now;
  float seconds = dt / 1000.0;

  // compute allowed change this step
  float maxDelta = MAX_DEG_PER_SEC * seconds;

  float deltaPan = targetPan - currentPan;
  if (fabs(deltaPan) <= maxDelta) currentPan = targetPan;
  else currentPan += (deltaPan > 0 ? 1 : -1) * maxDelta;

  float deltaTilt = targetTilt - currentTilt;
  if (fabs(deltaTilt) <= maxDelta) currentTilt = targetTilt;
  else currentTilt += (deltaTilt > 0 ? 1 : -1) * maxDelta;

  applyServoAngles(currentPan, currentTilt);
}

void centerServosAtStartup() {
  // Start eyes open per previous discussion? we center at provided default:
  currentPan = targetPan = 90.0;
  currentTilt = targetTilt = 70.0;
  applyServoAngles(currentPan, currentTilt);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);

  // Connect to WiFi
  Serial.printf("Connecting to %s\n", ssid);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
    if (millis() - start > 15000) break;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed - continuing (you can still send UDP if AP set)");
  }

  Udp.begin(UDP_PORT);
  Serial.printf("UDP listening on port %u\n", UDP_PORT);

  centerServosAtStartup();
  lastSmoothMillis = millis();
}

void loop() {
  // handle UDP packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    char packet[128];
    int len = Udp.read(packet, sizeof(packet)-1);
    if (len > 0) {
      packet[len] = 0;
      // Expected packet format: "PAN, TILT" (e.g. "85.2,71.3")
      // Or JSON-like "p:85.2,t:71.3"
      float p = NAN, t = NAN;
      // simple CSV parse
      char* token = strtok(packet, ", \r\n");
      if (token) {
        p = atof(token);
        token = strtok(NULL, ", \r\n");
        if (token) t = atof(token);
      }
      if (!isnan(p) && !isnan(t)) {
        // constrain to mechanical limits
        if (p < PAN_MIN) p = PAN_MIN;
        if (p > PAN_MAX) p = PAN_MAX;
        if (t < TILT_MIN) t = TILT_MIN;
        if (t > TILT_MAX) t = TILT_MAX;

        // set targets
        targetPan = p;
        targetTilt = t;
        // optional: debug print
        //Serial.printf("Received PAN=%.2f TILT=%.2f\n", targetPan, targetTilt);
      }
    }
  }

  // smooth movement toward target
  smoothStep();
}
