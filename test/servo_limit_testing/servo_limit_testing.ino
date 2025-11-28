#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// =========================
//  WiFi CONFIG
// =========================
const char* ssid = "moto";
const char* password = "architjain";

WebServer server(80);
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// Servo pulse limits
#define SERVO_MIN 110
#define SERVO_MAX 520

// ========================================
// SERVO LIST (10 total)
// Edit these if you change channels later
// ========================================
struct ServoUnit {
  const char* name;
  int channel;
  int minDeg;
  int maxDeg;
  int currentDeg;
};

ServoUnit servos[] = {
  {"Eye Pan (L/R)", 0, 0, 180, 90},
  {"Eye Tilt (Up/Down)", 1, 0, 180, 90},
  {"Upper Eyelid Right", 2, 0, 180, 90},
  {"Lower Eyelid Right", 3, 0, 180, 90},
  {"Upper Eyelid Left", 4, 0, 180, 90},
  {"Lower Eyelid Left", 5, 0, 180, 90},

  {"Jaw", 6, 0, 180, 90},

  {"Neck Tilt Left", 7, 0, 180, 90},
  {"Neck Tilt Right", 8, 0, 180, 90},
  {"Neck Rotate", 9, 0, 180, 90}
};

int servoCount = sizeof(servos) / sizeof(servos[0]);

// ========================================
//  SET PCA9685 SERVO PULSE
// ========================================
void setServoAngle(int channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pca.setPWM(channel, 0, pulse);
}

// ========================================
//  WEBPAGE
// ========================================
String generatePage() {
  String html = "<html><head><title>Servo Controller</title></head><body>";
  html += "<h1>ESP32 Animatronic Servo Control</h1>";
  html += "<hr>";

  for (int i = 0; i < servoCount; i++) {
    html += "<form action=\"/set\" method=\"GET\">";
    html += "<b>" + String(servos[i].name) + "</b><br>";
    html += "Channel: " + String(servos[i].channel) + "<br>";
    html += "Angle (0–180): ";
    html += "<input type=\"number\" name=\"angle\" min=\"0\" max=\"180\" value=\"" + String(servos[i].currentDeg) + "\">";
    html += "<input type=\"hidden\" name=\"id\" value=\"" + String(i) + "\">";
    html += "<input type=\"submit\" value=\"SET\">";
    html += "</form><br>";
  }

  html += "</body></html>";
  return html;
}

// ========================================
//  ROUTE HANDLERS
// ========================================
void handleRoot() {
  server.send(200, "text/html", generatePage());
}

void handleSet() {
  if (!server.hasArg("id") || !server.hasArg("angle")) {
    server.send(400, "text/plain", "Invalid request");
    return;
  }

  int id = server.arg("id").toInt();
  int angle = server.arg("angle").toInt();

  if (id < 0 || id >= servoCount) {
    server.send(400, "text/plain", "Servo ID out of range");
    return;
  }

  // Apply angle
  servos[id].currentDeg = angle;
  setServoAngle(servos[id].channel, angle);

  server.sendHeader("Location", "/");
  server.send(303);
}

// ========================================
//  SETUP
// ========================================
void setup() {
  Serial.begin(115200);

  // Start WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected.");
  Serial.print("Web Server: http://");
  Serial.println(WiFi.localIP());

  // PCA9685
  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);
  delay(500);

  // Routes
  server.on("/", handleRoot);
  server.on("/set", handleSet);

  server.begin();
  // ========================================
//  CENTER ALL SERVOS AT 90° ON STARTUP
// ========================================
for (int i = 0; i < servoCount; i++) {
  servos[i].currentDeg = 90;                   // update memory
  setServoAngle(servos[i].channel, 90);        // send pulse
  delay(50);                                   // small delay per channel
}

}

// ========================================
//  LOOP
// ========================================
void loop() {
  server.handleClient();
}
