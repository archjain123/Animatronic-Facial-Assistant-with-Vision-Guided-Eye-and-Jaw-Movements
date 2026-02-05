/*
 * ============================================================================
 * ANIMATRONIC EYE CONTROLLER WITH JAW - ESP32 + PCA9685
 * ============================================================================
 * - Eye tracking with ESP-NOW
 * - Dynamic eyelid system
 * - Automatic blinking
 * - Real-time jaw control via Serial (USB)
 * - JAW: Channel 11, CLOSED=80°, OPEN=150°
 * ============================================================================
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <esp_now.h>
#include <WiFi.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

// ============================
// ESP-NOW DATA STRUCTURE
// ============================
typedef struct struct_message {
  int16_t face_x;
  int16_t face_y;
  uint8_t face_detected;
  int16_t face_width;
  int16_t face_height;
  uint32_t timestamp;
} struct_message;

struct_message incomingData;
bool newDataReceived = false;
unsigned long lastDataTime = 0;

// ============================
// SERVO CONFIGURATION
// ============================
struct ServoInfo {
  const char* name;
  int channel;
  int minAngle;
  int maxAngle;
  int center;
};

ServoInfo servos[] = {
  {"Eye Pan",            0, 40, 140, 90},
  {"Eye Tilt",           1, 45, 90, 70},
  {"Upper Eyelid Right", 2, 40, 90, 40},
  {"Lower Eyelid Right", 3, 90, 150, 150},
  {"Upper Eyelid Left",  4, 90, 140, 140},
  {"Lower Eyelid Left",  5, 30, 90, 30},
  {"Jaw",               11, 80, 150, 80}  // CLOSED=80, OPEN=150, REST=80
};

const int SERVO_COUNT = sizeof(servos) / sizeof(servos[0]);

// ============================
// JAW CONTROL VARIABLES
// ============================
bool jawTalking = false;
unsigned long lastJawMove = 0;
bool jawOpening = true;
int jawAngle = 100;               // Start closed
const int JAW_SPEED = 25;        // FASTER - milliseconds between movements
const int JAW_STEP = 5;          // SMALLER STEPS for smoothness
const int JAW_MIN = 90;          // CLOSED position
const int JAW_MAX = 120;         // PARTIAL OPEN (natural talking, not yelling!)

// ============================
// SERVO CONTROL
// ============================
void setServoAngle(uint8_t channel, float angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, 102, 512);
  pca.setPWM(channel, 0, pulse);
}

void detachServo(uint8_t channel) {
  pca.setPWM(channel, 0, 0);
}

void detachAllServos() {
  for (int i = 0; i < SERVO_COUNT - 1; i++) {  // Don't detach jaw during talking
    detachServo(servos[i].channel);
  }
}

// ============================
// WATCHDOG TIMER
// ============================
unsigned long lastMoveTime = 0;
const unsigned long MOVE_TIMEOUT = 3000;
bool moveInProgress = false;

void startWatchdog() {
  lastMoveTime = millis();
  moveInProgress = true;
}

void stopWatchdog() {
  moveInProgress = false;
}

bool checkWatchdog() {
  if (moveInProgress && (millis() - lastMoveTime > MOVE_TIMEOUT)) {
    detachAllServos();
    moveInProgress = false;
    return false;
  }
  return true;
}



// ============================
// JAW ANIMATION - FAST OPEN/CLOSE
// ============================
void updateJaw() {
  if (!jawTalking) {
    // Move to rest position (80) when not talking
    setServoAngle(11, 80);
    jawAngle = 90;  // Reset for next talk cycle
    jawOpening = true;
    return;
  }
  
  unsigned long now = millis();
  if (now - lastJawMove < JAW_SPEED) return;
  lastJawMove = now;
  
  // Fast jaw movement between 100-130
  if (jawOpening) {
    jawAngle += JAW_STEP;
    if (jawAngle >= JAW_MAX) {
      jawAngle = JAW_MAX;
      jawOpening = false;
    }
  } else {
    jawAngle -= JAW_STEP;
    if (jawAngle <= JAW_MIN) {
      jawAngle = JAW_MIN;
      jawOpening = true;
    }
  }
  
  setServoAngle(11, jawAngle);
}
// ============================
// SERIAL COMMAND HANDLER
// ============================
void processSerialCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "JAW_START") {
      jawTalking = true;
      Serial.println("🗣️ Jaw talking started");
    } 
    else if (command == "JAW_STOP") {
      jawTalking = false;
      jawAngle = 80;  // Reset to closed
      setServoAngle(11, 80);
      Serial.println("🤐 Jaw talking stopped");
    }
  }
}

// ============================
// SMOOTH MOVEMENT
// ============================
bool smoothMoveMulti(int channels[], float fromDeg[], float toDeg[], int count, int steps, int delayMs) {
  startWatchdog();

  float step[count];
  float angles[count];

  for (int i = 0; i < count; i++) {
    step[i] = (toDeg[i] - fromDeg[i]) / float(steps);
    angles[i] = fromDeg[i];
  }

  for (int s = 0; s <= steps; s++) {
    if (!checkWatchdog()) return false;

    for (int i = 0; i < count; i++) {
      setServoAngle(channels[i], angles[i]);
      angles[i] += step[i];
    }
    delay(delayMs);
  }

  stopWatchdog();
  return true;
}

// ============================
// CURRENT POSITIONS
// ============================
float currentEyePan = 90;
float currentEyeTilt = 70;
float currentUER = 40;
float currentLER = 150;
float currentUEL = 140;
float currentLEL = 30;

// ============================
// EYELID POSITION UPDATE
// ============================
void updateEyelids(float eyePan, float eyeTilt) {
  float closure = 0;

  if (eyePan < 90) closure = map(eyePan, 90, 40, 0, 12);
  else if (eyePan > 90) closure = map(eyePan, 90, 140, 0, 12);

  bool tiltDown = (eyeTilt < 70);

  if (tiltDown) {
    currentUER = 60;
    currentUEL = 120;
    currentLER = 150 - closure;
    currentLEL = 30 + closure;
  } else {
    currentUER = 40 + closure;
    currentLER = 150 - closure;
    currentUEL = 140 - closure;
    currentLEL = 30 + closure;
  }

  setServoAngle(2, currentUER);
  setServoAngle(3, currentLER);
  setServoAngle(4, currentUEL);
  setServoAngle(5, currentLEL);
}

// ============================
// BLINK SYSTEM
// ============================
bool blinkQuick() {
  int ch[] = {2, 3, 4, 5};

  float openPos[] = {currentUER, currentLER, currentUEL, currentLEL};
  float closePos[] = {90, 90, 90, 90};

  if (!smoothMoveMulti(ch, openPos, closePos, 4, 20, 5)) return false;
  delay(40);

  updateEyelids(currentEyePan, currentEyeTilt);

  float reopen[] = {currentUER, currentLER, currentUEL, currentLEL};
  if (!smoothMoveMulti(ch, closePos, reopen, 4, 15, 5)) return false;

  delay(60);
  detachAllServos();
  return true;
}

unsigned long lastBlinkTime = 0;
unsigned long nextBlinkInterval = 3000;

void checkAndBlink() {
  if (millis() - lastBlinkTime > nextBlinkInterval) {
    lastBlinkTime = millis();
    nextBlinkInterval = random(2000, 5000);
    blinkQuick();
  }
}

// ============================
// FACE TRACKING
// ============================
void trackFace() {
  if (!newDataReceived) return;
  newDataReceived = false;

  if (incomingData.face_detected == 1) {
    float targetPan  = map(incomingData.face_x, 0, 240, 140, 40);
    float targetTilt = map(incomingData.face_y, 0, 240, 90, 45);

    targetPan = constrain(targetPan, 40, 140);
    targetTilt = constrain(targetTilt, 45, 90);

    int ch[] = {0, 1};
    float from[] = {currentEyePan, currentEyeTilt};
    float to[] = {targetPan, targetTilt};

    if (smoothMoveMulti(ch, from, to, 2, 12, 6)) {
      currentEyePan = targetPan;
      currentEyeTilt = targetTilt;
    }

    updateEyelids(currentEyePan, currentEyeTilt);
    delay(40);
    detachAllServos();
  }
}

// ============================
// CONNECTION TIMEOUT
// ============================
void checkConnectionTimeout() {
  if (millis() - lastDataTime > 3000) {
    int ch[] = {0, 1};
    float from[] = {currentEyePan, currentEyeTilt};
    float to[] = {90, 70};

    smoothMoveMulti(ch, from, to, 2, 20, 10);

    currentEyePan = 90;
    currentEyeTilt = 70;

    updateEyelids(90, 70);
    detachAllServos();
  }
}

// ============================
// OPEN EYES FULLY
// ============================
bool openEyesFully() {
  setServoAngle(2, 40);
  setServoAngle(3, 150);
  setServoAngle(4, 140);
  setServoAngle(5, 30);
  setServoAngle(0, 90);
  setServoAngle(1, 70);
  setServoAngle(11, 80);  // Jaw CLOSED at rest

  delay(400);
  detachAllServos();
  return true;
}

// ============================
// ESP-NOW CALLBACK
// ============================
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  if (len == sizeof(incomingData)) {
    memcpy(&incomingData, data, sizeof(incomingData));
    newDataReceived = true;
    lastDataTime = millis();
  }
}
// ============================
// ESP-NOW INIT
// ============================
bool initESPNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(OnDataRecv);
  return true;
}

// ============================
// SETUP
// ============================
void setup() {
  Serial.begin(115200);

  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);
  delay(100);

  initESPNow();
  openEyesFully();

  lastBlinkTime = millis();
  lastDataTime = millis();
  
  Serial.println("🤖 Skully ready - waiting for voice commands");
}

// ============================
// MAIN LOOP
// ============================
void loop() {
  processSerialCommand();  // Check for jaw commands
  updateJaw();             // Update jaw position
  checkAndBlink();
  trackFace();
  checkConnectionTimeout();
  delay(10);  // Reduced for smoother jaw movement
}