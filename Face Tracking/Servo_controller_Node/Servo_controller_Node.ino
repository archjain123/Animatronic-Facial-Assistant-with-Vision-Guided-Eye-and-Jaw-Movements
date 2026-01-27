/*
 * ============================================================================
 * IMPROVED ANIMATRONIC EYE CONTROLLER - ESP32 + PCA9685
 * ============================================================================
 * Features:
 * - Responsive face tracking with smooth movements
 * - Continuous random blinking (2-5 seconds)
 * - Connection timeout handling (returns to center)
 * - Watchdog timer for stall protection
 * - Servo detachment for power/heat management
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
  {"Eye Pan",            0, 40, 140, 90},   // Left=40, Right=140
  {"Eye Tilt",           1, 45, 90, 67},    // Down=45, Up=90
  {"Upper Eyelid Right", 2, 0, 180, 30},    // Open=30, Closed=90
  {"Lower Eyelid Right", 3, 0, 180, 150},   // Open=150, Closed=90
  {"Upper Eyelid Left",  4, 0, 180, 150},   // Open=150, Closed=90
  {"Lower Eyelid Left",  5, 0, 180, 30}     // Open=30, Closed=90
};

const int SERVO_COUNT = sizeof(servos) / sizeof(servos[0]);

// ============================
// WATCHDOG TIMER
// ============================
unsigned long lastMoveTime = 0;
const unsigned long MOVE_TIMEOUT = 3000;  // 3 seconds max
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
    Serial.println("⚠️ WATCHDOG TIMEOUT - Aborting movement!");
    detachAllServos();
    moveInProgress = false;
    return false;
  }
  return true;
}

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
  for (int i = 0; i < SERVO_COUNT; i++) {
    detachServo(servos[i].channel);
  }
}

// ============================
// SMOOTH MULTI-SERVO MOVEMENT
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
    if (!checkWatchdog()) {
      return false;
    }

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
// CURRENT EYE STATE
// ============================
float currentEyePan = 90;
float currentEyeTilt = 67;

float currentUER = 30;   // Upper Eyelid Right
float currentLER = 150;  // Lower Eyelid Right
float currentUEL = 150;  // Upper Eyelid Left
float currentLEL = 30;   // Lower Eyelid Left

// ============================
// BLINKING SYSTEM - EXACT PATTERN
// ============================
unsigned long lastBlinkTime = 0;
unsigned long nextBlinkInterval = 3000;

bool blinkQuick() {
  int ch[] = {2, 3, 4, 5};  // UER, LER, UEL, LEL

  // QUICK CLOSING (blink)
  float fromOpen[] = {30, 150, 150, 30};
  float toClose[]  = {90,  90,  90, 90};

  if (!smoothMoveMulti(ch, fromOpen, toClose, 4, 20, 5)) {
    Serial.println("❌ Blink close failed - watchdog timeout");
    return false;
  }
  
  delay(40);

  // SMOOTH RE-OPEN
  float fromClose[] = {90, 90, 90, 90};
  float toOpen[]    = {30, 150, 150, 30};

  if (!smoothMoveMulti(ch, fromClose, toOpen, 4, 15, 5)) {
    Serial.println("❌ Blink open failed - watchdog timeout");
    return false;
  }
  
  delay(100);  // Let servos settle
  detachAllServos();  // Turn off holding torque after blink
  
  return true;  // Success
}

void checkAndBlink() {
  if (millis() - lastBlinkTime > nextBlinkInterval) {
    lastBlinkTime = millis();
    nextBlinkInterval = random(2000, 5000);  // Random 2-5 seconds
    
    if (blinkQuick()) {
      Serial.println("👁️ Blink complete");
    } else {
      Serial.println("❌ Blink failed - waiting before retry");
    }
  }
}

// ============================
// EYELID ADJUSTMENT FOR EYE POSITION
// ============================
void updateEyelidsForEyePosition(float eyePan) {
  // Slight squint when looking left/right
  float closure = 0;

  if (eyePan < 90)
    closure = map(eyePan, 90, 40, 0, 12);
  else if (eyePan > 90)
    closure = map(eyePan, 90, 140, 0, 12);

  currentUER = 30 + closure;
  currentLER = 150 - closure;
  currentUEL = 150 - closure;
  currentLEL = 30 + closure;

  setServoAngle(2, currentUER);
  setServoAngle(3, currentLER);
  setServoAngle(4, currentUEL);
  setServoAngle(5, currentLEL);
}

// ============================
// FACE TRACKING
// ============================
void trackFace() {
  if (!newDataReceived) return;
  newDataReceived = false;

  if (incomingData.face_detected == 1) {
    // Map face coordinates to servo angles
    // X: 0-240 → Pan: 140-40 (inverted for natural tracking)
    // Y: 0-240 → Tilt: 90-45
    float targetEyePan = map(incomingData.face_x, 0, 240, 140, 40);
    float targetEyeTilt = map(incomingData.face_y, 0, 240, 90, 45);

    // Apply constraints
    targetEyePan = constrain(targetEyePan, 40, 140);
    targetEyeTilt = constrain(targetEyeTilt, 45, 90);

    // Move eyes with reduced steps for faster response
    int ch[] = {0, 1};
    float from[] = {currentEyePan, currentEyeTilt};
    float to[] = {targetEyePan, targetEyeTilt};

    if (smoothMoveMulti(ch, from, to, 2, 12, 6)) {  // Faster: 12 steps, 6ms delay
      currentEyePan = targetEyePan;
      currentEyeTilt = targetEyeTilt;
    }

    // Adjust eyelids based on pan position
    updateEyelidsForEyePosition(currentEyePan);
    
    delay(50);
    detachAllServos();

    Serial.printf("👀 Tracking - Pan:%.0f° Tilt:%.0f°\n", currentEyePan, currentEyeTilt);
  }
}

// ============================
// CONNECTION TIMEOUT HANDLING
// ============================
const unsigned long CONNECTION_TIMEOUT = 3000;  // 3 seconds
bool eyesCentered = false;

void checkConnectionTimeout() {
  if (millis() - lastDataTime > CONNECTION_TIMEOUT) {
    if (!eyesCentered) {
      Serial.println("⚠️ Connection lost - Centering eyes");
      
      // Return to center position
      int ch[] = {0, 1};
      float from[] = {currentEyePan, currentEyeTilt};
      float to[] = {90, 67};  // Center position

      if (smoothMoveMulti(ch, from, to, 2, 20, 10)) {
        currentEyePan = 90;
        currentEyeTilt = 67;
      }

      // Reset eyelids to open position
      currentUER = 30;
      currentLER = 150;
      currentUEL = 150;
      currentLEL = 30;
      updateEyelidsForEyePosition(90);

      delay(100);
      detachAllServos();
      
      eyesCentered = true;
      Serial.println("✓ Eyes centered");
    }
  } else {
    eyesCentered = false;  // Reset flag when receiving data
  }
}

// ============================
// OPEN EYES FULLY
// ============================
bool openEyesFully() {
  startWatchdog();

  setServoAngle(2, 30);     // UER open
  setServoAngle(3, 150);    // LER open
  setServoAngle(4, 150);    // UEL open
  setServoAngle(5, 30);     // LEL open
  setServoAngle(0, 90);     // Pan center
  setServoAngle(1, 67);     // Tilt center

  delay(500);

  if (!checkWatchdog()) {
    Serial.println("❌ Failed to open eyes");
    return false;
  }

  stopWatchdog();
  detachAllServos();
  Serial.println("✓ Eyes opened and centered");
  return true;
}

// ============================
// ESP-NOW CALLBACK
// ============================
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len == sizeof(incomingData)) {
    memcpy(&incomingData, data, sizeof(incomingData));
    newDataReceived = true;
    lastDataTime = millis();
  }
}

// ============================
// ESP-NOW INITIALIZATION
// ============================
bool initESPNow() {
  WiFi.mode(WIFI_STA);

  Serial.print("📌 ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println("⚠️  Copy this MAC into MEMENTO sender code!");

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed");
    return false;
  }

  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("✓ ESP-NOW Initialized");
  return true;
}

// ============================
// SETUP
// ============================
void setup() {
  Serial.begin(115200);
  Serial.println("\n========================================");
  Serial.println("  ANIMATRONIC EYE CONTROLLER v2.0");
  Serial.println("========================================");

  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);
  delay(100);

  if (!initESPNow()) {
    Serial.println("❌ ESP-NOW FAILED - Check configuration");
    while (1) delay(1000);
  }

  delay(300);
  
  if (openEyesFully()) {
    Serial.println("✓ Eyes opened successfully");
  } else {
    Serial.println("❌ Failed to open eyes");
  }

  lastBlinkTime = millis();
  lastDataTime = millis();
  nextBlinkInterval = random(2000, 5000);

  Serial.println("\n✓ System Ready");
  Serial.println("✓ Waiting for face tracking data...\n");
}

// ============================
// MAIN LOOP
// ============================
void loop() {
  checkAndBlink();              // Continuous random blinking
  trackFace();                  // Track face if data received
  checkConnectionTimeout();     // Handle connection loss
  
  delay(50);  // Small delay for stability
}