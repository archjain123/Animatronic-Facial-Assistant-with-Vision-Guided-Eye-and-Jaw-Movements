#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

// -----------------------------
// SERVO SETUP TABLE
// -----------------------------
struct ServoInfo {
  const char* name;
  int channel;
  int minAngle;
  int maxAngle;
  int center;
};

ServoInfo servos[] = {
  {"Eye Pan (L/R)",        0, 0, 180, 90},
  {"Eye Tilt (Up/Down)",   1, 0, 180, 90},
  {"Upper Eyelid Right",   2, 0, 180, 90},  // UER
  {"Lower Eyelid Right",   3, 0, 180, 90},  // LER
  {"Upper Eyelid Left",    4, 0, 180, 90},  // UEL
  {"Lower Eyelid Left",    5, 0, 180, 90}   // LEL
};

const int SERVO_COUNT = sizeof(servos) / sizeof(servos[0]);

// -----------------------------
// WATCHDOG TIMER SETTINGS
// -----------------------------
unsigned long lastMoveTime = 0;
const unsigned long MOVE_TIMEOUT = 3000;  // 3 seconds max per move sequence
bool moveInProgress = false;

// -----------------------------
// SERVO PWM FUNCTION
// -----------------------------
void setServoAngle(uint8_t channel, float angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, 102, 512);  // PCA9685 pulse range
  pca.setPWM(channel, 0, pulse);
}

// -----------------------------
// DETACH SERVO (Turn off PWM)
// -----------------------------
void detachServo(uint8_t channel) {
  pca.setPWM(channel, 0, 0);  // No signal = servo relaxes, no holding torque
}

// -----------------------------
// DETACH ALL SERVOS
// -----------------------------
void detachAllServos() {
  for(int i = 0; i < SERVO_COUNT; i++) {
    detachServo(servos[i].channel);
  }
}

// -----------------------------
// WATCHDOG CHECK
// -----------------------------
bool checkWatchdog() {
  if(moveInProgress && (millis() - lastMoveTime > MOVE_TIMEOUT)) {
    Serial.println("⚠️ WATCHDOG TIMEOUT - Movement taking too long!");
    detachAllServos();
    moveInProgress = false;
    return false;  // Timeout occurred
  }
  return true;  // All good
}

// -----------------------------
// START WATCHDOG
// -----------------------------
void startWatchdog() {
  lastMoveTime = millis();
  moveInProgress = true;
}

// -----------------------------
// STOP WATCHDOG
// -----------------------------
void stopWatchdog() {
  moveInProgress = false;
}

// -----------------------------
// MULTI-SERVO SYNCHRONIZED MOVE
// -----------------------------
bool smoothMoveMulti(int channels[], float fromDeg[], float toDeg[], int count, int steps, int delayMs) {
  startWatchdog();  // Start watchdog timer
  
  float step[count];
  float angles[count];

  for (int i = 0; i < count; i++) {
    step[i] = (toDeg[i] - fromDeg[i]) / float(steps);
    angles[i] = fromDeg[i];
  }

  for (int s = 0; s <= steps; s++) {
    // Check watchdog each iteration
    if(!checkWatchdog()) {
      return false;  // Timeout - abort movement
    }
    
    for (int i = 0; i < count; i++) {
      setServoAngle(channels[i], angles[i]);
      angles[i] += step[i];
    }
    delay(delayMs);
  }
  
  stopWatchdog();  // Movement complete
  return true;  // Success
}

// -----------------------------
// NATURAL QUICK BLINK (SYNCED)
// -----------------------------
bool blinkQuick() {
  int ch[] = {2, 3, 4, 5};  // UER, LER, UEL, LEL

  // QUICK CLOSING (blink)
  float fromOpen[] = {30, 150, 150, 30};
  float toClose[]  = {90,  90,  90, 90};

  if(!smoothMoveMulti(ch, fromOpen, toClose, 4, 20, 5)) {
    Serial.println("❌ Blink close failed - watchdog timeout");
    return false;
  }
  
  delay(40);

  // SMOOTH RE-OPEN
  float fromClose[] = {90, 90, 90, 90};
  float toOpen[]    = {30, 150, 150, 30};

  if(!smoothMoveMulti(ch, fromClose, toOpen, 4, 15, 5)) {
    Serial.println("❌ Blink open failed - watchdog timeout");
    return false;
  }
  
  delay(100);  // Let servos settle
  detachAllServos();  // Turn off holding torque after blink
  
  return true;  // Success
}

// -----------------------------
// START WITH EYES FULLY OPEN
// -----------------------------
bool openEyesFully() {
  startWatchdog();
  
  setServoAngle(2, 30);     // UER open
  setServoAngle(3, 150);    // LER open
  setServoAngle(4, 150);    // UEL open
  setServoAngle(5, 30);     // LEL open
  
  delay(200);  // Wait for servos to reach position
  
  if(!checkWatchdog()) {
    return false;
  }
  
  stopWatchdog();
  detachAllServos();  // Turn off holding torque
  return true;
}

// -----------------------------
// SETUP
// -----------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("MG90S Eye Control - Starting...");
  
  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);

  delay(300);
  
  if(openEyesFully()) {
    Serial.println("✓ Eyes opened successfully");
  } else {
    Serial.println("❌ Failed to open eyes");
  }
  
  delay(500);
  Serial.println("Ready - entering main loop");
}

// -----------------------------
// LOOP
// -----------------------------
void loop() {
  // Eyes remain open (servos detached = no power consumption)
  delay(5000);
  
  // Perform natural blink
  if(blinkQuick()) {
    Serial.println("✓ Blink complete");
  } else {
    Serial.println("❌ Blink failed - waiting before retry");
    delay(2000);  // Wait longer before retry
  }
}

