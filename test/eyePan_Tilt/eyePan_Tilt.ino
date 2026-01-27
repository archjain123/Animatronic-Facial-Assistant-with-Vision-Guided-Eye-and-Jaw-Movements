#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

// -----------------------------
// SERVO SETUP TABLE (WITH LIMITS)
// -----------------------------
struct ServoInfo {
  const char* name;
  int channel;
  int minAngle;
  int maxAngle;
  int center;
};

ServoInfo servos[] = {
  {"Eye Pan (L/R)",        0, 40, 140, 90},   // Left=40, Right=140
  {"Eye Tilt (Up/Down)",   1, 45, 90, 67},    // Down=45, Up=90
  {"Upper Eyelid Right",   2, 60, 90, 30},    // Open=30, Closed=90
  {"Lower Eyelid Right",   3, 90, 150, 150},  // Closed=90, Open=150
  {"Upper Eyelid Left",    4, 90, 120, 150},  // Closed=90, Open=150
  {"Lower Eyelid Left",    5, 30, 90, 30}     // Open=30, Closed=90
};

const int SERVO_COUNT = sizeof(servos) / sizeof(servos[0]);

// -----------------------------
// WATCHDOG TIMER SETTINGS
// -----------------------------
unsigned long lastMoveTime = 0;
const unsigned long MOVE_TIMEOUT = 3000;
bool moveInProgress = false;

// -----------------------------
// SERVO PWM FUNCTION
// -----------------------------
void setServoAngle(uint8_t channel, float angle) {
  // Apply limits based on servo channel
  for(int i = 0; i < SERVO_COUNT; i++) {
    if(servos[i].channel == channel) {
      angle = constrain(angle, servos[i].minAngle, servos[i].maxAngle);
      break;
    }
  }
  
  int pulse = map(angle, 0, 180, 102, 512);
  pca.setPWM(channel, 0, pulse);
}

// -----------------------------
// DETACH SERVO (Turn off PWM)
// -----------------------------
void detachServo(uint8_t channel) {
  pca.setPWM(channel, 0, 0);
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
// WATCHDOG FUNCTIONS
// -----------------------------
void startWatchdog() {
  lastMoveTime = millis();
  moveInProgress = true;
}

void stopWatchdog() {
  moveInProgress = false;
}

bool checkWatchdog() {
  if(moveInProgress && (millis() - lastMoveTime > MOVE_TIMEOUT)) {
    Serial.println("⚠️ WATCHDOG TIMEOUT!");
    detachAllServos();
    moveInProgress = false;
    return false;
  }
  return true;
}

// -----------------------------
// MULTI-SERVO SYNCHRONIZED MOVE
// -----------------------------
bool smoothMoveMulti(int channels[], float fromDeg[], float toDeg[], int count, int steps, int delayMs) {
  startWatchdog();
  
  float step[count];
  float angles[count];

  for (int i = 0; i < count; i++) {
    step[i] = (toDeg[i] - fromDeg[i]) / float(steps);
    angles[i] = fromDeg[i];
  }

  for (int s = 0; s <= steps; s++) {
    if(!checkWatchdog()) {
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

// -----------------------------
// NATURAL QUICK BLINK
// -----------------------------
void blinkQuick() {
  int ch[] = {2, 3, 4, 5};  // UER, LER, UEL, LEL

  // QUICK CLOSING (blink)
  float fromOpen[] = {60, 150, 120, 30};  // Eyes open
  float toClose[]  = {90,  90,  90, 90};  // Eyes closed

  if(!smoothMoveMulti(ch, fromOpen, toClose, 4, 20, 5)) {
    Serial.println("❌ Blink close failed");
    return;
  }
  
  delay(40);

  // SMOOTH RE-OPEN
  float fromClose[] = {90, 90, 90, 90};
  float toOpen[]    = {60, 150, 120, 30};

  if(!smoothMoveMulti(ch, fromClose, toOpen, 4, 15, 5)) {
    Serial.println("❌ Blink open failed");
  }
  
  delay(100);
  detachAllServos();
}

// -----------------------------
// EYE MOVEMENT - LOOK AROUND
// -----------------------------
void lookAround() {
  int ch[] = {0, 1};  // Pan, Tilt
  
  // Random eye position within safe limits
  float currentPan = 90;
  float currentTilt = 67;
  
  float targetPan = random(50, 130);   // Within 40-140 range
  float targetTilt = random(55, 85);   // Within 45-90 range
  
  float from[] = {currentPan, currentTilt};
  float to[] = {targetPan, targetTilt};
  
  if(smoothMoveMulti(ch, from, to, 2, 30, 15)) {
    Serial.print("👀 Looking at Pan:");
    Serial.print(targetPan);
    Serial.print("° Tilt:");
    Serial.print(targetTilt);
    Serial.println("°");
  }
  
  delay(100);
  detachAllServos();
}

// -----------------------------
// EYE MOVEMENT - LOOK AT POSITION
// -----------------------------
void lookAt(float panAngle, float tiltAngle) {
  int ch[] = {0, 1};
  
  // Constrain to limits
  panAngle = constrain(panAngle, 40, 140);
  tiltAngle = constrain(tiltAngle, 45, 90);
  
  // Get current position (assume center if unknown)
  static float currentPan = 90;
  static float currentTilt = 67;
  
  float from[] = {currentPan, currentTilt};
  float to[] = {panAngle, tiltAngle};
  
  if(smoothMoveMulti(ch, from, to, 2, 20, 10)) {
    currentPan = panAngle;
    currentTilt = tiltAngle;
  }
  
  delay(100);
  detachAllServos();
}

// -----------------------------
// EYE MOVEMENT - LOOK CENTER
// -----------------------------
void lookCenter() {
  lookAt(90, 67);  // Center position
  Serial.println("👁️ Eyes centered");
}

// -----------------------------
// EYE MOVEMENT - LOOK LEFT
// -----------------------------
void lookLeft() {
  lookAt(50, 67);  // Look left
  Serial.println("👁️ Looking left");
}

// -----------------------------
// EYE MOVEMENT - LOOK RIGHT
// -----------------------------
void lookRight() {
  lookAt(130, 67);  // Look right
  Serial.println("👁️ Looking right");
}

// -----------------------------
// EYE MOVEMENT - LOOK UP
// -----------------------------
void lookUp() {
  lookAt(90, 85);  // Look up
  Serial.println("👁️ Looking up");
}

// -----------------------------
// EYE MOVEMENT - LOOK DOWN
// -----------------------------
void lookDown() {
  lookAt(90, 50);  // Look down
  Serial.println("👁️ Looking down");
}

// -----------------------------
// START WITH EYES FULLY OPEN & CENTERED
// -----------------------------
bool openEyesFully() {
  startWatchdog();
  
  // Open eyelids
  setServoAngle(2, 60);     // UER open
  setServoAngle(3, 150);    // LER open
  setServoAngle(4, 120);    // UEL open
  setServoAngle(5, 30);     // LEL open
  
  // Center eyes
  setServoAngle(0, 90);     // Pan center
  setServoAngle(1, 67);     // Tilt center
  
  delay(200);
  
  if(!checkWatchdog()) {
    Serial.println("❌ Failed to open eyes");
    return false;
  }
  
  stopWatchdog();
  detachAllServos();
  Serial.println("✓ Eyes opened and centered");
  return true;
}

// -----------------------------
// SETUP
// -----------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Eye Control with Pan/Tilt ===");
  
  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);

  delay(300);
  openEyesFully();
  delay(500);
  
  Serial.println("Ready");
}

// -----------------------------
// LOOP - RANDOM EYE MOVEMENTS & BLINKS
// -----------------------------
void loop() {
  static unsigned long lastBlink = 0;
  static unsigned long lastLook = 0;
  
  unsigned long now = millis();
  
  // Blink every 4-6 seconds (random)
  if (now - lastBlink > random(4000, 6000)) {
    lastBlink = now;
    blinkQuick();
  }
  
  // Look around every 2-4 seconds (random)
  if (now - lastLook > random(2000, 4000)) {
    lastLook = now;
    
    // Random behavior
    int action = random(0, 6);
    switch(action) {
      case 0:
        lookCenter();
        break;
      case 1:
        lookLeft();
        break;
      case 2:
        lookRight();
        break;
      case 3:
        lookUp();
        break;
      case 4:
        lookDown();
        break;
    }
  }
  
  delay(50);  // Small delay
}