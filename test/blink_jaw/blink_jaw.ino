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
  {"Lower Eyelid Left",    5, 0, 180, 90},  // LEL
  {"Jaw",                  6, 80, 120, 90}  // JAW
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
  angle = constrain(angle, 0, 180);
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
// NATURAL QUICK BLINK (SYNCED)
// -----------------------------
void blinkQuick() {
  int ch[] = {2, 3, 4, 5};  // UER, LER, UEL, LEL

  // QUICK CLOSING (blink)
  float fromOpen[] = {30, 150, 150, 30};
  float toClose[]  = {90,  90,  90, 90};

  if(!smoothMoveMulti(ch, fromOpen, toClose, 4, 20, 5)) {
    Serial.println("❌ Blink close failed");
    return;
  }
  
  delay(40);

  // SMOOTH RE-OPEN
  float fromClose[] = {90, 90, 90, 90};
  float toOpen[]    = {30,150,150,30};

  if(!smoothMoveMulti(ch, fromClose, toOpen, 4, 15, 5)) {
    Serial.println("❌ Blink open failed");
  }
  
  delay(100);
  detachAllServos();
}

// -----------------------------
// CONTINUOUS JAW TALKING MOTION
// -----------------------------
void jawTalkContinuous() {
  static unsigned long lastMove = 0;
  static bool opening = true;
  static int angle = 90;
  unsigned long now = millis();
  
  // Control speed - adjust this value to change talking speed
  if (now - lastMove < 80) return;  // 80ms = moderate speed
  lastMove = now;
  
  // Animate jaw movement
  if (opening) {
    angle += 8;  // Adjust step size for smoother/faster movement
  } else {
    angle -= 8;
  }
  
  // Reverse direction at limits
  if (angle >= 120) { 
    angle = 120; 
    opening = false; 
  }
  if (angle <= 80) { 
    angle = 80;  
    opening = true;  
  }
  
  setServoAngle(6, angle);
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
  
  delay(200);
  
  if(!checkWatchdog()) {
    Serial.println("❌ Failed to open eyes");
    return false;
  }
  
  stopWatchdog();
  detachAllServos();
  return true;
}

// -----------------------------
// SETUP
// -----------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);

  delay(300);
  openEyesFully();
  delay(500);
  
  Serial.println("Ready - Jaw will talk continuously");
}

// -----------------------------
// LOOP
// -----------------------------
void loop() {
  // Blink every 5 seconds
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 5000) {
    lastBlink = millis();
    blinkQuick();
  }
  
  // Jaw keeps talking continuously
  jawTalkContinuous();
}