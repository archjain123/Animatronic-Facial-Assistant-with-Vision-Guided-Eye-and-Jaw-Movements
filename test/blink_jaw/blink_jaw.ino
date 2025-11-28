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
  {"Eye Pan",        0, 0, 180, 90},
  {"Eye Tilt",       1, 0, 180, 90},
  {"Upper Eyelid R", 2, 0, 180, 90},
  {"Lower Eyelid R", 3, 0, 180, 90},
  {"Upper Eyelid L", 4, 0, 180, 90},
  {"Lower Eyelid L", 5, 0, 180, 90},
  {"Jaw",            6, 80, 120, 90}   // Jaw servo added
};

const int SERVO_COUNT = sizeof(servos) / sizeof(servos[0]);

// -----------------------------
// BASIC SERVO FUNCTION
// -----------------------------
void setServoAngle(uint8_t channel, float angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, 102, 512);
  pca.setPWM(channel, 0, pulse);
}

// -----------------------------
// MULTI-SERVO SYNC MOVEMENT
// -----------------------------
void smoothMoveMulti(int channels[], float fromDeg[], float toDeg[], int count, int steps, int delayMs) {
  float step[count];
  float angles[count];

  for (int i = 0; i < count; i++) {
    step[i] = (toDeg[i] - fromDeg[i]) / float(steps);
    angles[i] = fromDeg[i];
  }

  for (int s = 0; s <= steps; s++) {
    for (int i = 0; i < count; i++) {
      setServoAngle(channels[i], angles[i]);
      angles[i] += step[i];
    }
    delay(delayMs);
  }
}

// -----------------------------
// NATURAL QUICK BLINK
// -----------------------------
void blinkQuick() {
  static unsigned long lastBlink = 0;
  unsigned long now = millis();

  if (now - lastBlink < 5000) return; // blink every 5 seconds
  lastBlink = now;

  int ch[] = {2, 3, 4, 5};
  float openFrom[] =  {30, 150, 150, 30};
  float closeTo[]  = {90,  90,  90, 90};

  float closeFrom[] = {90, 90, 90, 90};
  float openTo[]   = {30,150,150,30};

  smoothMoveMulti(ch, openFrom, closeTo, 4, 10, 3);
  delay(40);
  smoothMoveMulti(ch, closeFrom, openTo, 4, 15, 5);
}

// -----------------------------
// JAW TALKING MOTION
// -----------------------------
void jawTalk() {
  static unsigned long lastMove = 0;
  static bool opening = true;
  unsigned long now = millis();

  if (now - lastMove < 80) return;  // adjust speed
  lastMove = now;

  int jawCh = servos[6].channel;

  static int angle = 90;

  if (opening) angle += 8;
  else angle -= 8;

  if (angle >= 120) { angle = 120; opening = false; }
  if (angle <= 80)  { angle = 80;  opening = true;  }

  setServoAngle(jawCh, angle);
}

// -----------------------------
// INITIAL OPEN EYES
// -----------------------------
void openEyesFully() {
  setServoAngle(2, 30);
  setServoAngle(3, 150);
  setServoAngle(4, 150);
  setServoAngle(5, 30);
}

// -----------------------------
// SETUP
// -----------------------------
void setup() {
  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);

  delay(300);
  openEyesFully();
  delay(500);
}

// -----------------------------
// LOOP (BOTH AT SAME TIME)
// -----------------------------
void loop() {
  blinkQuick();  // eyes blink independently
  jawTalk();     // jaw moves continuously
}
