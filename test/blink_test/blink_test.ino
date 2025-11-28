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
// SERVO PWM FUNCTION
// -----------------------------
void setServoAngle(uint8_t channel, float angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, 102, 512);  // PCA9685 pulse range
  pca.setPWM(channel, 0, pulse);
}

// -----------------------------
// MULTI-SERVO SYNCHRONIZED MOVE
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
// NATURAL QUICK BLINK (SYNCED)
// -----------------------------
void blinkQuick() {
  int ch[] = {2, 3, 4, 5};  // UER, LER, UEL, LEL

  // QUICK CLOSING (blink)
  float fromOpen[] = {30, 150, 150, 30};
  float toClose[]  = {90,  90,  90, 90};

  smoothMoveMulti(ch, fromOpen, toClose, 4, 10, 3);
  delay(40);

  // SMOOTH RE-OPEN
  float fromClose[] = {90, 90, 90, 90};
  float toOpen[]    = {30,150,150,30};

  smoothMoveMulti(ch, fromClose, toOpen, 4, 15, 5);
}

// -----------------------------
// START WITH EYES FULLY OPEN
// -----------------------------
void openEyesFully() {
  setServoAngle(2, 30);     // UER open
  setServoAngle(3, 150);    // LER open
  setServoAngle(4, 150);    // UEL open
  setServoAngle(5, 30);     // LEL open
}

// -----------------------------
// SETUP
// -----------------------------
void setup() {
  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);

  delay(300);
  openEyesFully();   // Start OPEN
  delay(500);
}

// -----------------------------
// LOOP
// -----------------------------
void loop() {
  delay(2000);        // Eyes remain open for 5 seconds
  blinkQuick();       // Perform natural blink
}
