#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

// neck servo channels
#define NECK_ROTATE 9
#define NECK_TILT_L 7
#define NECK_TILT_R 8

// limits
#define ROT_MIN 70     // rotation left
#define ROT_MAX 160    // rotation right
#define TILT_MIN 0
#define TILT_MAX 180

// smooth setter
void setServoAngle(uint8_t channel, float angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, 102, 512);
  pca.setPWM(channel, 0, pulse);
}

void smoothNeckMove(float rotFrom, float rotTo,
                    float tiltFrom, float tiltTo,
                    int steps, int delayMs)
{
  float rotStep  = (rotTo - rotFrom) / steps;
  float tiltStep = (tiltTo - tiltFrom) / steps;

  float rot = rotFrom;
  float tilt = tiltFrom;

  for(int i=0; i<=steps; i++){
    setServoAngle(NECK_ROTATE, rot);

    // tilt left + right opposite directions
    setServoAngle(NECK_TILT_L, tilt);
    setServoAngle(NECK_TILT_R, 180 - tilt);

    rot  += rotStep;
    tilt += tiltStep;

    delay(delayMs);
  }
}

void setup() {
  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);
  delay(500);

  // center starting positions
  setServoAngle(NECK_ROTATE, 115);
  setServoAngle(NECK_TILT_L, 90);
  setServoAngle(NECK_TILT_R, 90);
}

void loop() {

  // ---- LEFT LOOK + DOWN ----
  smoothNeckMove(115, ROT_MIN, 90, 50, 50, 20);

  // ---- RIGHT LOOK + UP ----
  smoothNeckMove(ROT_MIN, ROT_MAX, 50, 130, 60, 20);

  // ---- BACK TO CENTER ----
  smoothNeckMove(ROT_MAX, 115, 130, 90, 50, 20);
}
