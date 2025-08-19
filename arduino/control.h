#include <math.h>
#include <AccelStepper.h>

const int ENABLE_PIN = 8;
const int STEPPER_X_STEP_PIN = 2;
const int STEPPER_X_DIR_PIN = 5;
const int STEPPER_Y_STEP_PIN = 3;
const int STEPPER_Y_DIR_PIN = 6;
const int LASER_PIN = 11;
const long STEPPER_MAX_SPEED = 1000;
const float STEPS_PER_MILLIMETER = 80.0; // 1 rotation = 40 mm = 200 steps * 16 (gearing)   -->   1 mm = 80 steps
const int BAUD = 9600;
const float ARC_RESOLUTION = 1;     // unit: mm

AccelStepper stepperX(AccelStepper::DRIVER, STEPPER_X_STEP_PIN, STEPPER_X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, STEPPER_Y_STEP_PIN, STEPPER_Y_DIR_PIN);

void moveToStep(long, long);

inline void startLaser() {
  digitalWrite(LASER_PIN, HIGH);
}

inline void stopLaser() {
  digitalWrite(LASER_PIN, LOW);
}

inline float getCurX() {
  return stepperX.currentPosition() / STEPS_PER_MILLIMETER;
}

inline float getCurY() {
  return stepperY.currentPosition() / STEPS_PER_MILLIMETER;
}

void homing() {
  while (true) {
    unsigned long timestamp = millis();
    if (timestamp > 2000) {
      stepperX.stop();
      stepperY.stop();
      break;
    } else {
      stepperX.setSpeed(-500);
      stepperX.runSpeed();
      stepperY.setSpeed(-500);
      stepperY.runSpeed();
    }
  }
  delay(1000);
  stepperX.setCurrentPosition(-STEPS_PER_MILLIMETER);  // move 1mm away from the zero position
  stepperY.setCurrentPosition(-STEPS_PER_MILLIMETER);
  moveToStep(0, 0);
  Serial.println("Homing completed");
}

// x, y: steps
// duration: seconds
void moveToStep(long x, long y) {
  if (x < 0 || y < 0) {
    // Cannot move out of bound
    Serial.print("Cannot move to: ");
    Serial.print(x);
    Serial.print(", ");
    Serial.println(y);
    Serial.println(". Point exceeds the border. Aborted.");
    return;
  }
  // Serial.print("Moving to: ");
  // Serial.print(x);
  // Serial.print(", ");
  // Serial.println(y);
  stepperX.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperY.setMaxSpeed(STEPPER_MAX_SPEED);
  while (true) {
    if (stepperX.currentPosition() == x && stepperY.currentPosition() == y) {
      break;
    }
    stepperX.moveTo(x);
    stepperY.moveTo(y);
    stepperX.run();
    stepperY.run();
  }
}

inline void moveToMillimeter(float x, float y) {
  long xStep = x * STEPS_PER_MILLIMETER;
  long yStep = y * STEPS_PER_MILLIMETER;
  moveToStep(xStep, yStep);
}

// move along a line from the current position to (x, y)
void moveLineStep(long x, long y) {
  if (x < 0 || y < 0) {  // TODO: Add maximum bound check
    // Cannot move out of bound
    Serial.print("Cannot move to: ");
    Serial.print(x);
    Serial.print(", ");
    Serial.println(y);
    Serial.println("Aborted.");
    return;
  }
  long curX = stepperX.currentPosition();
  long curY = stepperY.currentPosition();
  long dx = x - curX;
  long dy = y - curY;
  // float dist = hypot(dx, dy);
  // float duration = dist / STEPPER_MAX_SPEED;  // unit: seconds
  // float xSpeed = dx / duration;
  // float ySpeed = dy / duration;
  // long durationMillis = duration * 1000;  // unit: milliseconds

  // stepperX.setSpeed(xSpeed);
  // stepperY.setSpeed(ySpeed);
  // unsigned long curTime = millis();
  // while (millis() <= curTime + durationMillis) {
  //   stepperX.runSpeed();
  //   stepperY.runSpeed();
  // }
  // stepperX.stop();
  // stepperY.stop();
  // moveToStep(x, y);  // perform a final correction to ensure the step is accurate

  // use Bresenham's line drawing algorithm to draw the line
  digitalWrite(STEPPER_X_DIR_PIN, (dx >= 0) ? HIGH : LOW);
  digitalWrite(STEPPER_Y_DIR_PIN, (dy >= 0) ? HIGH : LOW);
  dx = abs(dx);
  dy = abs(dy);
  long distMajor, distMinor;
  int pinMajor, pinMinor;
  if(dx > dy) {
    distMajor = dx; distMinor = dy;
    pinMajor = STEPPER_X_STEP_PIN;
    pinMinor = STEPPER_Y_STEP_PIN;
  } else {
    distMajor = dy; distMinor = dx;
    pinMajor = STEPPER_Y_STEP_PIN;
    pinMinor = STEPPER_X_STEP_PIN;
  }
  float slope = ((float) (distMinor)) / distMajor;
  // Serial.println(slope);
  // Serial.print(pinMajor);
  // Serial.print(" ");
  // Serial.print(pinMinor);
  long curMajor = 0, curMinor = 0;
  float tmp = 0.0;
  while(curMajor < distMajor || curMinor < distMinor) {
    if(curMajor < distMajor) {
      digitalWrite(pinMajor, HIGH);
      curMajor += 1;
    }
    tmp += slope;
    if(tmp >= 1.0) {
      digitalWrite(pinMinor, HIGH);
      tmp -= 1.0;
      curMinor += 1;
    }
    delay(1);
    digitalWrite(pinMajor, LOW);
    digitalWrite(pinMinor, LOW);
    delay(1);
  }
  stepperX.setCurrentPosition(x);
  stepperY.setCurrentPosition(y);
}

inline void moveLineMillimeter(float x, float y) {
  long xStep = x * STEPS_PER_MILLIMETER;
  long yStep = y * STEPS_PER_MILLIMETER;
  moveLineStep(xStep, yStep);
}

const bool CIRCLE_QUADRANT_DIRS[4][2] = {
  {LOW, HIGH},
  {LOW, LOW},
  {HIGH, LOW},
  {HIGH, HIGH}
};
const int CIRCLE_QUADRANT_STEPS[4][2] = {
  {STEPPER_Y_STEP_PIN, STEPPER_X_STEP_PIN},
  {STEPPER_X_STEP_PIN, STEPPER_Y_STEP_PIN},
  {STEPPER_Y_STEP_PIN, STEPPER_X_STEP_PIN},
  {STEPPER_X_STEP_PIN, STEPPER_Y_STEP_PIN},
};
void drawCircleStep(long x, long y, float radius) {
  // move along a circle using Jesko's algorithm.
  // compute the step array for one octant
  if(x - radius < 0 || y - radius < 0) {
    Serial.println("Cannot draw circle: exceeds the border. Aborted.");
    return;
  }
  long arraySize = ceil(radius * M_SQRT1_2) + 1;
  long bitmaskSize = (arraySize>>3) + 1;
  if(bitmaskSize > 512) {
    Serial.println("Circle too big. Abort.");
    return;
  }
  unsigned char *stepArray = new unsigned char[bitmaskSize];        // use a bitmask to store the array
  float t1 = radius / 16;
  long xn = round(radius);
  long yn = 0;
  for(arraySize = 0; xn > yn; arraySize++) {
    // Step along y direction. Occasionally step along x direction.
    yn += 1;
    t1 += yn;
    float t2 = t1 - xn;
    if(t2 >= 0) {
      t1 = t2;
      xn -= 1;
      stepArray[arraySize>>3] |= (1 << (arraySize&7));
    } else {
      stepArray[arraySize>>3] &= ~(1 << (arraySize&7));
    }
  }
  bool spur = (xn == yn);     // special case for the spurious circle
  // move to the starting point of the circle and draw 8 octants
  moveToStep(x + radius, y);
  startLaser();
  for(int quadrant = 0; quadrant < 4; quadrant++) {
    digitalWrite(STEPPER_X_DIR_PIN, CIRCLE_QUADRANT_DIRS[quadrant][0]);
    digitalWrite(STEPPER_Y_DIR_PIN, CIRCLE_QUADRANT_DIRS[quadrant][1]);
    int firstPin = CIRCLE_QUADRANT_STEPS[quadrant][0];
    int secondPin = CIRCLE_QUADRANT_STEPS[quadrant][1];
    for(int i = 0; i < arraySize; i++) {
      digitalWrite(firstPin, HIGH);
      if(stepArray[i>>3] & (1 << (i&7))) {
        digitalWrite(secondPin, HIGH);
      }
      delay(1);
      digitalWrite(firstPin, LOW);
      digitalWrite(secondPin, LOW);
      delay(1);
    }
    for(int i = arraySize - 2 + spur; i >= 0; i--) {
      digitalWrite(secondPin, HIGH);
      if(stepArray[i>>3] & (1 << (i&7))) {
        digitalWrite(firstPin, HIGH);
      }
      delay(1);
      digitalWrite(secondPin, LOW);
      digitalWrite(firstPin, LOW);
      delay(1);
    }
  }
  stopLaser();
  delete[] stepArray;
}

inline void drawCircleMillimeter(float x, float y, float radius) {
  drawCircleStep(x * STEPS_PER_MILLIMETER, y * STEPS_PER_MILLIMETER, radius * STEPS_PER_MILLIMETER);
}

// Draw a path made up of line segments starting from the current x and y position. Length unit: millimeter
//
// `n` is the number of points to draw, not the size of the array. The array `path` need to have at lease `2*n` points.
void drawPath(float path[], int n) {
  if (n <= 0) {
    return;
  }
  startLaser();
  for (int i = 0; i < n; i++) {
    moveLineMillimeter(path[i * 2], path[i * 2 + 1]);
  }
  stopLaser();
}

// Draw an arc centered at (x, y) with the give radius and starting/ending angle.
//
// (+radius, 0) is angle 0. Angle increases counterclockwise.
void drawArc(float x, float y, float radius, float startAngle, float endAngle) {
  float arcLen = abs(endAngle - startAngle) * radius;
  int divisions = ceil(arcLen / ARC_RESOLUTION);
  Serial.print("Points on the arc: ");
  Serial.println(divisions);
  float angleStep = (endAngle - startAngle) / divisions;
  float *points = new float[divisions * 2];
  for(int i = 0; i < divisions; i++) {
    float angle = startAngle + angleStep * (i + 1);
    points[i * 2] = x + radius * cos(angle);
    points[i * 2 + 1] = y + radius * sin(angle);
  }
  moveToMillimeter(x + radius * cos(startAngle), y + radius * sin(startAngle));
  drawPath(points, divisions);
  delete[] points;
}

// draw a Bezier curve from the current point to (x, y), with two control points
// (xc1, yc1) and (xc2, yc2).
void drawBezier(float xc1, float yc1, float xc2, float yc2, float x, float y) {
  float curX = getCurX();
  float curY = getCurY();
  float arcLenUpperBound = hypot(xc1 - curX, yc1 - curY)
                         + hypot(xc2 - xc1, yc2 - yc1)
                         + hypot(x - xc2, y - yc2);
  int divisions = ceil(arcLenUpperBound / ARC_RESOLUTION);
  float *points = new float[divisions * 2];
  for(int i = 0; i < divisions; i++) {
    float t = ((float) (i + 1)) / divisions;
    float t1 = 1 - t;
    float xt1 = t1 * curX + t * xc1;
    float yt1 = t1 * curY + t * yc1;
    float xt2 = t1 * xc1 + t * xc2;
    float yt2 = t1 * yc1 + t * yc2;
    float xt3 = t1 * xc2 + t * x;
    float yt3 = t1 * yc2 + t * y;
    float xu1 = t1 * xt1 + t * xt2;
    float yu1 = t1 * yt1 + t * yt2;
    float xu2 = t1 * xt2 + t * xt3;
    float yu2 = t1 * yt2 + t * yt3;
    points[i * 2] = t1 * xu1 + t * xu2;
    points[i * 2 + 1] = t1 * yu1 + t * yu2;
    // Serial.print(points[i * 2]);
    // Serial.print(", ");
    // Serial.println(points[i * 2 + 1]);
  }
  drawPath(points, divisions);
  delete[] points;
}