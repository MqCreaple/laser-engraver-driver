#include "control.h"

#define CHECKPARAM(paramCnt, n) if(paramCnt != n) { Serial.println("Incorrect number of parameters. Abort."); return; }

float *parseParams(const String &command, int index, int &paramCnt) {
  float *params = new float[128];
  int lastIndex = index;
  paramCnt = 0;
  for(int i = 0; i < 128; i++) {
    while(command[lastIndex] == ' ') {
      lastIndex += 1;
    }
    int nextIndex = command.indexOf(' ', lastIndex);
    if(nextIndex == -1) {
      break;
    }
    params[paramCnt] = command.substring(lastIndex, nextIndex).toFloat();
    paramCnt += 1;
    lastIndex = nextIndex;
  }
  return params;
}

// change params[0] and params[1] to the relative position from the current position.
//
// This is useful when using the lowercase commands in SVG, such as `m dx dy`.
void convertDeltaPosition(float *params) {
  float curX = getCurX();
  float curY = getCurY();
  params[0] += curX;
  params[0] += curY;
}

float lastStartX = -1.0;
float lastStartY = -1.0;    // record the last starting point of a continuous path. Used in the Z command.
float lastBezierX = -1.0;
float lastBezierY = -1.0;   // record the last ending point of the Bezier curve
float lastBezierCX = -1.0;
float lastBezierCY = -1.0;
void executeCommand(const String &command) {
  // TODO: Make this compatible with the SVG format
  // get the operation
  int index = command.indexOf(' ');
  String op = command.substring(0, index);
  int paramCnt;
  float *params = parseParams(command, index, paramCnt);
  if(op == "M" || op == "m") {
    // move to position
    CHECKPARAM(paramCnt, 2);
    if(op == "m") {
      convertDeltaPosition(params);
    }
    moveToMillimeter(params[0], params[1]);
    lastStartX = params[0];
    lastStartY = params[1];
  } else if(op == "L") {
    // draw line path from the current position
    drawPath(params, paramCnt / 2);
  } else if(op == "H" || op == "h") {
    // draw horizontal line
    CHECKPARAM(paramCnt, 1);
    float y = getCurY();
    float x = params[0];
    if(op == "h") {
      x += getCurX();
    }
    startLaser();
    moveLineMillimeter(x, y);
    stopLaser();
  } else if(op == "V" || op == "v") {
    // draw vertical line
    CHECKPARAM(paramCnt, 1);
    float x = getCurX();
    float y = params[0];
    if(op == "v") {
      y += getCurY();
    }
    startLaser();
    moveLineMillimeter(x, y);
    stopLaser();
  } else if(op == "Z" || op == "z") {
    CHECKPARAM(paramCnt, 0);
    startLaser();
    if(lastStartX != -1 && lastStartY != -1) {
      moveLineMillimeter(lastStartX, lastStartY);
    }
    stopLaser();
  } else if(op == "C" || op == "c") {
    // draw Bezier curve
    CHECKPARAM(paramCnt, 6);
    if(op == "c") {
      convertDeltaPosition(params);
      convertDeltaPosition(params + 2);
      convertDeltaPosition(params + 4);
    }
    drawBezier(params[0], params[1], params[2], params[3], params[4], params[5]);
    lastBezierX = params[4];
    lastBezierY = params[5];
    lastBezierCX = params[2];
    lastBezierCY = params[3];
  } else if(op == "S" || op == "s") {
    CHECKPARAM(paramCnt, 4);
    if(op == "s") {
      convertDeltaPosition(params);
      convertDeltaPosition(params + 2);
    }
    float nextCX = lastBezierX * 2 - lastBezierCX;
    float nextCY = lastBezierY * 2 - lastBezierCY;
    moveToMillimeter(lastBezierX, lastBezierY);
    drawBezier(nextCX, nextCY, params[0], params[1], params[2], params[3]);
    lastBezierX = params[2];
    lastBezierY = params[3];
    lastBezierCX = params[0];
    lastBezierCY = params[1];
  } else if(op == "arc") {
    // draw arc
    // TODO: incompatible with SVG format
    CHECKPARAM(paramCnt, 5);
    drawArc(params[0], params[1], params[2], params[3], params[4]);
  } else if(op == "circle") {
    // draw circle
    // TODO: incompatible with SVG format
    CHECKPARAM(paramCnt, 3);
    drawCircleMillimeter(params[0], params[1], params[2]);
  } else if(op == "dot") {
    // draw dot
    // TODO: incompatible with SVG format
    CHECKPARAM(paramCnt, 0);
    startLaser();
    delay(100);
    stopLaser();
  }
  delete[] params;
  Serial.println("Finished command");
}

void setup() {
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);  // set this to LOW at first to ensure safety

  Serial.begin(BAUD);
  pinMode(STEPPER_X_STEP_PIN, OUTPUT);
  pinMode(STEPPER_X_DIR_PIN, OUTPUT);
  pinMode(STEPPER_Y_STEP_PIN, OUTPUT);
  pinMode(STEPPER_Y_DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  stepperX.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperX.setAcceleration(500);
  stepperY.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperY.setAcceleration(500);
  digitalWrite(STEPPER_Y_DIR_PIN, LOW);

  homing();
}

void loop() {
  if(Serial.available()) {
    String command = Serial.readStringUntil('\n');   // read a line
    command += ' ';   // to be able to parse the last parameter
    executeCommand(command);
  }
}
