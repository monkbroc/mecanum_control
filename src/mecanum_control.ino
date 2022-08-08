/*
 * Project mecanum_control
 * Description: Control a robot with 4 mecanum wheels driven by stepper motors
 * Author:
 * Date:
 */

#include "AccelStepper.h"

SYSTEM_THREAD(ENABLED);

void setup() {
  Serial.begin();

  setupJoystick();

  setupSteppers();
}

void loop() {
  // Uncomment to debug
  printJoystick();

  updateSpeed();

  runSteppers();
}

// ===== STEPPERS =====

const int minWheelSpeed = 500;
const int maxWheelSpeed = 1500;

const system_tick_t sleepTimer = 30000;
#define STEPPER_SLEEP_PIN D6
#define SLEEP_OFF HIGH
#define SLEEP_ON LOW

#define STEPPER_MODE_1_PIN A4
#define STEPPER_MODE_2_PIN A5
#define STEPPER_MODE_3_PIN A6

// The Left Front Stepper pins
#define STEPPER_LF_DIR_PIN A3
#define STEPPER_LF_STEP_PIN A2
// The Left Rear Stepper pins
#define STEPPER_LR_DIR_PIN A1
#define STEPPER_LR_STEP_PIN A0
// The Right Front stepper pins
#define STEPPER_RF_DIR_PIN D2
#define STEPPER_RF_STEP_PIN D3
// The Right Rear stepper pins
#define STEPPER_RR_DIR_PIN D4
#define STEPPER_RR_STEP_PIN D5

// Define some steppers and the pins the will use
AccelStepper stepperLF(AccelStepper::DRIVER, STEPPER_LF_STEP_PIN, STEPPER_LF_DIR_PIN);
AccelStepper stepperLR(AccelStepper::DRIVER, STEPPER_LR_STEP_PIN, STEPPER_LR_DIR_PIN);
AccelStepper stepperRF(AccelStepper::DRIVER, STEPPER_RF_STEP_PIN, STEPPER_RF_DIR_PIN);
AccelStepper stepperRR(AccelStepper::DRIVER, STEPPER_RR_STEP_PIN, STEPPER_RR_DIR_PIN);

void setupSteppers() {
  pinMode(STEPPER_SLEEP_PIN, OUTPUT);
  digitalWrite(STEPPER_SLEEP_PIN, SLEEP_ON);

  pinMode(STEPPER_MODE_1_PIN, OUTPUT);
  pinMode(STEPPER_MODE_2_PIN, OUTPUT);
  pinMode(STEPPER_MODE_3_PIN, OUTPUT);

  // LLL: Full step
  // HLL: Half step
  // LHL: Quarter step
  // HHL: Eight step
  // HHH: Sixteenth step
  digitalWrite(STEPPER_MODE_1_PIN, HIGH);
  digitalWrite(STEPPER_MODE_2_PIN, HIGH);
  digitalWrite(STEPPER_MODE_3_PIN, LOW);

  // Left side is inverted
  stepperLF.setPinsInverted(true);
  stepperLR.setPinsInverted(true);

  stepperLF.setMaxSpeed(maxWheelSpeed);
  stepperLR.setMaxSpeed(maxWheelSpeed);
  stepperRF.setMaxSpeed(maxWheelSpeed);
  stepperRR.setMaxSpeed(maxWheelSpeed);
}

system_tick_t motorLastActive = 0;
void stepperSleep(bool motorActive) {
  if (motorActive) {
    digitalWrite(STEPPER_SLEEP_PIN, SLEEP_OFF);
    motorLastActive = millis();
  } else {
    if ((millis() - motorLastActive) > sleepTimer) {
          digitalWrite(STEPPER_SLEEP_PIN, SLEEP_ON);
    }
  }
}

void runSteppers() {
  stepperLF.runSpeed();
  stepperLR.runSpeed();
  stepperRF.runSpeed();
  stepperRR.runSpeed();
}

// ===== JOYSTICK =====

// The Raspberry Pi will write joystick data to I2C on this address
const auto joystickI2cAddress = 8;
const int joystickDeadZone = 32;
const int joystickDiagonal = 56;
const int joystickMax = 127;

enum {
  AXIS_0,
  AXIS_1,
  AXIS_2,
  AXIS_3,
  AXIS_4,
  AXIS_5,
  BUTTON_0,
  BUTTON_1,
  BUTTON_2,
  BUTTON_3,
  BUTTON_4,
  BUTTON_5,
  BUTTON_6,
  BUTTON_7,

  AXIS_COUNT
};

int8_t joystick[AXIS_COUNT] = { 0 };

void setupJoystick() {
  Wire.begin(joystickI2cAddress);
  Wire.onReceive(updateJoystick);
}

bool joystickReceived = false;
void updateJoystick(int availableBytes) {
  while (Wire.available() >= 2) {
    uint8_t axis = Wire.read();
    int8_t value = (int8_t) Wire.read();
    // avoid issues when making value negative
    if (value > joystickMax) {
      value = joystickMax;
    }

    joystickReceived = true;

    if (axis < AXIS_COUNT) {
      joystick[axis] = value;
    }
  }
}

void printJoystick() {
  if (joystickReceived) {
    Serial.printlnf("a0=%4d a1=%4d a2=%4d a3=%4d a4=%4d a5=%4d b0=%d b1=%d b2=%d b3=%d b4=%d b5=%d b6=%d b7=%d",
    joystick[AXIS_0],
    joystick[AXIS_1],
    joystick[AXIS_2],
    joystick[AXIS_3],
    joystick[AXIS_4],
    joystick[AXIS_5],
    joystick[BUTTON_0],
    joystick[BUTTON_1],
    joystick[BUTTON_2],
    joystick[BUTTON_3],
    joystick[BUTTON_4],
    joystick[BUTTON_5],
    joystick[BUTTON_6],
    joystick[BUTTON_7]
    );
    joystickReceived = false;
  }
}

// ==== ROBOT CONTROL LOGIC ====

int linearInterpolate(int x, int x1, int x2, int y1, int y2) {
  if (x < x1) {
    return y1;
  }
  if (x > x2) {
    return y2;
  }
  return (x - x1) * (y2 - y1) / (x2 - x1) + y1;
}

void updateSpeed() {
  int forward = -joystick[AXIS_1];
  int sideways = joystick[AXIS_0];
  int rotate = joystick[AXIS_5];

  bool shouldForward = abs(forward) > joystickDeadZone;
  bool shouldSideways = abs(sideways) > joystickDeadZone;
  bool shouldDiagonal = (abs(forward) > joystickDiagonal && shouldSideways) || (abs(sideways) > joystickDiagonal && shouldForward);
  bool shouldRotate = abs(rotate) > joystickDeadZone;

  bool motorActive = true;

  if (shouldRotate) {
    rotateBy(rotate);
  } else if (shouldDiagonal) {
    moveDiagonalBy(forward, sideways);
  } else if (shouldForward) {
    moveForwardBy(forward);
  } else if (shouldSideways) {
    moveSidewaysBy(sideways);
  } else {
    stopMoving();
    motorActive = false;
  }

  // after 30 seconds of no activity, turn off the steppers
  stepperSleep(motorActive);
}

void rotateBy(int rotate) {
  int speed = linearInterpolate(abs(rotate), joystickDeadZone, joystickMax, minWheelSpeed, maxWheelSpeed);
  if (rotate < 0) {
    speed = -speed;
  }
  stepperLF.setSpeed(speed);
  stepperLR.setSpeed(speed);
  stepperRF.setSpeed(-speed);
  stepperRR.setSpeed(-speed);
}

/*
  To move diagonally, calculate the speed of pairs of wheels using linear interpolation and set the other pair of wheels to 0
 */
void moveDiagonalBy(int forward, int sideways) {
  int speed = linearInterpolate(abs(forward) + abs(sideways), 2 * joystickDeadZone, 2 * joystickMax, minWheelSpeed, maxWheelSpeed);

  if (sideways > 0 && forward > 0) {
    stepperLF.setSpeed(speed);
    stepperLR.setSpeed(0);
    stepperRF.setSpeed(0);
    stepperRR.setSpeed(speed);
  } else if (sideways < 0 && forward < 0) {
    stepperLF.setSpeed(-speed);
    stepperLR.setSpeed(0);
    stepperRF.setSpeed(0);
    stepperRR.setSpeed(-speed);
  } else if (sideways < 0 && forward > 0) {
    stepperLF.setSpeed(0);
    stepperLR.setSpeed(speed);
    stepperRF.setSpeed(speed);
    stepperRR.setSpeed(0);
  } else {
    stepperLF.setSpeed(0);
    stepperLR.setSpeed(-speed);
    stepperRF.setSpeed(-speed);
    stepperRR.setSpeed(0);
  }
}

void moveForwardBy(int forward) {
  int speed = linearInterpolate(abs(forward), joystickDeadZone, joystickMax, minWheelSpeed, maxWheelSpeed);
  if (forward < 0) {
    speed = -speed;
  }
  stepperLF.setSpeed(speed);
  stepperLR.setSpeed(speed);
  stepperRF.setSpeed(speed);
  stepperRR.setSpeed(speed);
}

void moveSidewaysBy(int sideways) {
  int speed = linearInterpolate(abs(sideways), joystickDeadZone, joystickMax, minWheelSpeed, maxWheelSpeed);
  if (sideways < 0) {
    speed = -speed;
  }
  stepperLF.setSpeed(speed);
  stepperLR.setSpeed(-speed);
  stepperRF.setSpeed(-speed);
  stepperRR.setSpeed(speed);
}

void stopMoving() {
  stepperLF.setSpeed(0);
  stepperLR.setSpeed(0);
  stepperRF.setSpeed(0);
  stepperRR.setSpeed(0);
}