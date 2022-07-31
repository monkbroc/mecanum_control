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

  runSteppers();
}

// ===== STEPPERS =====

#define STEPPER_MODE_1 A4
#define STEPPER_MODE_2 A5
#define STEPPER_MODE_3 A6


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
// AccelStepper stepperLF(AccelStepper::DRIVER, STEPPER_LF_STEP_PIN, STEPPER_LF_DIR_PIN);
// AccelStepper stepperRF(AccelStepper::DRIVER, STEPPER_RF_STEP_PIN, STEPPER_RF_DIR_PIN);
// AccelStepper stepperLR(AccelStepper::DRIVER, STEPPER_LR_STEP_PIN, STEPPER_LR_DIR_PIN);
// AccelStepper stepperRR(AccelStepper::DRIVER, STEPPER_RR_STEP_PIN, STEPPER_RR_DIR_PIN);

void setupSteppers() {
  pinMode(STEPPER_MODE_1, OUTPUT);
  pinMode(STEPPER_MODE_2, OUTPUT);
  pinMode(STEPPER_MODE_3, OUTPUT);

  // LOW, LOW, LOW = No micro-steps
  digitalWrite(STEPPER_MODE_1, HIGH);
  digitalWrite(STEPPER_MODE_2, HIGH);
  digitalWrite(STEPPER_MODE_3, LOW);

  pinMode(STEPPER_RR_DIR_PIN, OUTPUT);
  pinMode(STEPPER_RR_STEP_PIN, OUTPUT);
  digitalWrite(STEPPER_RR_DIR_PIN, HIGH);
  
  // auto accel = 200.0;
  // auto maxSpeed = 1000.0;
  // stepperLF.setAcceleration(accel);
  // stepperLF.setMaxSpeed(maxSpeed);
  // stepperRF.setAcceleration(accel);
  // stepperRF.setMaxSpeed(maxSpeed);
  // stepperLR.setAcceleration(accel);
  // stepperLR.setMaxSpeed(maxSpeed);
  // stepperRR.setAcceleration(accel);
  // stepperRR.setMaxSpeed(maxSpeed);

  // stepperLF.runSpeed();
  // stepperRF.runSpeed();
  // stepperLR.runSpeed();
  // stepperRR.runSpeed();
}

void runSteppers() {
  // stepperLF.runSpeed();
  // stepperRF.runSpeed();
  // stepperLR.runSpeed();
  // stepperRR.runSpeed();

  digitalWrite(STEPPER_RR_STEP_PIN, HIGH);
  delay(1);
  digitalWrite(STEPPER_RR_STEP_PIN, LOW);
  delay(1);
}

// ===== JOYSTICK =====

// The Raspberry Pi will write joystick data to I2C on this address
const auto joystickI2cAddress = 8;

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