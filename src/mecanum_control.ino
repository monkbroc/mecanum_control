/*
 * Project mecanum_control
 * Description: Control a robot with 4 mecanum wheels driven by stepper motors
 * Author:
 * Date:
 */

SYSTEM_THREAD(ENABLED);

// The Raspberry Pi will write joystick data to I2C on this port
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

void setup() {
  setupJoystick();

  Serial.begin();
}

void loop() {
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
  delay(100);
}

void setupJoystick() {
  Wire.begin(joystickI2cAddress);
  Wire.onReceive(updateJoystick);
}

void updateJoystick(int availableBytes) {
  if (availableBytes >= 2) {
    uint8_t axis = Wire.read();
    int8_t value = (int8_t) Wire.read();

    if (axis < AXIS_COUNT) {
      joystick[axis] = value;
    }
  }
}