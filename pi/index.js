const gamepad = require("gamepad");
const Raspi = require('raspi-io').RaspiIO;
const { Board } = require('johnny-five');
const board = new Board({
  io: new Raspi(),
  repl: false
});
 
// Initialize the gamepad library
gamepad.init();
 
// List the state of all currently attached devices
for (let i = 0, l = gamepad.numDevices(); i < l; i++) {
  console.log(i, gamepad.deviceAtIndex());
}

// Initialize the Johnny Five board
board.on('ready', function () {
  console.log('Johnny Five is ready!');

  // Support a fixed number of axes and buttons
  const AXIS_COUNT = 6;
  const BUTTON_COUNT = 8;

  // Transmit axis and button motion over I2C
  function sendMotion({ axis, button, value }) {
    // The Photon will expect joystick data to I2C on this address
    const joystickI2cAddress = 8;

    let id;
    if (typeof axis !== 'undefined' && axis < AXIS_COUNT) {
      // Axes have low IDs...
      id = axis;
    } else if (typeof button !== 'undefined' && button < BUTTON_COUNT) {
      // ...followed by buttons with the next set of IDs
      id = button + AXIS_COUNT;
    }

    // Don't transmit unsupported axis or button
    if (typeof id === 'undefined') {
      return;
    }

    try {
      //console.log(`Sent ${id} ${value} over I2C`);
      board.io.i2cWrite(joystickI2cAddress, [id, value]);
    } catch (error) {
      console.error('Joystick I2C transmission failed. Check I2C wiring between Pi and Photon');
    }
  }
 
  // Create a game loop and poll for events
  setInterval(gamepad.processEvents, 100);
  // Scan for new gamepads as a slower rate
  setInterval(gamepad.detectDevices, 1000);

  // 3d mouse
  // axis 0: left right (positive right)
  // axis 1: forward backward (positive backward)
  // axis 2: up down (positive down)
  // axis 3: tilt forward backward (positive backward)
  // axis 4: tilt left right (positive left)
  // axis 5: rotate left right (positive right)
  // button 0: left button
  // button 1: right button

  // Listen for move events on all gamepads and transmit over I2C
  gamepad.on("move", function (id, axis, value) {
    let roundedValue = Math.round(value * 128);
    if (roundedValue >= 128) {
      roundedValue = 127;
    }
    sendMotion({ axis, value: roundedValue });
  });

  // Listen for button down events on all gamepads
  gamepad.on("down", function (id, num) {
    sendMotion({ button: num, value: 1 });
  });

  // Listen for button up events on all gamepads
  gamepad.on("up", function (id, num) {
    sendMotion({ button: num, value: 0 });
  });
});
 
