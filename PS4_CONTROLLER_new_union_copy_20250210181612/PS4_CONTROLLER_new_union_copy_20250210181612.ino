#include <Bluepad32.h>
#include <HardwareSerial.h>
#include <Adafruit_SensorLab.h>
#include <Arduino.h>
#include <Adafruit_MMC56x3.h>
#include <Adafruit_Sensor_Calibration.h>

#define RX_PIN 19
#define TX_PIN 17

//global magnetometer vars

float GmagX = 0.0;
float GmagY = 0.0;
float GmagZ = 0.0;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);

// Hard-iron calibration settings 
const float hard_iron[3] = {
  -315.503921, -16.458611, -74.205421
};

// Soft-iron calibration settings
const float soft_iron[3][3] = {
  { 4.367392, 0.035151, 0.492519 },
  { 0.035151, 4.559536, -0.034281 },
  { 0.492519, -0.034281, 2.009672 }
};

HardwareSerial PicoSerial(1);


// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }

  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

//----------------------------------------------------------//
//READ MAGNETOMETER FUNCTION

void read_mag(float *mX, float *mY, float *mZ) {
  static float hi_cal[3];
  // Get a new sensor event
  sensors_event_t event;
  mmc.getEvent(&event);

  // Put raw magnetometer readings into an array
  float mag_data[] = { event.magnetic.x,
                       event.magnetic.y,
                       event.magnetic.z };

  // Apply hard-iron offsets
  for (uint8_t i = 0; i < 3; i++) {
    hi_cal[i] = mag_data[i] - hard_iron[i];
  }

  // Apply soft-iron scaling
  for (uint8_t i = 0; i < 3; i++) {
    mag_data[i] = (soft_iron[i][0] * hi_cal[0]) + (soft_iron[i][1] * hi_cal[1]) + (soft_iron[i][2] * hi_cal[2]);
  }

  *mX = mag_data[0];
  *mY = mag_data[1];
  *mZ = mag_data[2];
}


// ========= SEE CONTROLLER VALUES IN SERIAL MONITOR ========= //

//We will use this section to send controller data over UART//

void dumpGamepad(ControllerPtr ctl, float *mX, float *mY, float *mZ) {

  //sends start bit and splits uint32_t into 4 bytes

  uint32_t throttle_value = ctl->throttle();
  uint32_t brake_value = ctl->brake();
  uint32_t mag_x_value /*= mag_data[0]; */ = *mX;
  uint32_t mag_y_value /*= mag_data[1]; */ = *mY;
  uint32_t mag_z_value /*= mag_data[2]; */ = *mZ;
  
  Serial.print("\nStart byte: ");
  Serial.print(0xFF);
  Serial.print("   ");

  PicoSerial.write(0xFF);
  PicoSerial.write((throttle_value >> 24) & 0xFF);  // First byte
  PicoSerial.write((throttle_value >> 16) & 0xFF);  // Second byte
  PicoSerial.write((throttle_value >> 8) & 0xFF);   // Third byte
  PicoSerial.write(throttle_value & 0xFF);          // Fourth byte

  // PicoSerial.write(0xFE);
  PicoSerial.write((brake_value >> 24) & 0xFF);
  PicoSerial.write((brake_value >> 16) & 0xFF);
  PicoSerial.write((brake_value >> 8) & 0xFF);
  PicoSerial.write((brake_value)&0xFF);

  PicoSerial.write((mag_x_value >> 24) & 0xFF);
  PicoSerial.write((mag_x_value >> 16) & 0xFF);
  PicoSerial.write((mag_x_value >> 8) & 0xFF);
  PicoSerial.write((mag_x_value)&0xFF);

  PicoSerial.write((mag_y_value >> 24) & 0xFF);
  PicoSerial.write((mag_y_value >> 16) & 0xFF);
  PicoSerial.write((mag_y_value >> 8) & 0xFF);
  PicoSerial.write((mag_y_value)&0xFF);

  PicoSerial.write((mag_z_value >> 24) & 0xFF);
  PicoSerial.write((mag_z_value >> 16) & 0xFF);
  PicoSerial.write((mag_z_value >> 8) & 0xFF);
  PicoSerial.write((mag_z_value)&0xFF);

  Serial.printf("\nThrottle: %4d  Brake: %4d", ctl->throttle(), ctl->brake());
  Serial.printf("\nMagnetic Values:  X=%f,  Y=%f, Z=%f", *mX, *mY, *mZ);

  /* All controls:
  ctl->index(),        // Controller Index
  ctl->dpad(),         // D-pad
  ctl->buttons(),      // bitmask of pressed buttons
  ctl->axisX(),        // (-511 - 512) left X Axis
  ctl->axisY(),        // (-511 - 512) left Y axis
  ctl->axisRX(),       // (-511 - 512) right X axis
  ctl->axisRY(),       // (-511 - 512) right Y axis
  ctl->brake(),        // (0 - 1023): brake button  
  ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
  ctl->miscButtons(),  // bitmask of pressed "misc" buttons
  ctl->gyroX(),        // Gyro X
  ctl->gyroY(),        // Gyro Y
  ctl->gyroZ(),        // Gyro Z
  ctl->accelX(),       // Accelerometer X
  ctl->accelY(),       // Accelerometer Y
  ctl->accelZ() // Accelerometer Z
  ); */
}

//this section is likely not needed //

// ========= GAME CONTROLLER ACTIONS SECTION ========= //

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        dumpGamepad(myController, &GmagX, &GmagY, &GmagZ);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);

  PicoSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
}

void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();

  if (dataUpdated) {

    read_mag(&GmagX, &GmagY, &GmagZ);

    processControllers();
  }

  // vTaskDelay(1);
  // delay(1);
}