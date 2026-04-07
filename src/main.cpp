/// test imports
// #include "tests/sensor/sensor_test.hpp"
// #include "tests/sensor/finding_offset.hpp"
// #include "tests/driver/driver_test.hpp"
// #include "tests/scan_path/sine_path_test.hpp"
// #include "tests/sensor/noise_test.hpp"
// #include "tests/ldm/ldm_test.hpp"

// main imports
#include "Arduino.h"
#include "interface/Interface.hpp"
#include "state_machine/StateMachine.hpp"
#include "motor/Motor.hpp"
#include "motor/GM2804.hpp"
#include "foc_controller/FOCController.hpp"
#include "sensor/AS5048.hpp"
#include "pwm_driver/PWMDriverTeensy41.hpp"
#include "utils/tweakable_params.h"
#include "utils/constants.h"
#include "ldm/GarminV3HP.hpp"
#include "data_stream/DataStream.hpp"
#include "scan_path/ScanPath.hpp"

GarminV3HP ldm = GarminV3HP(GARMIN_SCL, GARMIN_SDA, GARMIN_MODE_PIN);
StateMachine stateMachine;
AS5048 sensor1 = AS5048(SENSOR_SPI_CS_1);
AS5048 sensor2 = AS5048(SENSOR_SPI_CS_2);
PWMDriverTeensy41 pwmDriver1 = PWMDriverTeensy41(N1_1, N2_1, N3_1, EN_1);
PWMDriverTeensy41 pwmDriver2 = PWMDriverTeensy41(N1_2, N2_2, N3_2, EN_2);
GM2804 motor1(1, SENSOR_OFFSET_1);
GM2804 motor2(2, SENSOR_OFFSET_2);
FOCController focController1(sensor1, pwmDriver1, motor1);
FOCController focController2(sensor2, pwmDriver2, motor2);
SinusoidalScanPath scanPath(THETA_INC, PHI_INC, THETA_AMPLITUDE, PHI_AMPLITUDE, THETA_CENTRE, PHI_CENTRE);
Interface interface(stateMachine, focController1, focController2, sensor1, sensor2, ldm, scanPath);
DataStream dataStream(focController1, focController2, ldm, scanPath);

void setup() {

  Serial.begin(BAUD_RATE);
  while (!Serial) {
    delay(100);
  }
  Serial.println("Serial initialised.");
  delay(1000);
  ldm.init();
  focController1.init();
  focController2.init();
  Serial.println("System initialised.");
  Serial.println("Calibrating...");
  focController1.calibrate();
  focController2.calibrate();
  Serial.println("Calibrated. Enable to begin.");

  // TESTS  (activate only one at a time)
  // sensor_test();
  // motor_test();
  // driver_test();
  // sensor_offset_test();
  // pid_test();
  // ldm_test();
  // i2c_test();
  // sine_path_test();
  // noise_test();
  // ldm_test();
}

void loop() {
  static unsigned long cmdTime = millis();
  if (millis() - cmdTime > 200) {
    cmdTime = millis();
    interface.commandListener();
    if ( stateMachine.enabled ) {
      interface.printFOCStatus(focController1);
    }
  }

  if ( stateMachine.enabled ) {
    if ( stateMachine.scanning ) {
      scanPath.update();
      focController1.setTargetAngle(scanPath.theta);
      focController2.setTargetAngle(scanPath.phi);
    }
    focController1.loop();
    focController2.loop();
    if ( ldm.isDataReady() ) {
      ldm.triggerMeasurement();
      ldm.readDistance();
      dataStream.sendData();
    }
  }
  // delay(100);
}

// loop FOC and read sensor takes 8us on average (one motor)
// in the office, the Garmin returns 300-400 measurements per second.
// in the office, loopFOC is called 300-400 times per returned distance measurement.
// FOC = 120KHz
// LDM = 300-400Hz