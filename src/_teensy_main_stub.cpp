// Entry point for [env:teensy41] — switch the active bring-up test here.
#if defined(ARDUINO_TEENSY41)

#include "Arduino.h"
#include "interface/Interface.hpp"
#include "state_machine/StateMachine.hpp"
#include "motor/Motor.hpp"
#include "motor/GM2804.hpp"
#include "foc_controller/FOCController.hpp"
#include "sensor/AS5048.hpp"
#include "pwm_driver/teensy41/PWMDriverTeensy41.hpp"
#include "current_sense/teensy41/InlineCurrentSenseTeensy41.hpp"
#include "utils/tweakable_params.h"
#include "utils/constants.h"
#include "ldm/GarminV3HP.hpp"
#include "scan_path/ScanPath.hpp"

GarminV3HP ldm = GarminV3HP(GARMIN_SCL, GARMIN_SDA, GARMIN_MODE_PIN);
StateMachine stateMachine;
AS5048 sensorX = AS5048(SENSOR_SPI_CS_X);
AS5048 sensorY = AS5048(SENSOR_SPI_CS_Y);
PWMDriverTeensy41 pwmDriverX = PWMDriverTeensy41(N1_X, N2_X, N3_X, EN_X);
PWMDriverTeensy41 pwmDriverY = PWMDriverTeensy41(N1_Y, N2_Y, N3_Y, EN_Y);
GM2804 motorX(1, SENSOR_OFFSET_X);
GM2804 motorY(2, SENSOR_OFFSET_Y);
FOCController focControllerX(sensorX, pwmDriverX, motorX);
FOCController focControllerY(sensorY, pwmDriverY, motorY);
InlineCurrentSenseTeensy41 currentSenseX;
InlineCurrentSenseTeensy41 currentSenseY;
SinusoidalScanPath scanPath(THETA_INC, PHI_INC, THETA_AMPLITUDE, PHI_AMPLITUDE, THETA_CENTRE, PHI_CENTRE);
Interface interface(stateMachine, focControllerX, focControllerY, sensorX, sensorY, currentSenseX, currentSenseY, ldm, scanPath);

void setup() {

  Serial.begin(BAUD_RATE);
  while (!Serial) {
    delay(100);
  }
  Serial.println("Serial initialised.");
  delay(1000);
  ldm.init();
  focControllerX.init();
  focControllerY.init();
  Serial.println("System initialised.");
  Serial.println("Calibrating...");
  focControllerX.calibrate();
  focControllerY.calibrate();
  Serial.println("Calibrated. Enable to begin.");
}

void loop() {
  static unsigned long cmdTime = millis();
  static unsigned long controlTime = micros();
  static unsigned long statusTime = millis();

  if (millis() - cmdTime > 200) {
    cmdTime = millis();
    interface.commandListener();
  }

  if (stateMachine.enabled) {
    focControllerX.loop();
    focControllerY.loop();
    angleTrackerX.update();
    angleTrackerY.update();

    if ((uint32_t)(micros() - controlTime) >= CONTROL_PACKET_PERIOD_US) {
      controlTime = micros();
      interface.sendControlPacket();
    }

    if (millis() - statusTime >= STATUS_PACKET_PERIOD_MS) {
      statusTime = millis();
      interface.sendStatusPacket();
    }

    if (ldm.isDataReady()) {
      angleTrackerX.calculateMedianAverage();
      angleTrackerY.calculateMedianAverage();
      ldm.triggerMeasurement();
      ldm.readDistance();
    }
  }
}

#endif

// loop FOC and read sensor takes 8us on average (one motor)
// in the office, the Garmin returns 300-400 measurements per second.
// in the office, loopFOC is called 300-400 times per returned distance measurement.
// FOC = 120KHz
// LDM = 300-400Hz