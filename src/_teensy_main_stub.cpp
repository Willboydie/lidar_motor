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
AS5048 sensor1 = AS5048(SENSOR_SPI_CS_X);
AS5048 sensor2 = AS5048(SENSOR_SPI_CS_Y);
PWMDriverTeensy41 pwmDriver1 = PWMDriverTeensy41(N1_X, N2_X, N3_X, EN_X);
PWMDriverTeensy41 pwmDriver2 = PWMDriverTeensy41(N1_Y, N2_Y, N3_Y, EN_Y);
GM2804 motor1(1, SENSOR_OFFSET_X);
GM2804 motor2(2, SENSOR_OFFSET_Y);
FOCController focController1(sensor1, pwmDriver1, motor1);
FOCController focController2(sensor2, pwmDriver2, motor2);
InlineCurrentSenseTeensy41 currentSense1;
InlineCurrentSenseTeensy41 currentSense2;
SinusoidalScanPath scanPath(THETA_INC, PHI_INC, THETA_AMPLITUDE, PHI_AMPLITUDE, THETA_CENTRE, PHI_CENTRE);
Interface interface(stateMachine, focController1, focController2, sensor1, sensor2, currentSense1, currentSense2, ldm, scanPath);

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
    focController1.loop();
    focController2.loop();

    if ((uint32_t)(micros() - controlTime) >= CONTROL_PACKET_PERIOD_US) {
      controlTime = micros();
      interface.sendControlPacket();
    }

    if (millis() - statusTime >= STATUS_PACKET_PERIOD_MS) {
      statusTime = millis();
      interface.sendStatusPacket();
    }

    if (ldm.isDataReady()) {
      ldm.triggerMeasurement();
      ldm.readDistance();
      interface.onLdmDistanceReady();
    }
  }
}

#endif
