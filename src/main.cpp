/// test imports
#include "tests/sensor/sensor_test.hpp"
#include "tests/sensor/finding_offset.hpp"
#include "tests/driver/driver_test.hpp"

// main imports
#include "Arduino.h"
#include "interface/Interface.hpp"
#include "state_machine/StateMachine.hpp"
#include "motor/Motor.hpp"
#include "sensor/AS5048.hpp"
#include "pwm_driver/PWMDriverTeensy41.hpp"
#include "utils/tweakable_params.h"
#include "ldm/GarminV3HP.hpp"
#include "data_stream/DataStream.hpp"

GarminV3HP ldm = GarminV3HP(GARMIN_SCL, GARMIN_SDA, GARMIN_MODE_PIN);
StateMachine stateMachine;
AS5048 sensor1 = AS5048(SENSOR_SPI_CS_1);
AS5048 sensor2 = AS5048(SENSOR_SPI_CS_2);
PWMDriverTeensy41 pwmDriver1 = PWMDriverTeensy41(N1_1, N2_1, N3_1, EN_1);
PWMDriverTeensy41 pwmDriver2 = PWMDriverTeensy41(N1_2, N2_2, N3_2, EN_2);
Motor motor1(sensor1, pwmDriver1, 1);
Motor motor2(sensor2, pwmDriver2, 2);
Interface interface(Serial, stateMachine, motor1, motor2, sensor1, sensor2, ldm);
DataStream dataStream(motor1, motor2, ldm);

void setup() {
  interface.init();
  ldm.init();
  motor1.init();
  motor2.init();
  Serial.println("System initialised. Enable to begin.");

  // TESTS  (activate only one at a time)
  // sensor_test();
  // motor_test();
  // angle_control_test();
  // driver_test();
  // sensor_offset_test();
  // pid_test();
  // ldm_test();
  // i2c_test();
  // garmin_hardware_test();
}

void loop() {
  static unsigned long cmdTime = millis();
  if (millis() - cmdTime > 200) {
    cmdTime = millis();
    interface.commandListener();
    if (stateMachine.enabled) {
      Serial.println("Distance: " + String(ldm.getLatestDistance()) + "  |  Associated Angle 1: " + String(dataStream.associated_angle1, 3) + "  |  Associated Angle 2: " + String(dataStream.associated_angle2, 3));
      // interface.printMotorStatus(motor1);
      // interface.printMotorStatus(motor2);
    }
  }

  if ( stateMachine.enabled ) {
    motor1.loopFOC();
    motor2.loopFOC();
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