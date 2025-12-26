/// test imports
// #include "tests/motor/motor_test.hpp"
// #include "tests/control/angle_control.hpp"
// #include "tests/sensor/sensor_test.hpp"
// #include "tests/driver/driver_test.hpp"
// #include "tests/sensor/finding_offset.hpp"
// #include "tests/motor/pid_test.hpp"
#include "tests/ldm/ldm_test.hpp"
// #include "tests/ldm/i2c_test.hpp"
// #include "tests/ldm/garmin_hardware_test.hpp"

// #include "Arduino.h"
// #include "interface/Interface.hpp"
// #include "state_machine/StateMachine.hpp"
// #include "motor/Motor.hpp"
// #include "sensor/AS5048.hpp"
// #include "pwm_driver/PWMDriverTeensy41.hpp"
// #include "utils/tweakable_params.h"
// #include "ldm/GarminV3HP.hpp"

// GarminV3HP ldm = GarminV3HP(GARMIN_SCL, GARMIN_SDA, GARMIN_MODE_PIN);
// StateMachine stateMachine;
// AS5048 sensor = AS5048(SENSOR_SPI_CS);
// PWMDriverTeensy41 pwmDriver = PWMDriverTeensy41(N1, N2, N3, EN);
// Motor motor(sensor, pwmDriver);
// Interface interface(Serial, stateMachine, motor, sensor, ldm);


void setup() {
  // interface.init();
  // ldm.init();
  // motor.init();
  // Serial.println("System initialised. Enable to begin.");

  // TESTS  (activate only one at a time)
  // sensor_test();
  // motor_test();
  // angle_control_test();
  // driver_test();
  // sensor_offset_test();
  // pid_test();
  ldm_test();
  // i2c_test();
  // garmin_hardware_test();
  
}

// int counter = 0;
// int FOC_start_time = 0;
// int FOC_duration = 0;


void loop() {
  // if (stateMachine.enabled) counter++;
  // static unsigned long cmdTime = millis();
  // if (millis() - cmdTime > 200) {
  //   cmdTime = millis();
  //   interface.printStatus();
  //   interface.commandListener();
  //   if (stateMachine.enabled) {
  //     Serial.println("FOC duration: " + String(FOC_duration/counter) + " microseconds");
  //     counter = 0;
  //     FOC_duration = 0;
  //   }
  // }

  // FOC_start_time = micros();
  // motor.loopFOC();
  // ldm.readSensor();
  // FOC_duration += (micros() - FOC_start_time);
  
  delay(100);
}