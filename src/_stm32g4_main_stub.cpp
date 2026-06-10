// Entry point for [env:stm32g4] until the full firmware main.cpp is ported.
#if defined(STM32G4xx)

#include <Arduino.h>
// #include "tests/foc/stm32g4_foc_test.hpp"
// #include "tests/sensor/electrical_offset_test.hpp"
// #include "tests/motor/stm32g4_open_loop.hpp"
// #include "tests/sensor/electrical_offset_accuracy_test.hpp"
// #include "tests/sensor/electrical_offset_autocal_test.hpp"
// #include "tests/sensor/stm32g4_sensor_test.hpp"
#include "tests/current_sense/stm32g4_adc_raw_test.hpp"

void setup() {
  // stm32g4_foc_test();
  // electrical_offset_test();
  // stm32g4_open_loop();
  // electrical_offset_accuracy_test();
  // electrical_offset_autocal_test();
  // stm32g4_sensor_test();
  stm32g4_adc_raw_test();
}

void loop() {}

#endif