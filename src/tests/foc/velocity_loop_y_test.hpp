#pragma once

#include <Arduino.h>
#include "velocity_loop_common.hpp"

inline void velocity_loop_y_test() {
    Serial.print("TEST velocity_loop axis=Y target=");
    Serial.println(wiring_target_name());

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);

#if defined(ARDUINO_TEENSY41)
    PWMDriverTeensy41::clearRegisteredSubmodules();
#endif

    PlatformPWMDriver driver(N1_Y, N2_Y, N3_Y, EN_Y);

#if defined(STM32G4xx)
    InlineCurrentSenseSTM32G4 current_sense(ACS712_MV_PER_AMP, ADC_CS_Y_1, ADC_CS_Y_2, driver);
    if (!current_sense_bringup(driver, current_sense, "velocity_loop_y")) {
        while (true) { delay(1000); }
    }
#else
    if (!driver.init()) {
        Serial.println("RESULT velocity_loop_y FAIL driver_init=0");
        while (true) { delay(1000); }
    }
#endif

    GM2804 motor(2, SENSOR_OFFSET_Y);
    AS5048 sensor(SENSOR_SPI_CS_Y);
    FOCController foc(sensor, driver, motor);

#if defined(STM32G4xx)
    foc.setCurrentSense(current_sense);
#endif

    if (!sensor.init()) {
        Serial.println("RESULT velocity_loop_y FAIL encoder_init=0");
        while (true) { delay(1000); }
    }
    sensor.enable();

    if (!foc.init()) {
        Serial.println("RESULT velocity_loop_y FAIL foc_init=0");
        while (true) { delay(1000); }
    }

    velocity_loop_bench_prompt("Y");
    velocity_loop_run(foc, driver, "Y");

    driver.deinit();
    sensor.deinit();
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);
}
