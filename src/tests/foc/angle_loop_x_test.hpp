#pragma once

#include <Arduino.h>
#include "angle_loop_common.hpp"

inline void angle_loop_x_test() {
    Serial.print("TEST angle_loop axis=X target=");
    Serial.println(wiring_target_name());

    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

#if defined(ARDUINO_TEENSY41)
    PWMDriverTeensy41::clearRegisteredSubmodules();
#endif

    PlatformPWMDriver driver(N1_X, N2_X, N3_X, EN_X);

#if defined(STM32G4xx)
    InlineCurrentSenseSTM32G4 current_sense(ACS712_MV_PER_AMP, ADC_CS_X_1, ADC_CS_X_2, driver);
    if (!current_sense_bringup(driver, current_sense, "angle_loop_x")) {
        while (true) { delay(1000); }
    }
#else
    if (!driver.init()) {
        Serial.println("RESULT angle_loop_x FAIL driver_init=0");
        while (true) { delay(1000); }
    }
#endif

    GM2804 motor(1, SENSOR_OFFSET_X);
    AS5048 sensor(SENSOR_SPI_CS_X);
    FOCController foc(sensor, driver, motor);

#if defined(STM32G4xx)
    foc.setCurrentSense(current_sense);
#endif

    if (!sensor.init()) {
        Serial.println("RESULT angle_loop_x FAIL encoder_init=0");
        while (true) { delay(1000); }
    }
    sensor.enable();

    if (!foc.init()) {
        Serial.println("RESULT angle_loop_x FAIL foc_init=0");
        while (true) { delay(1000); }
    }

    angle_loop_bench_prompt("X");
    angle_loop_run(foc, driver, "X");

    driver.deinit();
    sensor.deinit();
    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
}
