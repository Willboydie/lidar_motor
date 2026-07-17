#pragma once

#include <Arduino.h>
#include "current_loop_pi_common.hpp"

#if defined(STM32G4xx)

inline void current_loop_pi_y_test() {
    Serial.print("TEST current_loop_pi axis=Y target=");
    Serial.println(wiring_target_name());

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);

    PlatformPWMDriver driver(N1_Y, N2_Y, N3_Y, EN_Y);
    InlineCurrentSenseSTM32G4 current_sense(ACS712_MV_PER_AMP, ADC_CS_Y_1, ADC_CS_Y_2, driver);

    if (!current_sense_bringup(driver, current_sense, "current_loop_pi_y")) {
        while (true) { delay(1000); }
    }

    GM2804 motor(2, SENSOR_OFFSET_Y);
    AS5048 sensor(SENSOR_SPI_CS_Y);
    FOCController foc(sensor, driver, motor);

    if (!sensor.init()) {
        Serial.println("RESULT current_loop_pi_y FAIL encoder_init=0");
        while (true) { delay(1000); }
    }
    sensor.enable();

    if (!foc.init()) {
        Serial.println("RESULT current_loop_pi_y FAIL foc_init=0");
        while (true) { delay(1000); }
    }

    current_loop_pi_bench_prompt("Y");
    current_loop_pi_run(foc, driver, current_sense, "Y");

    driver.deinit();
    sensor.deinit();
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);
}

#endif
