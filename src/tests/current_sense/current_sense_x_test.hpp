#pragma once

#include <Arduino.h>
#include "current_sense_common.hpp"

#if defined(STM32G4xx)

// L11 — Motor X: inline current-sense system validation (STM32G4 only).
inline void current_sense_x_test() {
    Serial.print("TEST current_sense axis=X target=");
    Serial.println(wiring_target_name());
    Serial.print("PINS N1=");
    Serial.print(N1_X);
    Serial.print(" N2=");
    Serial.print(N2_X);
    Serial.print(" N3=");
    Serial.print(N3_X);
    Serial.print(" EN=");
    Serial.print(EN_X);
    Serial.print(" ADC=");
    Serial.print(ADC_CS_X_1);
    Serial.print("/");
    Serial.print(ADC_CS_X_2);
    Serial.print(" offset=");
    Serial.println(SENSOR_OFFSET_X, 4);

    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    PlatformPWMDriver driver(N1_X, N2_X, N3_X, EN_X);
    InlineCurrentSenseSTM32G4 current_sense(ACS712_MV_PER_AMP, ADC_CS_X_1, ADC_CS_X_2, driver);

    if (!current_sense_bringup(driver, current_sense, "current_sense_x")) {
        driver.deinit();
        wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
        while (true) { delay(1000); }
    }

    GM2804 motor(1, SENSOR_OFFSET_X);
    AS5048 sensor(SENSOR_SPI_CS_X);
    FOCController foc(sensor, driver, motor);
    foc.setCurrentSense(current_sense);

    if (!sensor.init()) {
        Serial.println("RESULT current_sense_x FAIL encoder_init=0 (angle required to project Iq/Id)");
        driver.deinit();
        wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
        while (true) { delay(1000); }
    }
    sensor.enable();

    current_sense_bench_prompt("X", SENSOR_OFFSET_X);
    current_sense_run(foc, driver, current_sense, sensor, SENSOR_OFFSET_X, "X");

    driver.disable();
    driver.deinit();
    sensor.deinit();
    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
}

#endif // STM32G4xx
