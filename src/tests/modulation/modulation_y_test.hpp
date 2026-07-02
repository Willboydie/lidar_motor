#pragma once

#include <Arduino.h>
#include "modulation_common.hpp"

// L7 — Motor Y: validate setPhaseVoltage() numerically, then sweep for scope.
inline void modulation_y_test() {
    Serial.print("TEST modulation axis=Y target=");
    Serial.println(wiring_target_name());
    Serial.print("PINS N1=");
    Serial.print(N1_Y);
    Serial.print(" N2=");
    Serial.print(N2_Y);
    Serial.print(" N3=");
    Serial.print(N3_Y);
    Serial.print(" EN=");
    Serial.print(EN_Y);
    Serial.print(" offset=");
    Serial.println(SENSOR_OFFSET_Y, 4);

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);

#if defined(ARDUINO_TEENSY41)
    PWMDriverTeensy41::clearRegisteredSubmodules();
#endif

    PlatformPWMDriver driver(N1_Y, N2_Y, N3_Y, EN_Y);
    if (!driver.init()) {
        Serial.println("RESULT modulation_y FAIL driver_init=0");
        while (true) {
            delay(1000);
        }
    }

    GM2804 motor(2, SENSOR_OFFSET_Y);
    AS5048 sensor(SENSOR_SPI_CS_Y);
    FOCController foc(sensor, driver, motor);

    // Part 1 — numeric math check, coils idle (EN low).
    modulation_force_disable(driver);
    const bool numeric_ok = modulation_numeric_check(foc, driver.getVoltageLimit(), "Y");

    // Part 2 — swept waveform, rotor clamped, EN high.
    modulation_sweep_prompt("Y");
    modulation_sweep(foc, driver, "Y");

    driver.disable();
    driver.deinit();
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    Serial.print("RESULT modulation_y ");
    Serial.print(numeric_ok ? "PASS" : "FAIL");
    Serial.println(" numeric=asserted sweep=scope-confirmed");
}
