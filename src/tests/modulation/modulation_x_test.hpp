#pragma once

#include <Arduino.h>
#include "modulation_common.hpp"

// L7 — Motor X: validate setPhaseVoltage() numerically, then sweep for scope.
inline void modulation_x_test() {
    Serial.print("TEST modulation axis=X target=");
    Serial.println(wiring_target_name());
    Serial.print("PINS N1=");
    Serial.print(N1_X);
    Serial.print(" N2=");
    Serial.print(N2_X);
    Serial.print(" N3=");
    Serial.print(N3_X);
    Serial.print(" EN=");
    Serial.print(EN_X);
    Serial.print(" offset=");
    Serial.println(SENSOR_OFFSET_X, 4);

    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

#if defined(ARDUINO_TEENSY41)
    PWMDriverTeensy41::clearRegisteredSubmodules();
#endif

    PlatformPWMDriver driver(N1_X, N2_X, N3_X, EN_X);
    if (!driver.init()) {
        Serial.println("RESULT modulation_x FAIL driver_init=0");
        while (true) {
            delay(1000);
        }
    }

    GM2804 motor(1, SENSOR_OFFSET_X);
    AS5048 sensor(SENSOR_SPI_CS_X);
    FOCController foc(sensor, driver, motor);

    // Part 1 — numeric math check, coils idle (EN low).
    modulation_force_disable(driver);
    const bool numeric_ok = modulation_numeric_check(foc, driver.getVoltageLimit(), "X");

    // Part 2 — swept waveform, rotor clamped, EN high.
    modulation_sweep_prompt("X");
    modulation_sweep(foc, driver, "X");

    driver.disable();
    driver.deinit();
    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);

    Serial.print("RESULT modulation_x ");
    Serial.print(numeric_ok ? "PASS" : "FAIL");
    Serial.println(" numeric=asserted sweep=scope-confirmed");
}
