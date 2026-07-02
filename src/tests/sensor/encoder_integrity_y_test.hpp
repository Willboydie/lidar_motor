#pragma once

#include <Arduino.h>
#include "../../sensor/AS5048.hpp"
#include "sensor_common.hpp"

// L3 — Motor Y: AS5048 angle integrity, diagnostics, rest noise.
inline void encoder_integrity_y_test() {
    Serial.print("TEST encoder_integrity axis=Y target=");
    Serial.println(wiring_target_name());
    Serial.print("CS=");
    Serial.println(SENSOR_SPI_CS_Y);

    AS5048 sensor(SENSOR_SPI_CS_Y);
    if (!sensor.init()) {
        Serial.println("RESULT encoder_integrity_y FAIL init=0");
        while (true) {
            delay(1000);
        }
    }
    sensor.enable();

    Serial.println("REST — keep motor Y rotor still (~1 s noise capture at 500 Hz).");
    delay(300);
    const float rest_std = sensor_capture_rest_noise(sensor, L3_REST_SAMPLES);
    Serial.print("NOISE rest_std_rad=");
    Serial.println(rest_std, 6);

    sensor_print_diagnostics(sensor);
    Serial.println("SPEC AGC mid-range (not 0 or 255); mag healthy; ocf=cof=0 at rest.");
    wiring_wait_for_confirm("rest noise + diagnostics look healthy");

    Serial.println("ROTATE — turn motor Y rotor slowly one full mechanical revolution.");
    uint32_t parity_faults = 0;
    float min_angle = 0.0f;
    float max_angle = 0.0f;
    sensor_stream_rotation(sensor, "Y", parity_faults, min_angle, max_angle);

    Serial.println("Check serial/plot: angle sweeps 0..2pi, one clean wrap, monotonic in turn direction.");
    wiring_wait_for_confirm("full-turn sweep monotonic with clean single wrap");

    sensor.disable();
    sensor.deinit();

    Serial.print("RESULT encoder_integrity_y PASS rest_std_rad=");
    Serial.print(rest_std, 6);
    Serial.print(" parity_faults=");
    Serial.println(parity_faults);
}
