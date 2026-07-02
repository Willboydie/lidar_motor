#pragma once

#include <Arduino.h>
#include "../../utils/constants.h"
#include "../../utils/tweakable_params.h"

// L0.2 — Tight-loop iteration rate and micros() delta statistics for 1 s.
// Returns true when the loop rate exceeds the minimum headroom gate.
inline bool loop_timing_test() {
#if defined(STM32G4xx)
    constexpr const char* kTarget = "g4";
#elif defined(ARDUINO_TEENSY41)
    constexpr const char* kTarget = "teensy";
#else
    constexpr const char* kTarget = "unknown";
#endif

    Serial.println();
    Serial.print("TEST loop_timing axis=NA target=");
    Serial.println(kTarget);

    // Reference rate implied by the sensor update period constant (for logging only).
    const float sensor_ref_hz = 1.0e6f / static_cast<float>(SENSOR_UPDATE_DT_US);

    // Minimum acceptable empty-loop rate. Well above practical FOC loop needs.
    constexpr float kMinLoopRateHz = 10000.0f;

    const uint32_t start_ms = millis();
    uint32_t iterations = 0;
    uint32_t prev_us = micros();
    uint32_t min_dt_us = UINT32_MAX;
    uint32_t max_dt_us = 0;
    uint64_t sum_dt_us = 0;

    while (millis() - start_ms < 1000) {
        const uint32_t now_us = micros();
        const uint32_t dt_us = now_us - prev_us;
        prev_us = now_us;

        if (dt_us < min_dt_us) min_dt_us = dt_us;
        if (dt_us > max_dt_us) max_dt_us = dt_us;
        sum_dt_us += dt_us;
        ++iterations;
    }

    const float loop_rate_hz = static_cast<float>(iterations);
    const float mean_dt_us = static_cast<float>(sum_dt_us) / static_cast<float>(iterations);

    Serial.print("LOOP iterations=");
    Serial.print(iterations);
    Serial.print(" rate_hz=");
    Serial.print(loop_rate_hz, 1);
    Serial.print(" mean_dt_us=");
    Serial.print(mean_dt_us, 2);
    Serial.print(" min_dt_us=");
    Serial.print(min_dt_us);
    Serial.print(" max_dt_us=");
    Serial.print(max_dt_us);
    Serial.print(" sensor_ref_hz=");
    Serial.println(sensor_ref_hz, 1);

    const bool rate_ok = loop_rate_hz >= kMinLoopRateHz;
    Serial.print("RESULT loop_timing ");
    Serial.print(rate_ok ? "PASS" : "FAIL");
    Serial.print(" rate_hz=");
    Serial.println(loop_rate_hz, 1);

    return rate_ok;
}
