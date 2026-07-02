#pragma once

#include <Arduino.h>
#include "../../utils/constants.h"

// L0.1 — Serial alive, F_CPU report, micros() accuracy self-check.
// Returns true when the micros self-check is within tolerance.
inline bool mcu_alive_test() {
#if defined(STM32G4xx)
    constexpr const char* kTarget = "g4";
#elif defined(ARDUINO_TEENSY41)
    constexpr const char* kTarget = "teensy";
#else
    constexpr const char* kTarget = "unknown";
#endif

    Serial.begin(BAUD_RATE);
    while (!Serial && millis() < 3000) {
        delay(10);
    }

    Serial.println();
    Serial.print("TEST mcu_alive axis=NA target=");
    Serial.println(kTarget);
    Serial.print("F_CPU ");
    Serial.println(F_CPU);

    // Busy-wait micros() self-check: expect ~10 ms per trial.
    constexpr uint32_t kExpectedUs = 10000;
    constexpr uint8_t kTrials = 10;
    constexpr float kMaxErrorPct = 1.0f;

    uint32_t sum_us = 0;
    uint32_t min_us = UINT32_MAX;
    uint32_t max_us = 0;

    for (uint8_t i = 0; i < kTrials; ++i) {
        const uint32_t t0 = micros();
        const uint32_t deadline = t0 + kExpectedUs;
        while (static_cast<int32_t>(micros() - deadline) < 0) {
            // spin
        }
        const uint32_t elapsed = micros() - t0;
        sum_us += elapsed;
        if (elapsed < min_us) min_us = elapsed;
        if (elapsed > max_us) max_us = elapsed;
    }

    const float mean_us = static_cast<float>(sum_us) / static_cast<float>(kTrials);
    const float error_pct = 100.0f * (mean_us - static_cast<float>(kExpectedUs))
                            / static_cast<float>(kExpectedUs);

    Serial.print("MICROS expected_us=");
    Serial.print(kExpectedUs);
    Serial.print(" mean_us=");
    Serial.print(mean_us, 1);
    Serial.print(" min_us=");
    Serial.print(min_us);
    Serial.print(" max_us=");
    Serial.print(max_us);
    Serial.print(" error_pct=");
    Serial.println(error_pct, 3);

    const bool micros_ok = fabsf(error_pct) <= kMaxErrorPct;
    Serial.print("RESULT mcu_alive ");
    Serial.print(micros_ok ? "PASS" : "FAIL");
    Serial.print(" micros_error_pct=");
    Serial.println(error_pct, 3);

    return micros_ok;
}
