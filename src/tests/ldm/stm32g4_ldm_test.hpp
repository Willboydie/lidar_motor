#pragma once

#include "../../ldm/GarminV3HP.hpp"
#include "../../utils/constants.h"
#include "Arduino.h"

// Stage 2 bench test: Garmin V3HP over I2C on STM32G4, no motor stack.
// Logs every sample at the data-ready interrupt rate and prints 1 Hz stats
// (count, rate, mean distance) for bench pass/fail checks.
inline void stm32g4_ldm_test() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        delay(10);
    }
    delay(500);
    Serial.println("stm32g4 ldm test starting...");

    GarminV3HP ldm(GARMIN_SCL, GARMIN_SDA, GARMIN_MODE_PIN);
    ldm.init();
    ldm.enable();
    Serial.println("LDM enabled. Streaming samples (cm) at data-ready rate.");

    uint32_t sample_count = 0;
    uint32_t window_count = 0;
    uint64_t window_sum_cm = 0;
    unsigned long window_start_ms = millis();
    unsigned long test_start_ms = window_start_ms;

    while (true) {
        if (ldm.isDataReady()) {
            ldm.readDistance();
            ldm.triggerMeasurement();

            const uint16_t distance_cm = ldm.getLatestDistance();
            sample_count++;
            window_count++;
            window_sum_cm += distance_cm;

            Serial.print(millis() - test_start_ms);
            Serial.print(" ms  ");
            if (distance_cm == 0) {
                Serial.println("NO_RETURN");
            } else {
                Serial.print(distance_cm);
                Serial.println(" cm");
            }
        }

        const unsigned long now = millis();
        if (now - window_start_ms >= 1000UL) {
            const float rate_hz = (1000.0f * window_count) / (now - window_start_ms);
            const float mean_cm = window_count > 0
                ? static_cast<float>(window_sum_cm) / window_count
                : 0.0f;

            Serial.print("--- stats: samples=");
            Serial.print(sample_count);
            Serial.print(" rate=");
            Serial.print(rate_hz, 1);
            Serial.print(" Hz mean=");
            Serial.print(mean_cm / 100.0f, 3);
            Serial.println(" m ---");

            window_start_ms = now;
            window_count = 0;
            window_sum_cm = 0;
        }
    }
}
