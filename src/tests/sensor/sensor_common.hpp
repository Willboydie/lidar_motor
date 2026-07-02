#pragma once

#include <Arduino.h>
#include <math.h>
#include "../../sensor/AS5048.hpp"
#include "../../utils/constants.h"
#include "../../utils/quick_defs.h"
#include "../wiring/wiring_common.hpp"

static constexpr uint32_t L3_REST_SAMPLES = 500;
static constexpr uint32_t L3_SAMPLE_PERIOD_US = 2000;  // 500 Hz DBG stream
static constexpr uint32_t L3_ROTATION_STREAM_MS = 15000;

inline uint16_t sensor_angle_to_count(float angle_rad) {
    return static_cast<uint16_t>(normalise_angle(angle_rad) / _2PI * static_cast<float>(CPR) + 0.5f);
}

inline float sensor_std_dev(const float* samples, uint32_t n) {
    if (n < 2) {
        return 0.0f;
    }
    float mean = 0.0f;
    for (uint32_t i = 0; i < n; i++) {
        mean += samples[i];
    }
    mean /= static_cast<float>(n);

    float var = 0.0f;
    for (uint32_t i = 0; i < n; i++) {
        const float d = samples[i] - mean;
        var += d * d;
    }
    return sqrtf(var / static_cast<float>(n));
}

inline void sensor_print_dbg(
    uint16_t count,
    float angle_rad,
    uint16_t agc,
    uint16_t mag,
    uint16_t ocf,
    uint16_t cof) {

    Serial.print("DBG ");
    Serial.print(count);
    Serial.print(",");
    Serial.print(angle_rad, 6);
    Serial.print(",");
    Serial.print(agc);
    Serial.print(",");
    Serial.print(mag);
    Serial.print(",");
    Serial.print(ocf);
    Serial.print(",");
    Serial.println(cof);
}

inline void sensor_print_diagnostics(AS5048& sensor) {
    sensor.updateDiagnostics();
    Serial.print("DIAG agc=");
    Serial.print(sensor.getAGCValue());
    Serial.print(" mag=");
    Serial.print(sensor.getSensorMagnitude());
    Serial.print(" ocf=");
    Serial.print(sensor.getOCFValue());
    Serial.print(" cof=");
    Serial.print(sensor.getCOFValue());
    Serial.print(" compLow=");
    Serial.print(sensor.getCompLowValue());
    Serial.print(" compHigh=");
    Serial.println(sensor.getCompHighValue());
}

inline float sensor_capture_rest_noise(AS5048& sensor, uint32_t n_samples) {
    float samples[L3_REST_SAMPLES];
    if (n_samples > L3_REST_SAMPLES) {
        n_samples = L3_REST_SAMPLES;
    }

    const uint32_t t_start = micros();
    for (uint32_t i = 0; i < n_samples; i++) {
        sensor.update();
        sensor.updateDiagnostics();
        const float angle = sensor.getLatestRotorAngle();
        samples[i] = angle;
        sensor_print_dbg(
            sensor_angle_to_count(angle),
            angle,
            sensor.getAGCValue(),
            sensor.getSensorMagnitude(),
            sensor.getOCFValue(),
            sensor.getCOFValue());

        const uint32_t elapsed = micros() - t_start;
        const uint32_t target = (i + 1) * L3_SAMPLE_PERIOD_US;
        if (elapsed < target) {
            delayMicroseconds(target - elapsed);
        }
    }

    return sensor_std_dev(samples, n_samples);
}

inline void sensor_stream_rotation(
    AS5048& sensor,
    const char* axis,
    uint32_t& parity_faults,
    float& min_angle_rad,
    float& max_angle_rad) {

    parity_faults = 0;
    min_angle_rad = _2PI;
    max_angle_rad = 0.0f;

    const uint32_t stream_start_ms = millis();
    uint32_t sample_idx = 0;

    while ((millis() - stream_start_ms) < L3_ROTATION_STREAM_MS) {
        sensor.update();
        sensor.updateDiagnostics();

        const float angle = sensor.getLatestRotorAngle();
        if (angle < min_angle_rad) {
            min_angle_rad = angle;
        }
        if (angle > max_angle_rad) {
            max_angle_rad = angle;
        }

        if (sensor.getOCFValue() || sensor.getCOFValue()) {
            parity_faults++;
        }

        sensor_print_dbg(
            sensor_angle_to_count(angle),
            angle,
            sensor.getAGCValue(),
            sensor.getSensorMagnitude(),
            sensor.getOCFValue(),
            sensor.getCOFValue());

        sample_idx++;
        const uint32_t elapsed = millis() - stream_start_ms;
        const uint32_t target_ms = (sample_idx * L3_SAMPLE_PERIOD_US) / 1000;
        if (elapsed < target_ms) {
            delay(target_ms - elapsed);
        }
    }

    Serial.print("STREAM axis=");
    Serial.print(axis);
    Serial.print(" samples=");
    Serial.print(sample_idx);
    Serial.print(" span_rad=");
    Serial.print(max_angle_rad - min_angle_rad, 4);
    Serial.print(" parity_faults=");
    Serial.println(parity_faults);
}
