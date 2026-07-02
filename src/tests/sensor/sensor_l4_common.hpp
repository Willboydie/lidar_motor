#pragma once

#include <Arduino.h>
#include <math.h>
#include "../../sensor/AS5048.hpp"
#include "../../utils/tweakable_params.h"
#include "sensor_common.hpp"

static constexpr uint32_t L4_STREAM_PERIOD_US = 100;       // 10 kHz DBG stream
static constexpr uint32_t L4_ROTATE_PHASE_MS = 12000;
static constexpr uint32_t L4_WRAP_PHASE_MS = 12000;
static constexpr float L4_WRAP_SPIKE_THRESH_RAD_S = 80.0f;

struct VelocityStreamStats {
    float sum_raw = 0.0f;
    float sum_filt = 0.0f;
    uint32_t n = 0;
    float max_abs_raw = 0.0f;
    uint32_t spike_count = 0;
};

inline void sensor_print_vel_dbg(float angle_rad, float raw_vel, float filt_vel) {
    Serial.print("DBG ");
    Serial.print(angle_rad, 6);
    Serial.print(",");
    Serial.print(raw_vel, 6);
    Serial.print(",");
    Serial.println(filt_vel, 6);
}

inline void sensor_print_l4_params() {
    Serial.print("PARAM VELOCITY_ESTIMATION_DT_US=");
    Serial.println(VELOCITY_ESTIMATION_DT_US);
    Serial.print("PARAM VELOCITY_LOW_PASS_FILTER_TAU=");
    Serial.println(VELOCITY_LOW_PASS_FILTER_TAU, 4);
    Serial.print("PARAM SENSOR_DIRECTION=");
    Serial.println(SENSOR_DIRECTION);
    Serial.print("PARAM stream_hz=");
    Serial.println(1e6f / static_cast<float>(L4_STREAM_PERIOD_US), 1);
}

inline VelocityStreamStats sensor_stream_velocity_phase(
    AS5048& sensor,
    uint32_t duration_ms,
    float spike_thresh_rad_s) {

    VelocityStreamStats stats;
    const uint32_t stream_start_ms = millis();
    uint32_t sample_idx = 0;
    const uint32_t t_start_us = micros();

    while ((millis() - stream_start_ms) < duration_ms) {
        sensor.update();

        const float angle = sensor.getLatestRotorAngle();
        const float raw = sensor.getRawAngularVelocity();
        const float filt = sensor.getFilteredAngularVelocity();

        sensor_print_vel_dbg(angle, raw, filt);

        stats.sum_raw += raw;
        stats.sum_filt += filt;
        stats.n++;

        const float abs_raw = fabsf(raw);
        if (abs_raw > stats.max_abs_raw) {
            stats.max_abs_raw = abs_raw;
        }
        if (abs_raw > spike_thresh_rad_s) {
            stats.spike_count++;
        }

        sample_idx++;
        const uint32_t elapsed = micros() - t_start_us;
        const uint32_t target_us = sample_idx * L4_STREAM_PERIOD_US;
        if (elapsed < target_us) {
            delayMicroseconds(target_us - elapsed);
        }
    }

    return stats;
}

inline void sensor_print_velocity_stats(const char* phase, const VelocityStreamStats& stats) {
    const float mean_raw = (stats.n > 0) ? (stats.sum_raw / static_cast<float>(stats.n)) : 0.0f;
    const float mean_filt = (stats.n > 0) ? (stats.sum_filt / static_cast<float>(stats.n)) : 0.0f;

    Serial.print("STAT phase=");
    Serial.print(phase);
    Serial.print(" samples=");
    Serial.print(stats.n);
    Serial.print(" mean_raw_rad_s=");
    Serial.print(mean_raw, 4);
    Serial.print(" mean_filt_rad_s=");
    Serial.print(mean_filt, 4);
    Serial.print(" max_abs_raw_rad_s=");
    Serial.print(stats.max_abs_raw, 4);
    Serial.print(" spike_count=");
    Serial.print(stats.spike_count);
    Serial.print(" spike_thresh_rad_s=");
    Serial.println(L4_WRAP_SPIKE_THRESH_RAD_S, 1);
}
