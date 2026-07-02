#pragma once

#include <Arduino.h>
#include <math.h>
#include "../../filter/LowPassFilter.hpp"
#include "../../filter/NotchFilter.hpp"
#include "../../utils/tweakable_params.h"

static constexpr uint32_t FILTER_STEP_PERIOD_US = 1000;   // 1 kHz synthetic stream
static constexpr uint32_t FILTER_STEP_DURATION_MS = 1000;
static constexpr uint32_t FILTER_SINE_PERIOD_US = 1000;
static constexpr uint32_t FILTER_SINE_CYCLES = 8;

inline float filter_rms(const float* samples, uint32_t n) {
    if (n == 0) {
        return 0.0f;
    }
    float sum_sq = 0.0f;
    for (uint32_t i = 0; i < n; i++) {
        sum_sq += samples[i] * samples[i];
    }
    return sqrtf(sum_sq / static_cast<float>(n));
}

inline void filter_print_params() {
    Serial.print("PARAM ANGLE_LOW_PASS_FILTER_TAU=");
    Serial.println(ANGLE_LOW_PASS_FILTER_TAU, 4);
    Serial.print("PARAM VELOCITY_LOW_PASS_FILTER_TAU=");
    Serial.println(VELOCITY_LOW_PASS_FILTER_TAU, 4);
    Serial.print("PARAM FOC_VOLTAGE_LPF_TAU=");
    Serial.println(FOC_VOLTAGE_LPF_TAU, 4);
    Serial.print("PARAM NOTCH_FILTER_FREQ=");
    Serial.println(NOTCH_FILTER_FREQ, 1);
    Serial.print("PARAM NOTCH_FILTER_Q=");
    Serial.println(NOTCH_FILTER_Q, 1);
    Serial.print("PARAM NOTCH_FILTER_SAMPLE_RATE=");
    Serial.println(NOTCH_FILTER_SAMPLE_RATE, 1);
}

// Step input 0 -> amplitude; stream FLT step,<label>,<t_s>,<input>,<output>.
inline float filter_step_response(const char* label, _LowPassFilter& lpf, float tau_s, float amplitude) {
    lpf.reset();

    Serial.print("FLT step label=");
    Serial.println(label);

    const uint32_t start_ms = millis();
    uint32_t sample_idx = 0;
    const uint32_t t0_us = micros();
    float settle_time_s = -1.0f;
    const float target_63 = 0.632f * amplitude;

    while ((millis() - start_ms) < FILTER_STEP_DURATION_MS) {
        const float input = amplitude;
        const float output = lpf(input);
        const float t_s = static_cast<float>(sample_idx) * static_cast<float>(FILTER_STEP_PERIOD_US) * 1e-6f;

        Serial.print("FLT step,");
        Serial.print(label);
        Serial.print(",");
        Serial.print(t_s, 6);
        Serial.print(",");
        Serial.print(input, 6);
        Serial.print(",");
        Serial.println(output, 6);

        if (settle_time_s < 0.0f && output >= target_63) {
            settle_time_s = t_s;
        }

        sample_idx++;
        const uint32_t elapsed = micros() - t0_us;
        const uint32_t target_us = sample_idx * FILTER_STEP_PERIOD_US;
        if (elapsed < target_us) {
            delayMicroseconds(target_us - elapsed);
        }
    }

    Serial.print("STAT step label=");
    Serial.print(label);
    Serial.print(" tau_s=");
    Serial.print(tau_s, 4);
    Serial.print(" t63_s=");
    Serial.println(settle_time_s, 4);

    return settle_time_s;
}

// Sinusoid through LPF; stream FLT sine,<label>,<t_s>,<input>,<output>.
inline void filter_sine_sweep(const char* label, _LowPassFilter& lpf, float tau_s, float freq_hz, float amplitude) {
    lpf.reset();

    Serial.print("FLT sine label=");
    Serial.print(label);
    Serial.print(" freq_hz=");
    Serial.println(freq_hz, 2);

    const uint32_t period_us = FILTER_SINE_PERIOD_US;
    const float dt_s = static_cast<float>(period_us) * 1e-6f;
    const uint32_t samples_per_cycle = static_cast<uint32_t>(1.0f / (freq_hz * dt_s) + 0.5f);
    const uint32_t total_samples = samples_per_cycle * FILTER_SINE_CYCLES;
    const uint32_t skip_samples = samples_per_cycle;  // skip first cycle for transient

    float in_buf[256];
    float out_buf[256];
    uint32_t capture_n = 0;

    const uint32_t t0_us = micros();
    for (uint32_t i = 0; i < total_samples; i++) {
        const float t_s = static_cast<float>(i) * dt_s;
        const float input = amplitude * sinf(_2PI * freq_hz * t_s);
        const float output = lpf(input);

        Serial.print("FLT sine,");
        Serial.print(label);
        Serial.print(",");
        Serial.print(t_s, 6);
        Serial.print(",");
        Serial.print(input, 6);
        Serial.print(",");
        Serial.println(output, 6);

        if (i >= skip_samples && capture_n < 256) {
            in_buf[capture_n] = input;
            out_buf[capture_n] = output;
            capture_n++;
        }

        const uint32_t elapsed = micros() - t0_us;
        const uint32_t target_us = (i + 1) * period_us;
        if (elapsed < target_us) {
            delayMicroseconds(target_us - elapsed);
        }
    }

    const float in_rms = filter_rms(in_buf, capture_n);
    const float out_rms = filter_rms(out_buf, capture_n);
    const float gain = (in_rms > 1e-6f) ? (out_rms / in_rms) : 0.0f;

    Serial.print("STAT sine label=");
    Serial.print(label);
    Serial.print(" freq_hz=");
    Serial.print(freq_hz, 2);
    Serial.print(" tau_s=");
    Serial.print(tau_s, 4);
    Serial.print(" in_rms=");
    Serial.print(in_rms, 6);
    Serial.print(" out_rms=");
    Serial.print(out_rms, 6);
    Serial.print(" gain=");
    Serial.println(gain, 4);
}

// Notch at center and passband frequencies; stream FLT notch,<freq_hz>,<t_s>,<input>,<output>.
inline float filter_notch_at_freq(float test_freq_hz) {
    NotchFilter notch(NOTCH_FILTER_FREQ, NOTCH_FILTER_Q, NOTCH_FILTER_SAMPLE_RATE);
    notch.reset();

    const uint32_t period_us = static_cast<uint32_t>(1e6f / NOTCH_FILTER_SAMPLE_RATE + 0.5f);
    const float dt_s = static_cast<float>(period_us) * 1e-6f;
    const uint32_t samples_per_cycle = static_cast<uint32_t>(1.0f / (test_freq_hz * dt_s) + 0.5f);
    const uint32_t total_cycles = 6;
    const uint32_t total_samples = samples_per_cycle * total_cycles;
    const uint32_t skip_samples = samples_per_cycle * 2;

    float in_buf[512];
    float out_buf[512];
    uint32_t capture_n = 0;

    Serial.print("FLT notch freq_hz=");
    Serial.println(test_freq_hz, 2);

    const uint32_t t0_us = micros();
    for (uint32_t i = 0; i < total_samples; i++) {
        const float t_s = static_cast<float>(i) * dt_s;
        const float input = sinf(_2PI * test_freq_hz * t_s);
        const float output = notch(input);

        Serial.print("FLT notch,");
        Serial.print(test_freq_hz, 2);
        Serial.print(",");
        Serial.print(t_s, 6);
        Serial.print(",");
        Serial.print(input, 6);
        Serial.print(",");
        Serial.println(output, 6);

        if (i >= skip_samples && capture_n < 512) {
            in_buf[capture_n] = input;
            out_buf[capture_n] = output;
            capture_n++;
        }

        const uint32_t elapsed = micros() - t0_us;
        const uint32_t target_us = (i + 1) * period_us;
        if (elapsed < target_us) {
            delayMicroseconds(target_us - elapsed);
        }
    }

    const float in_rms = filter_rms(in_buf, capture_n);
    const float out_rms = filter_rms(out_buf, capture_n);
    const float attenuation_db = (in_rms > 1e-6f && out_rms > 1e-9f)
        ? (20.0f * log10f(out_rms / in_rms))
        : -120.0f;

    Serial.print("STAT notch freq_hz=");
    Serial.print(test_freq_hz, 2);
    Serial.print(" in_rms=");
    Serial.print(in_rms, 6);
    Serial.print(" out_rms=");
    Serial.print(out_rms, 6);
    Serial.print(" attenuation_db=");
    Serial.println(attenuation_db, 2);

    return attenuation_db;
}

inline void filter_run_characterization() {
    filter_print_params();

    _LowPassFilter vel_lpf(VELOCITY_LOW_PASS_FILTER_TAU);
    _LowPassFilter angle_lpf(ANGLE_LOW_PASS_FILTER_TAU);
    _LowPassFilter voltage_lpf(FOC_VOLTAGE_LPF_TAU);

    Serial.println("--- LPF step responses (input 0 -> 1) ---");
    const float vel_t63 = filter_step_response("velocity", vel_lpf, VELOCITY_LOW_PASS_FILTER_TAU, 1.0f);
    const float angle_t63 = filter_step_response("angle", angle_lpf, ANGLE_LOW_PASS_FILTER_TAU, 1.0f);
    const float voltage_t63 = filter_step_response("voltage", voltage_lpf, FOC_VOLTAGE_LPF_TAU, 1.0f);

    Serial.println("--- LPF sine sweeps (1 Hz, amplitude 1) ---");
    filter_sine_sweep("velocity", vel_lpf, VELOCITY_LOW_PASS_FILTER_TAU, 1.0f, 1.0f);

    Serial.println("--- Notch filter (center vs passband) ---");
    const float notch_at_center_db = filter_notch_at_freq(NOTCH_FILTER_FREQ);
    const float notch_pass_db = filter_notch_at_freq(100.0f);

    Serial.print("SUMMARY vel_t63_s=");
    Serial.print(vel_t63, 4);
    Serial.print(" angle_t63_s=");
    Serial.print(angle_t63, 4);
    Serial.print(" voltage_t63_s=");
    Serial.print(voltage_t63, 4);
    Serial.print(" notch_center_db=");
    Serial.print(notch_at_center_db, 2);
    Serial.print(" notch_pass_db=");
    Serial.println(notch_pass_db, 2);
}
