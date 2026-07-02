#pragma once

#include <Arduino.h>
#include "../../pwm_driver/PlatformPWMDriver.hpp"
#include "../wiring/wiring_common.hpp"

static constexpr uint32_t L5_PWM_DWELL_MS = 2000;
static constexpr uint32_t L5_CENTER_HOLD_MS = 5000;
static constexpr float L5_MID_DUTY = 0.5f;
static constexpr float L5_DUTIES[] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
static constexpr size_t L5_DUTY_COUNT = sizeof(L5_DUTIES) / sizeof(L5_DUTIES[0]);

inline void pwm_serial_begin() {
    wiring_serial_begin();
}

inline float pwm_duty_to_voltage(float duty) {
    return duty * VOLTAGE_POWER_SUPPLY;
}

inline void pwm_print_spec() {
    Serial.print("SPEC pwm_frequency_hz=");
    Serial.print(MOTOR_PWM_FREQUENCY);
    Serial.print(" voltage_limit_V=");
    Serial.print(MOTOR_VOLTAGE_LIMIT, 1);
    Serial.print(" vps_V=");
    Serial.println(VOLTAGE_POWER_SUPPLY, 1);
    Serial.println("SPEC alignment=center-aligned (up-down counting)");
}

inline void pwm_bench_prompt() {
    Serial.println();
    Serial.println("SAFETY — L5 PWM signal integrity:");
    Serial.println("  VM ON (12 V). Bench supply current limit ~1.0 A.");
    Serial.println("  Oscilloscope on DRV8313 phase outputs (IN1/IN2/IN3).");
    Serial.println("  Other axis driver EN must stay LOW.");
    Serial.println("  E-stop: cut VM or pull EN low if anything looks wrong.");
    pwm_print_spec();
    wiring_wait_for_confirm("VM on, scope ready, supply current-limited");
}

inline void pwm_set_duties(PlatformPWMDriver& driver, float da, float db, float dc) {
    driver.setPwm(pwm_duty_to_voltage(da), pwm_duty_to_voltage(db), pwm_duty_to_voltage(dc));
}

inline void pwm_channel_sweep(PlatformPWMDriver& driver, const char* axis) {
    const char* channel_names[] = {"N1", "N2", "N3"};

    for (int ch = 0; ch < 3; ++ch) {
        for (size_t i = 0; i < L5_DUTY_COUNT; ++i) {
            const float duty = L5_DUTIES[i];
            float duties[3] = {0.0f, 0.0f, 0.0f};
            duties[ch] = duty;
            pwm_set_duties(driver, duties[0], duties[1], duties[2]);
            Serial.print("PWM ");
            Serial.print(channel_names[ch]);
            Serial.print("=");
            Serial.print(duty, 2);
            Serial.print(" axis=");
            Serial.print(axis);
            Serial.print(" dwell_ms=");
            Serial.println(L5_PWM_DWELL_MS);
            delay(L5_PWM_DWELL_MS);
        }
    }
}

inline void pwm_center_aligned_hold(PlatformPWMDriver& driver, const char* axis) {
    pwm_set_duties(driver, L5_MID_DUTY, L5_MID_DUTY, L5_MID_DUTY);
    Serial.print("PWM N1=");
    Serial.print(L5_MID_DUTY, 2);
    Serial.print(" N2=");
    Serial.print(L5_MID_DUTY, 2);
    Serial.print(" N3=");
    Serial.print(L5_MID_DUTY, 2);
    Serial.print(" axis=");
    Serial.print(axis);
    Serial.print(" hold_ms=");
    Serial.println(L5_CENTER_HOLD_MS);
    Serial.println("SCOPE — all three phases: confirm 25 kHz center-aligned PWM.");
    delay(L5_CENTER_HOLD_MS);
    wiring_wait_for_confirm("scope: 25 kHz center-aligned on N1/N2/N3");
}

inline void pwm_disable_verify(PlatformPWMDriver& driver) {
    driver.disable();
    Serial.println("DISABLE — duties zeroed, EN driven LOW.");
    wiring_wait_for_confirm("scope: all three phase outputs cut immediately after disable");
}
