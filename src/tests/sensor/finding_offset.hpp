#pragma once

#include "../../sensor/AS5048.hpp"
#include "../../pwm_driver/PWMDriverTeensy41.hpp"
#include "../../utils/constants.h"
#include <Arduino.h>
#include <cmath>

void sensor_offset_test() {
    AS5048 sensor = AS5048(SENSOR_SPI_CS);
    if (!sensor.init()) {
        Serial.println("Failed to initialize sensor");
        return;
    }
    Serial.println("Finding sensor offset...");
    PWMDriverTeensy41 driver = PWMDriverTeensy41(N1, N2, N3, EN);
    if (!driver.init()) {
        Serial.println("Failed to initialize driver");
        return;
    }
    driver.enable();
    driver.setPwm(ALIGNMENT_VOLTAGE, 0.0f, 0.0f); // Vq = ALIGNMENT_VOLTAGE, Vd = 0, angle_el = 0
    delay(1000);
    int sample_num = 0;
    float sample = 0.0f;
    float angle = 0.0f;
    unsigned long sample_time = millis();
    while (millis() - sample_time < 1000) {
        sensor._update();
        angle = sensor.getLatestAbsoluteRotorAngle();
        sample += angle;
        sample_num ++;
    }
    float sensor_offset = fmod((sample * POLE_PAIRS * SENSOR_DIRECTION) / (float)sample_num, _2PI);
    Serial.println("Sensor offset found: " + String(sensor_offset));

}