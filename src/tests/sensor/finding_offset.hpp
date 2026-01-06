#pragma once

#include "../../sensor/AS5048.hpp"
#include "../../pwm_driver/PWMDriverTeensy41.hpp"
#include "../../motor/Motor.hpp"
#include "../../utils/constants.h"
#include "../../utils/tweakable_params.h"
#include "../../utils/quick_defs.h"
#include <Arduino.h>
#include <cmath>

void sensor_offset_test() {
    Serial.begin(115200);
    Serial.println("Initialising sensors and drivers...");
    delay(1000);
    AS5048 sensor1 = AS5048(SENSOR_SPI_CS_1);
    AS5048 sensor2 = AS5048(SENSOR_SPI_CS_2);
    PWMDriverTeensy41 pwmDriver1 = PWMDriverTeensy41(N1_1, N2_1, N3_1, EN_1);
    PWMDriverTeensy41 pwmDriver2 = PWMDriverTeensy41(N1_2, N2_2, N3_2, EN_2);
    Motor motor1(sensor1, pwmDriver1, 1);
    Motor motor2(sensor2, pwmDriver2, 2);

    motor1.init();
    motor2.init();

    float angle1 = 0.0f;
    float angle2 = 0.0f;
    float elec_offset1 = 0.0f;
    float elec_offset2 = 0.0f;

    float Id = MOTOR_CURRENT_LIMIT * 0.75f;
    float Vd = Id * PHASE_RESISTANCE;

    float sample_offset1 = 0.0f;
    float sample_offset2 = 0.0f;
    int count = 0;
    Serial.println("Setting alignment voltage to: " + String(Vd) + " V");
    Serial.println("Finding sensor offset...");
    motor1.setPhaseVoltage(0.0f, Vd, 0.0f); // Vq = 0, Vd = ALIGNMENT_VOLTAGE, angle_el = 0
    motor2.setPhaseVoltage(0.0f, Vd, 0.0f);
    delay(1000);
    while (true) {
        sensor1.update();
        sensor2.update();
        angle1 = sensor1.getLatestRotorAngle();
        angle2 = sensor2.getLatestRotorAngle();
        sample_offset1 += angle1;
        sample_offset2 += angle2;
        count++;
        static unsigned long time0 = millis();
        if (millis() - time0 > 500) {
            time0 = millis();
            elec_offset1 = normalise_angle((sample_offset1 * POLE_PAIRS * SENSOR_DIRECTION) / (float)count);
            elec_offset2 = normalise_angle((sample_offset2 * POLE_PAIRS * SENSOR_DIRECTION) / (float)count);
            sample_offset1 = 0.0f;
            sample_offset2 = 0.0f;
            count = 0;
            Serial.print("Angle 1: " + String(angle1, 4) + "   Elec Offset 1: " + String(elec_offset1, 4));
            Serial.print("   Angle 2: " + String(angle2, 4)+ "   Elec Offset 2: " + String(elec_offset2, 4));
            Serial.println();
        }

        if (Serial.available()) {
            Serial.read();
            Serial.println("Halted by user.");
            motor1.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            motor2.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            return;
        }
    }
}