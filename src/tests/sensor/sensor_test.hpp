#pragma once

#include "../../sensor/AS5048.hpp"
#include "../../pwm_driver/PWMDriverTeensy41.hpp"
#include "../../motor/GM2804.hpp"
#include "../../foc_controller/FOCController.hpp"
#include "../../utils/constants.h"
#include "../../utils/tweakable_params.h"
#include "Arduino.h"

void sensor_test() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting sensor test...");
    delay(1000);
    AS5048 sensor1 = AS5048(SENSOR_SPI_CS_1);
    AS5048 sensor2 = AS5048(SENSOR_SPI_CS_2);
    PWMDriverTeensy41 pwmDriver1 = PWMDriverTeensy41(N1_1, N2_1, N3_1, EN_1);
    PWMDriverTeensy41 pwmDriver2 = PWMDriverTeensy41(N1_2, N2_2, N3_2, EN_2);
    GM2804 motor1(1, SENSOR_OFFSET_1);
    GM2804 motor2(1, SENSOR_OFFSET_2);
    FOCController focController1(sensor1, pwmDriver1, motor1);
    FOCController focController2(sensor2, pwmDriver2, motor2);
    
    if (!focController1.init()) {
        Serial.println("Failed to initialize foc controller1");
        return;
    }
    if (!focController2.init()) {
        Serial.println("Failed to initialize foc controller2");
        return;
    }
    delay(1000);
    Serial.println("Sensor test started");
    while (true) {
        static unsigned long time1 = micros();
        if (micros() - time1 >= SENSOR_UPDATE_DT_US) {
            time1 = micros();
            if (!sensor1.update()) {
                Serial.println("Failed to update sensor1");
                return;
            }
            if (!sensor2.update()) {
                Serial.println("Failed to update sensor2");
                return;
            }
        }   

        // AS5048 sensor = sensor1;

        static unsigned long time0 = millis();
        if (millis() - time0 > 100) {
            time0 = millis();

            float filtered_angle = sensor1.getLatestRotorAngle();
            float velocity = sensor1.getAngularVelocity();

            Serial.print("Raw Angle: " + String(sensor1.getRawAngle(), 4));
            if (filtered_angle > 0.0f) {
                Serial.print(" |  Filtered Angle:  " + String(filtered_angle, 3) + " rad");
            } else {
                Serial.print(" |  Filtered Angle: " + String(filtered_angle, 3) + " rad");
            }
            if (velocity > 0.0f) {
                Serial.print(" |  Angular Velocity:  " + String(velocity, 3) + " rad/s");
            } else {
                Serial.print(" |  Angular Velocity: " + String(velocity, 3) + " rad/s");
            }

            Serial.print(" |  Cumulated Rotations: " + String(sensor1.getCumulatedRotations()));
            // Serial.print(" |  Angle Change: " + String(sensor2.getAngleChange()));
            // Serial.print(" |  Acceleration: " + String(sensor2.getAcceleration(), 4) + " rad/s^2");
            // Serial.print(" |  Sensor Magnitude: " + String(sensor1.getSensorMagnitude()));
            // Serial.print(" |  AGC Value: " + String(sensor1.getAGCValue()));
            // Serial.print(" |  OCF Value: " + String(sensor1.getOCFValue()));
            // Serial.print(" |  COF Value: " + String(sensor1.getCOFValue()));
            // Serial.print(" |  Comp Low Value: " + String(sensor1.getCompLowValue()));
            // Serial.print(" |  Comp High Value: " + String(sensor1.getCompHighValue()));
            Serial.println();
        }
    }
}