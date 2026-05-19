#pragma once

#include "../../sensor/AS5048.hpp"
#include "../../utils/constants.h"
#include "../../utils/tweakable_params.h"
#include "Arduino.h"

// Stage 1 bench test: AS5048 on STM32G4, no motor/PWM stack.
inline void stm32g4_sensor_test() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        delay(10);
    }
    delay(500);
    Serial.println("stm32g4 sensor test starting...");

    AS5048 sensor1(SENSOR_SPI_CS_X);
    AS5048 sensor2(SENSOR_SPI_CS_Y);

    if (!sensor1.init()) {
        Serial.println("Failed to init sensor X");
        return;
    }
    if (!sensor2.init()) {
        Serial.println("Failed to init sensor Y");
        return;
    }

    sensor1.enable();
    sensor2.enable();
    Serial.println("Sensors ready. Logging at 10 Hz.");

    while (true) {
        static unsigned long update_time = micros();
        if (micros() - update_time >= SENSOR_UPDATE_DT_US) {
            update_time = micros();
            if (!sensor1.update()) {
                Serial.println("sensor1 update failed");
                return;
            }
            if (!sensor2.update()) {
                Serial.println("sensor2 update failed");
                return;
            }
        }

        static unsigned long log_time = millis();
        if (millis() - log_time >= 100) {
            log_time = millis();

            const float angle1 = sensor1.getLatestRotorAngle();
            const float velocity1 = sensor1.getFilteredAngularVelocity();
            const float angle2 = sensor2.getLatestRotorAngle();
            const float velocity2 = sensor2.getFilteredAngularVelocity();

            Serial.print("X Angle: ");
            Serial.print(angle1, 4);
            Serial.print(" rad | Y Angle: ");
            Serial.print(angle2, 3);
            Serial.print(" rad | X Velocity: ");
            Serial.print(velocity1, 3);
            Serial.print(" rad/s | Y Velocity: ");
            Serial.print(velocity2, 3);
            Serial.println(" rad/s");
        }
    }
}
