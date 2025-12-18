#pragma once

#include "Arduino.h"
#include "../../motor/Motor.hpp"
#include "../../sensor/AS5048.hpp"
#include "../../pwm_driver/PWMDriverTeensy41.hpp"
#include "../../utils/tweakable_params.h"

void pid_test() {
    Serial.begin(115200);
    delay(3000);
    Serial.println("Starting PID Test...");
    delay(2000);

    AS5048 sensor = AS5048(SENSOR_SPI_CS);
    if (!sensor.init()) {
        Serial.println("Failed to initialize sensor");
        return;
    }
    PWMDriverTeensy41 pwm_driver = PWMDriverTeensy41(N1, N2, N3, EN);
    if (!pwm_driver.init()) {
        Serial.println("Failed to initialize driver");
        return;
    }
    Motor motor = Motor(sensor, pwm_driver);
    if (!motor.init()) {
        Serial.println("Failed to initialize motor");
        return;
    }
    
    unsigned long time = millis();
    while (true) {
        motor.loopFOC();
        motor.updateFOC();
        if (millis() - time > 100) {
            time = millis();
            Serial.print("Mechanical angle: " + String(motor.getMechanicalAngle(), 3));
            Serial.print("  |  Electrical angle: " + String(motor.getElectricalAngle()));
            Serial.print("  |  Angular velocity: " + String(motor.getAngularVelocity()));
            Serial.print("  |  Angular error: " + String(motor.getAngularError()));
            Serial.print("  |  Target velocity: " + String(motor.getTargetVelocity()));
            Serial.print("  |  Target q current: " + String(motor.getTargetQCurrent()));
            Serial.print("  |  Vq: " + String(motor.getVq()));
            Serial.println();
        }
    }
}