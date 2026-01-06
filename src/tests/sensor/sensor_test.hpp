#pragma once

#include "../../sensor/Sensor.hpp"
#include "../../sensor/AS5048.hpp"
#include "Arduino.h"
#include "../../utils/Constants.h"

void sensor_test() {
    Serial.begin(115200);
    AS5048 sensor1 = AS5048(SENSOR_SPI_CS_1);
    AS5048 sensor2 = AS5048(SENSOR_SPI_CS_2);
    sensor1.init();
    sensor2.init();
    while (true) {
        delayMicroseconds(100);
        sensor1.update();
        sensor2.update();
        static unsigned long time0 = millis();
        if (millis() - time0 > 500) {
            time0 = millis();
            Serial.print("1: Rotor Angle: " + String(sensor1.getLatestRotorAngle()) + " units");
            Serial.print("  |  1: Angular Velocity: " + String(sensor1.getAngularVelocity()) + " units/s");
            Serial.print("  |  2: Rotor Angle: " + String(sensor2.getLatestRotorAngle()) + " units");
            Serial.print("  |  2: Angular Velocity: " + String(sensor2.getAngularVelocity()) + " units/s");
            Serial.println();
        }
    }
}