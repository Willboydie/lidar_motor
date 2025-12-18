#pragma once

#include "../../sensor/Sensor.hpp"
#include "../../sensor/AS5048.hpp"
#include "Arduino.h"
#include "../../utils/Constants.h"

void sensor_test() {
    Serial.begin(115200);
    AS5048 sensor(SENSOR_SPI_CS);
    sensor.init();
    while (true) {
        delayMicroseconds(100);
        sensor._update();
        static unsigned long time0 = millis();
        if (millis() - time0 > 100) {
            time0 = millis();
            Serial.print("Cumulative Rotor Angle: " + String(sensor.getLatestRotorAngle(), 3) + " Radians");
            Serial.print("  |  Angular Velocity: " + String(sensor.getAngularVelocity()) + " Radians/s");
            Serial.print("  |  Timestamp: " + String(sensor.getLatestTimestamp()) + " Microseconds");
            Serial.println();
        }
    }
}