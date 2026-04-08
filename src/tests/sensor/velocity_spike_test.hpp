#pragma once

#include "../../sensor/AS5048.hpp"
#include "../../utils/constants.h"
#include "../../utils/tweakable_params.h"
#include "Arduino.h"

static void printDebugTable(AS5048& sensor) {
    const float* data = sensor.getDebugData();
    uint32_t count = sensor.getDebugCount();

    Serial.println();
    Serial.println("  #  |    Velocity   |      dt       |  Rotor Angle  |  Prev Angle  |  Filtered Velocity");
    Serial.println("-----|---------------|---------------|---------------|--------------|----------------");

    for (uint32_t i = 0; i < count; i++) {
        uint32_t base = i * 5;
        
        float dt          = data[base + 0];
        float rotor_angle = data[base + 1];
        float prev_angle  = data[base + 2];
        float velocity    = data[base + 3];
        float filtered_velocity = data[base + 4];

        char line[120];
        snprintf(line, sizeof(line),
                 "%4lu | %+12.5f | %13.7f | %+12.5f | %+12.5f | %+12.5f",
                 (unsigned long)(i + 1), velocity, dt, rotor_angle, prev_angle, filtered_velocity);
        Serial.println(line);
    }

    Serial.println("-----|---------------|---------------|---------------|---------------|---------------");
    Serial.println();
}

void velocity_spike_test() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== Velocity Spike Test ===");
    Serial.println("Initialising sensor...");

    AS5048 sensor = AS5048(SENSOR_SPI_CS_1);
    if (!sensor.init()) {
        Serial.println("Failed to initialise sensor");
        return;
    }
    Serial.println("Sensor initialised.");

    while (true) {
        Serial.println("Send any character to enable and begin capture...");
        while (!Serial.available()) { delay(10); }
        while (Serial.available()) { Serial.read(); }

        sensor.clearDebugData();
        sensor.enable();
        Serial.println("Enabled. Capturing " + String(Sensor::DEBUG_N) + " velocity samples...");

        while (!sensor.isDebugCaptureComplete()) {
            static unsigned long t = micros();
            if (micros() - t >= SENSOR_UPDATE_DT_US) {
                t = micros();
                if (!sensor.update()) {
                    Serial.println("Sensor update failed");
                    return;
                }
            }
        }

        sensor.disable();
        Serial.println("Capture complete. Disabling sensor.");

        printDebugTable(sensor);
    }
}
