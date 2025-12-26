#pragma once

#include "../../ldm/GarminV3HP.hpp"
#include "Arduino.h"
#include "../../utils/tweakable_params.h"

void ldm_test() {
    Serial.begin(115200);
    delay(3000);
    Serial.println("Starting LDM Test (interrupt mode)...");
    delay(1000);

    GarminV3HP ldm = GarminV3HP(GARMIN_SCL, GARMIN_SDA, GARMIN_MODE_PIN);
    ldm.init();
    ldm.enable();
    Serial.println("LDM initialised. Enabled status = " + String(ldm.isEnabled()));
    
    // Trigger the first measurement
    ldm.triggerMeasurement();

    while (true) {
        // Check if measurement is ready (non-blocking)
        if (ldm.isDataReady()) {
            // unsigned long start_time = micros();
            ldm.readDistance();           // Read the result
            ldm.triggerMeasurement();     // Start the next measurement
            // unsigned long end_time = micros();
            // Serial.println("Total duration: " + String(end_time - start_time) + " microseconds");
        }
        
        // Print status periodically
        static unsigned long time0 = millis();
        if (millis() - time0 > 200) {
            time0 = millis();
            Serial.print("Distance: " + String(ldm.getLatestDistance()) + " cm");
            Serial.print("  |  Timestamp: " + String(ldm.getLatestTimestamp()) + " microseconds");
            Serial.println();
        }
    }
}
