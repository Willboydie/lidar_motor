#include "../../utils/constants.h"
#include "../../scan_path/ScanPath.hpp"
#include "Arduino.h"


void sine_path_test() {
    SinusoidalScanPath scanPath(500, 10, _PI_4, _PI_4);
    delay(1000);
    Serial.print("Theta Period: " + String(scanPath.theta_period));
    Serial.println(" | Phi Period: " + String(scanPath.phi_period));
    Serial.print("Theta limit: " + String(scanPath.theta_limit));
    Serial.println(" | Phi limit: " + String(scanPath.phi_limit));
    delay(5000);

    while (true) {
        delayMicroseconds(10);
        scanPath.update();

        static unsigned long start_time = millis();
        if (millis() - start_time > 200) {
            start_time = millis();
            Serial.print("Steps: " + String(scanPath.step_count));
            Serial.print(" | Theta: " + String(scanPath.theta, 3));
            Serial.print(" | Phi: " + String(scanPath.phi, 3));
            Serial.print(" | Theta Phase: " + String(scanPath.theta_phase));
            Serial.println(" | Phi Phase: " + String(scanPath.phi_phase));
        }
    }
}