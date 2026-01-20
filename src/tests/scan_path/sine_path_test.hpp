#include "../../utils/constants.h"
#include "../../scan_path/ScanPath.hpp"
#include "Arduino.h"


void sine_path_test() {
    SinusoidalScanPath scanPath(750019, 1500007, _PI_4, _PI_4, 0.0f, 0.0f);
    delay(1000);
    Serial.print("Theta Phase Inc: " + String(scanPath.theta_phase_inc));
    Serial.print(" | Phi Phase Inc: " + String(scanPath.phi_phase_inc));
    Serial.print(" | Theta Amplitude: " + String(scanPath.theta_amplitude));
    Serial.print(" | Phi Amplitude: " + String(scanPath.phi_amplitude));
    Serial.println(" | Theta Centre: " + String(scanPath.theta_centre));
    Serial.print("Phi Centre: " + String(scanPath.phi_centre));
    Serial.print(" | Theta Phase: " + String(scanPath.theta_phase));
    Serial.print(" | Phi Phase: " + String(scanPath.phi_phase));
    Serial.print(" | Theta: " + String(scanPath.theta, 3));
    Serial.println(" | Phi: " + String(scanPath.phi, 3));
    delay(2000);

    while (true) {
        delayMicroseconds(10);
        scanPath.update();

        static unsigned long start_time = millis();
        if (millis() - start_time > 200) {
            start_time = millis();
            Serial.print("Theta: " + String(scanPath.theta, 3));
            Serial.print(" | Phi: " + String(scanPath.phi, 3));
            Serial.print(" | Theta Phase: " + String(scanPath.theta_phase));
            Serial.println(" | Phi Phase: " + String(scanPath.phi_phase));
        }
    }
}