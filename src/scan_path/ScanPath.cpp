#include "ScanPath.hpp"


void ScanPath::update() {
    static uint32_t next_tick_us = micros();

    uint32_t now = micros();

    // Catch up if we are behind (hard RT approximation)
    while ((int32_t)(now - next_tick_us) >= 0) {

        // Advance phases deterministically
        theta_phase = (theta_phase + theta_phase_inc);
        phi_phase = (phi_phase + phi_phase_inc);

        // Schedule next tick
        next_tick_us += SCAN_PATH_UPDATE_PERIOD_US;
    }

    // Compute targets
    theta = calcTargetTheta();
    phi   = calcTargetPhi();
}

                                                                                // 0.000030518509476f = 1 / 32767
float ScanPath::calcTargetTheta() { return static_cast<float>(lookUpTheta()) * theta_amplitude * 0.000030518509476f + theta_centre; }
float ScanPath::calcTargetPhi() { return static_cast<float>(lookUpPhi()) * phi_amplitude * 0.000030518509476f + phi_centre; }

int16_t SinusoidalScanPath::lookUpTheta() { return sin_q15_u32(theta_phase); }
int16_t SinusoidalScanPath::lookUpPhi() { return sin_q15_u32(phi_phase); }