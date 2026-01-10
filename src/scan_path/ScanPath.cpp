#include "ScanPath.hpp"


void ScanPath::update() {
    step_count++;

    if ( step_count % theta_period == 0 ) {
        theta_phase = (theta_phase + 1) % num_phases;
        theta = calcTargetTheta();
    }

    if ( step_count % phi_period == 0 ) {
        phi_phase = (phi_phase + 1) % num_phases;
        phi = calcTargetPhi();
    }
}

float ScanPath::calcTargetTheta() { return static_cast<float>(lookUpTheta()) * theta_limit / 32767.0f;  }
float ScanPath::calcTargetPhi() { return static_cast<float>(lookUpPhi()) * phi_limit / 32767.0f;  }

int16_t SinusoidalScanPath::lookUpTheta() { return sin_q15_u16(theta_phase); }
int16_t SinusoidalScanPath::lookUpPhi() { return sin_q15_u16(phi_phase); }