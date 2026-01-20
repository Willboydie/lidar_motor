#pragma once

#include <cstdint>
#include "../utils/quick_defs.h"
#include "Arduino.h"
#include "../utils/tweakable_params.h"
#include "sin_lut_q15.hpp"

class ScanPath {
public:
    ScanPath(uint32_t _theta_phase_inc, uint32_t _phi_phase_inc, float _theta_amplitude, float _phi_amplitude, float _theta_centre, float _phi_centre) 
    : theta_phase_inc( _theta_phase_inc )
    , phi_phase_inc( _phi_phase_inc )
    , theta_amplitude(_theta_amplitude)
    , phi_amplitude(_phi_amplitude)
    , theta_centre(_theta_centre)
    , phi_centre(_phi_centre)
    {
        if (theta_phase_inc < 1) theta_phase_inc = 1;
        if (phi_phase_inc < 1) phi_phase_inc = 1;
        if (theta_amplitude < 0.0f) theta_amplitude = 0.0f;
        if (phi_amplitude < 0.0f) phi_amplitude = 0.0f;
     }
    virtual ~ScanPath() = default;
    
    virtual int16_t lookUpTheta() = 0;
    virtual int16_t lookUpPhi() = 0;

    uint32_t theta_phase_inc; // controls frequency
    uint32_t phi_phase_inc;
    uint32_t theta_phase = 3ULL * UINT32_MAX / 4ULL; // theta = sin(theta_phase)  --- starts at -limit
    uint32_t phi_phase = 0;

    float theta_amplitude;
    float phi_amplitude;

    float theta_centre;
    float phi_centre;

    float calcTargetTheta();
    float calcTargetPhi();

    void update();
    
    // Setters for scan path parameters
    void setThetaPhaseInc(uint32_t value) { theta_phase_inc = value; }
    void setPhiPhaseInc(uint32_t value) { phi_phase_inc = value; }
    void setThetaAmplitude(float value) { theta_amplitude = value; }
    void setPhiAmplitude(float value) { phi_amplitude = value; }
    void setThetaCentre(float value) { theta_centre = value; }
    void setPhiCentre(float value) { phi_centre = value; }
    
    float theta = 0.0f;
    float phi = 0.0f;
};


class SinusoidalScanPath : public ScanPath {
public:
    SinusoidalScanPath(uint32_t _theta_phase_inc, uint32_t _phi_phase_inc, float _theta_amplitude, float _phi_amplitude, float _theta_centre, float _phi_centre)
    : ScanPath(
        _theta_phase_inc, 
        _phi_phase_inc, 
        _theta_amplitude, 
        _phi_amplitude, 
        _theta_centre, 
        _phi_centre
    )
    {}

protected:
    int16_t lookUpPhi() override;
    int16_t lookUpTheta() override;
};