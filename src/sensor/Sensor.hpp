#pragma once

#include "../utils/constants.h"
#include "../utils/tweakable_params.h"
#include "../utils/quick_defs.h"
#include "../filter/LowPassFilter.hpp"
#include "../filter/MeanFilter.hpp"
#include "../filter/NotchFilter.hpp"
#include "Arduino.h"

class Sensor {
public:
    Sensor();
    virtual ~Sensor() = default;
    
    bool init();
    void deinit();
    void enable();
    void disable();
    bool update();
    virtual void updateDiagnostics() = 0;

    bool isInitialised() { return initialised; }
    bool isEnabled() { return enabled; }

    float getLatestRotorAngle() { return raw_sensor_angle; }
    float getRawAngularVelocity() { return raw_angular_velocity; }
    float getFilteredAngularVelocity() { return filtered_angular_velocity; }

    static constexpr uint32_t DEBUG_N = 100;
    const float* getDebugData() const { return debug_data; }
    uint32_t getDebugCount() const { return debug_idx; }
    bool isDebugCaptureComplete() const { return debug_idx >= DEBUG_N; }
    void clearDebugData();

    uint16_t getSensorMagnitude() { return magnitude; }
    uint16_t getAGCValue() { return agc; }
    uint16_t getOCFValue() { return ocf; }
    uint16_t getCOFValue() { return cof; }
    uint16_t getCompLowValue() { return compLow; }
    uint16_t getCompHighValue() { return compHigh; }

    void setVelocityLPFTau(float _tau) { velocityLPF.setTau(_tau); }
    void setAngleLPFTau(float _tau) { angleLPF.setTau(_tau); }
    float getVelocityLPFTau() const { return velocityLPF.getTau(); }
    float getAngleLPFTau() const { return angleLPF.getTau(); }

private:
    virtual bool hardwareInit() = 0;
    virtual void hardwareDeinit() = 0;
    virtual float getSensorAngle() = 0;

    float raw_sensor_angle = 0.0f;
    float previous_raw_angle = 0.0f;
    float raw_angular_velocity = 0.0f;
    float filtered_angular_velocity = 0.0f;
    uint32_t t0 = 0;

    float debug_data[DEBUG_N * 5] = {};
    uint32_t debug_idx = 0;

    _LowPassFilter velocityLPF{VELOCITY_LOW_PASS_FILTER_TAU};
    _LowPassFilter angleLPF{ANGLE_LOW_PASS_FILTER_TAU};

    MeanFilter meanFilter;

protected:
    bool enabled = false;
    bool initialised = false;

    // Diagnostics 
    uint16_t magnitude = 0;
    uint16_t agc = 0;
    uint16_t ocf = 0;
    uint16_t cof = 0;
    uint16_t compLow = 0;
    uint16_t compHigh = 0;
};