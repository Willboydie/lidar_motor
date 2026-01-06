#pragma once

#include "../utils/constants.h"
#include "../utils/tweakable_params.h"
#include "../utils/quick_defs.h"
#include "../low_pass_filter/LowPassFilter.hpp"
#include "Arduino.h"

class Sensor {
public:
    
    virtual bool init() = 0;
    virtual void deinit() = 0; // TODO: implement this
    
    void enable();
    void disable();
    bool update();

    float getLatestAbsoluteRotorAngle();
    float getLatestRotorAngle();
    float getAngularVelocity();
    unsigned long getLatestTimestamp();

    void setSensorVelocityLPF(float _sensor_velocity_lpf) { angular_velocity_LPF.setTau(_sensor_velocity_lpf); }

private:

    virtual uint16_t getRawCount() = 0;

    _LowPassFilter angular_velocity_LPF = _LowPassFilter(ANGULAR_VELOCITY_LPF_TAU);

    int32_t absolute_rotor_angle = 0;
    int32_t rotor_angle0 = 0;
    int32_t rotor_angle1 = 0;
    int32_t angular_velocity = 0;
    unsigned long timestamp0 = 0;
    unsigned long timestamp1 = 0;
    int32_t cumulated_rotations = 0;

protected:
    bool enabled = false;
    bool initialised = false;
    bool first_measurement = true;
};