#pragma once

#include "../utils/constants.h"
#include "../utils/tweakable_params.h"
#include "../low_pass_filter/LowPassFilter.hpp"
#include "Arduino.h"

class Sensor {
public:
    
    virtual bool init() = 0;
    virtual void deinit() = 0;
    void enable();
    void disable();
    bool _update();
    float getLatestAbsoluteRotorAngle();
    float getLatestRotorAngle();
    float getAngularVelocity();
    unsigned long getLatestTimestamp();

    void setSensorVelocityLPF(float _sensor_velocity_lpf) { angular_velocity_LPF.setTau(_sensor_velocity_lpf); }

private:

    virtual float getSensorAngle() = 0;

    _LowPassFilter angular_velocity_LPF = _LowPassFilter(ANGULAR_VELOCITY_LPF_TAU);

    float absolute_rotor_angle = 0.0f;
    float rotor_angle0 = 0.0f;
    float rotor_angle1 = 0.0f;
    float angular_velocity = 0.0f;
    unsigned long timestamp0 = 0;
    unsigned long timestamp1 = 0;
    int32_t cumulated_rotations = 0;

    unsigned long time_delta = 0;

    bool enabled = false;

};