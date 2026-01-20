#pragma once

#include <Arduino.h>
#include "../utils/constants.h"
#include "../utils/tweakable_params.h"


class Motor {
public:
    Motor(int _motor_number, float _electrical_offset, float _phase_resistance, float _K_v, float _K_e, float _pole_pairs, float _sensor_direction)
    : motor_number(_motor_number)
    , electrical_offset(_electrical_offset)
    , phase_resistance(_phase_resistance)
    , K_v(_K_v)
    , K_e(_K_e)
    , pole_pairs(_pole_pairs)
    , sensor_direction(_sensor_direction)
    {}
    
    int motor_number;
    float electrical_offset; // hardcoded offset for loaded motor.
    float phase_resistance;
    float K_v;
    float K_e;
    float pole_pairs;
    float sensor_direction;
};