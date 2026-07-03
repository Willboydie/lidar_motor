#pragma once

#include "../utils/constants.h"
#include "../utils/tweakable_params.h"
#include "Motor.hpp"
#include "Arduino.h"

#define GM2804_POLE_PAIRS 7
#define GM2804_PHASE_RESISTANCE 8.125f
#define GM2804_K_V 169.0f
#define GM2804_K_E 0.057f
#define GM2804_SENSOR_DIRECTION -1 // CCW = 1, CW = -1. Measured -1 by L9 pole-pair/dir (X slope -6.998, Y -7.006).

class GM2804 : public Motor {
public:
    GM2804(int _motor_number, float _electrical_offset)
    : Motor(_motor_number, _electrical_offset, GM2804_PHASE_RESISTANCE, GM2804_K_V, GM2804_K_E, GM2804_POLE_PAIRS, GM2804_SENSOR_DIRECTION)
    {}
};
