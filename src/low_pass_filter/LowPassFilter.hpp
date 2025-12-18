#pragma once

#include "../utils/constants.h"
#include "../utils/tweakable_params.h"
#include "Arduino.h"

class _LowPassFilter {
public:
    _LowPassFilter(float _tau);
    ~_LowPassFilter() = default;

    float operator() (float x);
    float tau; // LPF time constant (search "First Order Backward Discretisation of the Canonical LPF ODE" for more details)

protected:
    unsigned long timestamp_prev;  //!< Last execution timestamp
    float y_prev; //!< filtered value in previous execution step 
};
