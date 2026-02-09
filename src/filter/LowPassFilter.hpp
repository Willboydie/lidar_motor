#pragma once

#include "../utils/constants.h"
#include "../utils/tweakable_params.h"
#include "Arduino.h"

class _LowPassFilter {
public:
    _LowPassFilter(float _tau);
    ~_LowPassFilter() = default;

    float operator() (float x);
    void reset();

    void setTau(float _tau) { tau = _tau; }

protected:
    float y_prev = 0.0f; //!< filtered value in previous execution step 
    float tau; // LPF time constant (search "First Order Backward Discretisation of the Canonical LPF ODE" for more details)
    unsigned long timestamp_prev = 0;
};
