#pragma once

#include "../utils/quick_defs.h"
#include "../utils/tweakable_params.h"

// By constructing members of this class using different parameters, any PID controller can be created.

class PIDControl {
public:
    PIDControl(float p, float i, float d, float _output_ramp, float _lower_limit, float _upper_limit);
    ~PIDControl() = default;
    float operator() (float error);
    void reset();
    
    void setP(float _p) { P = _p; }
    void setI(float _i) { I = _i; }
    void setD(float _d) { D = _d; }
    void setRamp(float _ramp) { output_ramp = _ramp; }

    float P;
    float I;
    float D;
    float output_ramp; // maximum speed of change of the output value
    float upper_limit; // max output value from controller
    float lower_limit;

protected:
    float error_prev;
    float output_prev;
    float integral_prev;
    unsigned long timestamp_prev;
};
