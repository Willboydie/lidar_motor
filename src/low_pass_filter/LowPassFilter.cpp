#include "LowPassFilter.hpp"

_LowPassFilter::_LowPassFilter(float _tau)
    : tau(_tau)
    , y_prev(0.0f)
{
    timestamp_prev = micros();
}


float  _LowPassFilter::operator() (float x) {
    unsigned long timestamp = micros();
    float dt = (timestamp - timestamp_prev) * 1e-6f;

    if (dt < 0.0f ) dt = 1e-3f; // overflow protection
    else if (dt > 0.3f) {
        y_prev = x;
        timestamp_prev = timestamp;
        return x;
    }

    float alpha = tau / (tau + dt);
    float y = alpha * y_prev + (1.0f - alpha) * x;
    y_prev = y;
    timestamp_prev = timestamp;
    return y;
}