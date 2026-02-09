#include "LowPassFilter.hpp"

_LowPassFilter::_LowPassFilter(float _tau)
: tau(_tau)
{}

// Update filter with new measurement
float _LowPassFilter::operator()(float x, float dt) {
    float alpha = tau / (tau + dt);
    // Standard first-order IIR
    float y = alpha * y_prev + (1 - alpha) * x;
    y_prev = y;
    return y;
}
