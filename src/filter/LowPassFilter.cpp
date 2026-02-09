#include "LowPassFilter.hpp"

_LowPassFilter::_LowPassFilter(float _tau)
: tau(_tau)
{
    reset();
}

// Update filter with new measurement
float _LowPassFilter::operator()(float x) {
    unsigned long timestamp_now = micros();
    float Ts = static_cast<float>(timestamp_now - timestamp_prev) * 1e-6f;
    if (Ts < 0.0f) {
        timestamp_prev = timestamp_now;
        return y_prev;
    }
    float alpha = tau / (tau + Ts);
    // Standard first-order IIR
    float y = alpha * y_prev + (1 - alpha) * x;
    y_prev = y;
    timestamp_prev = timestamp_now;
    return y;
}

void _LowPassFilter::reset() {
    y_prev = 0.0f;
    timestamp_prev = micros();
}
