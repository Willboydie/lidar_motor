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
    timestamp_prev = timestamp_now;

    // tau <= 0 means "no filtering": identity pass-through with NO latched state.
    // Critically, this avoids alpha = 0/(0+0) = NaN when two calls land in the same
    // micros() tick, and avoids latching a transient NaN forever (0 * NaN == NaN).
    if (tau <= 0.0f) {
        y_prev = x;
        return x;
    }
    // Non-positive dt (same tick or micros() rollover): hold last output, no divide.
    if (Ts <= 0.0f) {
        return y_prev;
    }
    float alpha = tau / (tau + Ts);
    // Standard first-order IIR
    float y = alpha * y_prev + (1 - alpha) * x;
    y_prev = y;
    return y;
}

void _LowPassFilter::reset() {
    y_prev = 0.0f;
    timestamp_prev = micros();
}
