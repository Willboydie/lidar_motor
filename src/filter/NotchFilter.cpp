#include "NotchFilter.hpp"

NotchFilter::NotchFilter(float _centerFreq, float _Q, float _sampleRate)
    : centerFreq(_centerFreq), Q(_Q), sampleRate(_sampleRate)
{
    updateCoefficients();
}

float NotchFilter::operator()(float input) {
    // Biquad difference equation: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    
    // Update delay line
    x2 = x1;
    x1 = input;
    y2 = y1;
    y1 = output;
    
    return output;
}

void NotchFilter::setCenterFreq(float _centerFreq) {
    centerFreq = _centerFreq;
    updateCoefficients();
}

void NotchFilter::setQ(float _Q) {
    Q = _Q;
    updateCoefficients();
}

void NotchFilter::setSampleRate(float _sampleRate) {
    sampleRate = _sampleRate;
    updateCoefficients();
}

void NotchFilter::reset() {
    x1 = 0.0f;
    x2 = 0.0f;
    y1 = 0.0f;
    y2 = 0.0f;
}

void NotchFilter::updateCoefficients() {
    // Normalized frequency
    float w0 = _2PI * centerFreq / sampleRate;
    
    // Precompute trigonometric values
    float cos_w0 = cosf(w0);
    float sin_w0 = sinf(w0);
    
    // Bandwidth parameter
    float alpha = sin_w0 / (2.0f * Q);
    
    // Denominator coefficient (for normalization)
    float a0 = 1.0f + alpha;
    
    // Feedforward coefficients (numerator)
    b0 = 1.0f / a0;
    b1 = -2.0f * cos_w0 / a0;
    b2 = 1.0f / a0;
    
    // Feedback coefficients (denominator, normalized so a0 = 1)
    a1 = -2.0f * cos_w0 / a0;
    a2 = (1.0f - alpha) / a0;
}
