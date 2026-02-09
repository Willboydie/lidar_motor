#pragma once

#include "../utils/constants.h"
#include "../utils/tweakable_params.h"
#include "../utils/quick_defs.h"
#include "Arduino.h"

class NotchFilter {
public:
    NotchFilter(float centerFreq, float Q, float sampleRate);
    ~NotchFilter() = default;

    float operator()(float input);
    
    void setCenterFreq(float centerFreq);
    void setQ(float Q);
    void setSampleRate(float sampleRate);
    void reset();

private:
    void updateCoefficients();
    
    float centerFreq;
    float Q;
    float sampleRate;
    
    // Biquad coefficients (normalized, a0 = 1)
    float b0, b1, b2;  // Feedforward coefficients
    float a1, a2;      // Feedback coefficients
    
    // Delay line for biquad filter
    float x1 = 0.0f;   // x[n-1]
    float x2 = 0.0f;   // x[n-2]
    float y1 = 0.0f;   // y[n-1]
    float y2 = 0.0f;   // y[n-2]
};
