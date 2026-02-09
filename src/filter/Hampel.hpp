#pragma once

#include "Arduino.h"
#include "MedianFilter.hpp"

template <uint16_t WINDOW_SIZE, uint8_t BIT_RES>
class HampelFilter {
public:
    HampelFilter(float nSigma = 3.0f)
    : nSigma(nSigma) {
        medianFilter.reset();
        madFilter.reset();
    }

    float operator()(int32_t new_val) {
        // 1. Get rolling median
        float m = medianFilter(new_val);

        // 2. Compute absolute deviation
        int32_t abs_dev = abs(new_val - (int)m);

        // 3. Rolling MAD
        float mad = madFilter(abs_dev);

        // 4. Statistical test
        float sigma = 1.4826f * mad;

        if (sigma > 0 && abs_dev > nSigma * sigma)
            return m;          // replace outlier
        else
            return new_val;    // keep original
    }

    void reset() {
        medianFilter.reset();
        madFilter.reset();
    }

private:
    float nSigma;
    MedianFilter<WINDOW_SIZE, BIT_RES> medianFilter;
    MedianFilter<WINDOW_SIZE, BIT_RES> madFilter;
};