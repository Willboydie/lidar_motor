#pragma once

#include "../utils/constants.h"
#include "../utils/tweakable_params.h"
#include "../utils/quick_defs.h"


class MeanFilter {
public:
    MeanFilter();
    ~MeanFilter() = default;

    int32_t operator()(int32_t reading);
    float operator()(float reading);

    void reset();

protected:
    const uint16_t window_size = DEFAULT_ANGLE_ERROR_MEAN_WINDOW_LENGTH;
    uint16_t head = 0;

    int32_t int_sum = 0;
    int32_t int_buffer[DEFAULT_ANGLE_ERROR_MEAN_WINDOW_LENGTH] = {0};  // Instance-specific buffer, initialized to zeros
    
    float float_sum = 0.0f;
    float float_buffer[DEFAULT_ANGLE_ERROR_MEAN_WINDOW_LENGTH] = {0.0f};
    const float one_over_window_size = 1.0f / static_cast<float>(DEFAULT_ANGLE_ERROR_MEAN_WINDOW_LENGTH);
};