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

protected:
    static constexpr uint16_t window_size = FILTER_WINDOW_SIZE;
    uint16_t head = 0;

    int32_t int_sum = 0;
    int32_t int_buffer[window_size] = {0};  // Instance-specific buffer, initialized to zeros
    
    float float_sum = 0.0f;
    float float_buffer[window_size] = {0.0f};
    static constexpr float one_over_window_size = 1.0f / static_cast<float>(window_size);
};