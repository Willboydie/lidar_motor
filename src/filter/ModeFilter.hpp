#pragma once

#include "../utils/constants.h"
#include "../utils/tweakable_params.h"
#include "../utils/quick_defs.h"
#include "Arduino.h"

class ModeFilter {
public:
    ModeFilter(uint32_t size); // size must be a power of 2
    ~ModeFilter() = default;

    uint16_t operator()(uint16_t reading);
    void reset();

private:
    static constexpr uint32_t SENSOR_RANGE = CPR;
    static constexpr uint32_t MAX_WINDOW = MAX_FILTER_WINDOW_SIZE;
    
    // Per-instance arrays (not static) to avoid cross-instance corruption
    uint8_t counts[SENSOR_RANGE];       
    uint8_t freq_bin[MAX_WINDOW + 1];  
    uint16_t history[MAX_WINDOW];       
    
    uint32_t head = 0;
    uint32_t window_mask;
    uint16_t current_mode = 0;
    uint8_t max_freq = 0;
};