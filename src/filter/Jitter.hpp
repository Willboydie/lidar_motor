#pragma once

#include "../utils/constants.h"
#include "../utils/tweakable_params.h"
#include "Arduino.h"

class Jitter {
public:
    Jitter(uint16_t _amplitude, uint16_t _period = 1);
    ~Jitter() = default;

    uint16_t operator()(uint16_t reading);

private:
    const uint16_t amplitude;
    const uint16_t period;
    uint16_t increment = 0;
};