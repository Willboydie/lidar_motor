#pragma once

#include "../utils/constants.h"
#include "../utils/tweakable_params.h"
#include "../utils/quick_defs.h"


class AlphaBetaFilter {
public:
    AlphaBetaFilter(float _bandwidth_hz);

    ~AlphaBetaFilter() = default;

    void update(float reading, float dt);
    float getVelocity() { return x_dot; }
    float getPosition() { return x; }

    void setBandwidth(float _bandwidth_hz) { bandwidth_hz = _bandwidth_hz; }
    void setFirstMeasurement(float _first_measurement) { x = _first_measurement; } // to be used on first measurement only!

protected:
    float bandwidth_hz;
    float x = 0.0f;
    float x_dot = 0.0f;
};