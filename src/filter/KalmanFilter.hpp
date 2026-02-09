#pragma once

#include "../utils/constants.h"
#include "../utils/tweakable_params.h"
#include "../utils/quick_defs.h"

class KalmanFilter {
public:
    KalmanFilter(float _Qa);

    ~KalmanFilter() = default;

    void update(float reading, float dt);

    float getVelocity() { return v; }
    float getPosition() { return x; }
    float getAcceleration() { return a; }

    void setQa(float _Qa){ Qa = _Qa; }

    void setFirstMeasurement(float _first_measurement) { 
        x = _first_measurement; 
        v = 0.0f; 
        a = 0.0f;
    }

protected:
    // State
    float x = 0.0f;   // position
    float v = 0.0f;   // velocity
    float a = 0.0f;   // acceleration

    // Covariance matrix
    float P00 = 1, P01 = 0, P02 = 0;
    float P11 = 1, P12 = 0;
    float P22 = 1;

    // Tunables
    static constexpr float R = DEFAULT_KALMAN_R;
    float Qa;   // acceleration process noise
};