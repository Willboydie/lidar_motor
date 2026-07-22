#pragma once

#include "../debug/Debug.hpp"
#include "../utils/quick_defs.h"
#include <cmath>

class SCurve {

public:
    SCurve(float jerk_max, float accel_max);
    ~SCurve() = default;

    void updateParams(float new_D);
    void updateTime(float dt);
    void updatePosTarget(float& targetPos);
    void updateVeloAccelTarget(float& target_vel, float& target_accel);

private:
    float compute_tau();

    float phase1_position(float t);
    float phases23_position(float t);
    float phase4_position(float t);
    float phase1_velocity(float t);
    float phases23_velocity(float t);
    float phase4_velocity(float t);
    float phase1_acceleration(float t);
    float phases23_acceleration(float t);
    float phase4_acceleration(float t);

    float t;

    const float jmax;
    const float amax;
    const float _1_2amax = 0.5 / amax;
    const float _1_2jmax = 0.5 / jmax;
    
    float J = 0.0;
    float D = 0.0;
    float tau = 0.0;
    float tau_2 = 0.0;
    float tau_3 = 0.0;
};