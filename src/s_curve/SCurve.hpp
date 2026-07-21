#pragma once

#include "Debug.hpp"
#include <cmath>

class SCurve {

public:
    SCurve(float jerk_max, float accel_max, float eps);
    ~SCurve() = default;

    void update_params(float new_D);
    void updateTargets(float dt);

    float getTargetPos() const { return targetPos; }
    float getTargetV() const { return targetV; }
    float getTargetA() const { return targetA; }

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

    float targetPos = 0.0;
    float targetV = 0.0;
    float targetA = 0.0;

    const float jmax;
    const float amax;
    const float eps;
    const float _1_2amax = 0.5 / amax;
    const float _1_2jmax = 0.5 / jmax;
    const float eps_2 = eps * eps;
    const float eps_3 = eps_2 * eps;

    float J = 0.0;
    float D = 0.0;
    float tau = 0.0;
    float tau_2 = 0.0;
    float tau_3 = 0.0;
};