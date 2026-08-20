#pragma once

#include "../../utils/quick_defs.h"
#include <cmath>

class SCurve {

public:
    SCurve();
    ~SCurve() = default;

    void updatePosition(float dt, float& targetPos);
    void updateVelocity(float dt, float& targetVel);
    void updateAcceleration(float dt, float& targetAccel);

    void setNewDisplacement(float new_D);
    void resetTime() { t = 0.0f; }
    void setTimeToEnd() { t = tau; }
    float movementFinished() { return t >= tau; }

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

    float t = 0.0f;

    const float jmax = JERK_MAX;
    const float amax = ACCEL_MAX;
    const float vmax = VELOCITY_MAX;
    const float _1_2amax = 0.5f / amax;
    const float _1_2jmax = 0.5f / jmax;
    const float _1_2vmax = 0.5f / vmax;
    
    float J = 0.0f;
    float D = 0.0f;
    float tau = 0.0f;
    float tau_2 = 0.0f;
    float tau_3 = 0.0f;
};