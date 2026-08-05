constexpr float _1_6 = 1.0f / 6.0f;
constexpr float _1_3 = 1.0f / 3.0f;

#include "SCurve.hpp"

SCurve::SCurve() = default;


void SCurve::updatePosition(float dt, float& targetPos) {
    t += dt;
    if (t < tau) {
        targetPos = phase1_position(t);
        return;
    } else if (t < 3 * tau) {
        targetPos = phases23_position(t);
        return;
    } else if (t < 4 * tau) {
        targetPos = phase4_position(t);
        return;
    } else {
        return;
    }
}


void SCurve::updateVelocity(float dt, float& targetVel) {
    t += dt;
    if (t < tau) {
        targetVel = phase1_velocity(t);
        return;
    } else if (t < 3 * tau) {
        targetVel = phases23_velocity(t);
        return;
    } else if (t < 4 * tau) {
        targetVel = phase4_velocity(t);
        return;
    } else {
        targetVel = 0.0f;
        return;
    }
}

void SCurve::updateAcceleration(float dt, float& targetAccel) {
    t += dt;

    if (t < tau) {
        targetAccel = phase1_acceleration(t);
        return;
    } else if (t < 3 * tau) {
        targetAccel = phases23_acceleration(t);
        return;
    } else if (t < 4 * tau) {
        targetAccel = phase4_acceleration(t);
        return;
    } else {
        targetAccel = 0.0f;
        return;
    }
}


float SCurve::phase1_position(float t) {
    return _1_6 * J * t*t*t;
}

float SCurve::phases23_position(float t) {
    return (
        J * (-_1_6 * t*t*t
    + tau * t*t
    - tau_2 * t
    + _1_3 * tau_3)
    );
}

float SCurve::phase4_position(float t) {
    float t_minus_4_tau = t - 4.0f * tau;
    return J * (_1_6 * t_minus_4_tau*t_minus_4_tau*t_minus_4_tau + 2.0f * tau_3);
}

float SCurve::phase1_velocity(float t) {
    return 0.5f * J * t*t;
}

float SCurve::phases23_velocity(float t) {
    return J * (-0.5f * t*t + 2.0f * tau * t - tau_2);
}

float SCurve::phase4_velocity(float t) {
    float t_minus_4_tau = t - 4.0f * tau;
    return 0.5f * J * t_minus_4_tau*t_minus_4_tau;
}

float SCurve::phase1_acceleration(float t) {
    return J * t;
}

float SCurve::phases23_acceleration(float t) {
    return J * (-t + 2.0f * tau);
}

float SCurve::phase4_acceleration(float t) {
    return J * (t - 4.0f * tau);
}

void SCurve::setNewDisplacement(float new_D) {
    t = 0.0f;
    D = new_D;
    tau = compute_tau();
    tau_2 = tau * tau;
    tau_3 = tau_2 * tau; 
    J = D / (2.0f * tau_3);
}

float SCurve::compute_tau() {
    float tau_a = sqrtf(fabsf(D) * _1_2amax);
    float tau_j = _cuberoot(fabsf(D) * _1_2jmax);
    return fmaxf(tau_a, tau_j);
}
