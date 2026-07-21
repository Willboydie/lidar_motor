#pragma once

constexpr float _1_6 = 1.0f / 6.0f;
constexpr float _1_3 = 1.0f / 3.0f;

#include "SCurve.hpp"

void SCurve::updateTargets(float dt) {
    t += dt;
    if (t < tau) {
        targetPos = phase1_position(t);
        targetV = phase1_velocity(t);
        targetA = phase1_acceleration(t);
        return;
    } else if (t < 3 * tau) {
        targetPos = phases23_position(t);
        targetV = phases23_velocity(t);
        targetA = phases23_acceleration(t);
        return;
    } else {
        targetPos = phase4_position(t);
        targetV = phase4_velocity(t);
        targetA = phase4_acceleration(t);
        return;
    }
    Debug::msg("t is out of bounds: %f, tau = %f", t, tau);
    return;
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
    float t_minus_4_tau = t - 4.0 * tau;
    return J * (_1_6 * t_minus_4_tau*t_minus_4_tau*t_minus_4_tau + 2.0 * tau_3);
}

float SCurve::phase1_velocity(float t) {
    return 0.5 * J * t*t;
}

float SCurve::phases23_velocity(float t) {
    return J * (-0.5 * t*t + 2.0 * tau * t - tau_2);
}

float SCurve::phase4_velocity(float t) {
    float t_minus_4_tau = t - 4.0 * tau;
    return 0.5 * J * t_minus_4_tau*t_minus_4_tau;
}

float SCurve::phase1_acceleration(float t) {
    return J * t;
}

float SCurve::phases23_acceleration(float t) {
    return J * (-t + 2.0 * tau);
}

float SCurve::phase4_acceleration(float t) {
    return J * (t - 4.0 * tau);
}

void SCurve::update_params(float new_D) {
    D = new_D;
    tau = compute_tau();
    tau_2 = tau * tau;
    tau_3 = tau_2 * tau;
    t = 0.0f;
    J = D / (2.0 * tau_3);
}

float SCurve::compute_tau() {
    float tau_a = sqrtf(D * _1_2amax);
    float tau_j = _cuberoot(D * _1_2jmax);
    return max(max(tau_a, tau_j), eps);
}