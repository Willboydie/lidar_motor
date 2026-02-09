#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(float _Qa) 
: Qa(_Qa) {}

void KalmanFilter::update(float reading, float dt) {

    float dt2 = dt * dt;
    float dt3 = dt2 * dt;

    // ---------- Predict state ----------
    float x_p = x + v * dt + 0.5f * a * dt2;
    float v_p = v + a * dt;
    float a_p = a;

    // ---------- Predict covariance ----------
    float P00_p = P00 
        + dt * (P01 + P01) 
        + dt2 * (P11 + 0.5f * (P02 + P02)) 
        + 0.5f * dt3 * (P12 + P12) 
        + 0.25f * dt2 * dt2 * P22;

    float P01_p = P01 
        + dt * (P11 + P02) 
        + 0.5f * dt2 * (P12 + P12) 
        + 0.5f * dt3 * P22;

    float P02_p = P02 + dt * P12 + 0.5f * dt2 * P22;
    float P11_p = P11 + dt * (P12 + P12) + dt2 * P22;
    float P12_p = P12 + dt * P22;
    float P22_p = P22 + Qa;

    // ---------- Update ----------
    float residual = reading - x_p;
    float S = P00_p + R;
    float invS = 1.0f / S;

    float K0 = P00_p * invS;
    float K1 = P01_p * invS;
    float K2 = P02_p * invS;

    x = x_p + K0 * residual;
    v = v_p + K1 * residual;
    a = a_p + K2 * residual;

    // ---------- Update covariance ----------
    P00 = (1.0f - K0) * P00_p;
    P01 = (1.0f - K0) * P01_p;
    P02 = (1.0f - K0) * P02_p;
    P11 = P11_p - K1 * P01_p;
    P12 = P12_p - K1 * P02_p;
    P22 = P22_p - K2 * P02_p;
}
