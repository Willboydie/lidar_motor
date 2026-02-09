#include "AlphaBetaFilter.hpp"

static constexpr float _2PI_SQUARED = _2PI * _2PI;

AlphaBetaFilter::AlphaBetaFilter(float _bandwidth_hz) : bandwidth_hz(_bandwidth_hz) {}

void AlphaBetaFilter::update(float reading, float dt) {

    // find alpha and beta - parameterisation from continuous-time design
    float alpha = _4PI * bandwidth_hz * dt;
    float beta = _2PI_SQUARED * bandwidth_hz * bandwidth_hz * dt * dt;

    // Predict next position based on current position and velocity
    float x_prediction = x + x_dot * dt;

    // Calculate residual (error between measurement and prediction)
    float residual = reading - x_prediction;
    
    // Update position estimate
    x = x_prediction + alpha * residual;
    
    // Update velocity estimate
    x_dot += beta * residual / dt;
}