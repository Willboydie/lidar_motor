#include "MovingLissa.hpp"
#include "../../debug/Debug.hpp"


MovingLissa::MovingLissa(float _x_frequency, float _y_frequency, float _x_amplitude, float _y_amplitude, float _x_phase, float _y_phase)
    : x_frequency(_x_frequency)
    , y_frequency(_y_frequency)
    , x_amplitude(_x_amplitude)
    , y_amplitude(_y_amplitude)
    , x_phase(_x_phase)
    , y_phase(_y_phase) {}

// ------------------------------------------------------------------------------------------

void MovingLissa::updateXPosition(float dt, float& x_pos) {
    t += dt;
    x_pos = x_amplitude * fast_sin(x_frequency * t + x_phase);
}

void MovingLissa::updateYPosition(float dt, float& y_pos) {
    t += dt;
    y_pos = y_amplitude * fast_sin(y_frequency * t + y_phase);
}

void MovingLissa::updateXVelocity(float dt, float& x_vel) {
    t += dt;
    x_vel = x_amplitude * x_frequency * fast_cos(x_frequency * t + x_phase);
}

void MovingLissa::updateYVelocity(float dt, float& y_vel) {
    t += dt;
    y_vel = y_amplitude * y_frequency * fast_cos(y_frequency * t + y_phase);
}

void MovingLissa::setXAmplitude(float _x_amplitude) {
    if (xAmplitudeWithinLimits(_x_amplitude)) {
        x_amplitude = _x_amplitude;
    }
}

void MovingLissa::setYAmplitude(float _y_amplitude) {
    if (yAmplitudeWithinLimits(_y_amplitude)) {
        y_amplitude = _y_amplitude;
    }
}

bool MovingLissa::xAmplitudeWithinLimits(float amplitude) const {
    if (amplitude < 0.0f || amplitude > X_SCAN_AMPLITUDE_LIMIT) {
        Debug::msg("X amplitude cannot be negative or greater than X_SCAN_AMPLITUDE_LIMIT");
        return false;
    }
    return true;
}

bool MovingLissa::yAmplitudeWithinLimits(float amplitude) const {
    if (amplitude < 0.0f || amplitude > Y_SCAN_AMPLITUDE_LIMIT) {
        Debug::msg("Y amplitude cannot be negative or greater than Y_SCAN_AMPLITUDE_LIMIT");
        return false;
    }
    return true;
}

bool MovingLissa::xCentreWithinLimits(float centreX) {
    if (centreX + x_amplitude > DEFAULT_X_UPPER_LIMIT || centreX - x_amplitude < DEFAULT_X_LOWER_LIMIT) {
        Debug::msg("X resulting path would exceed X limits");
        return false;
    }
    return true;
}

bool MovingLissa::yCentreWithinLimits(float centreY) {
    if (centreY + y_amplitude > DEFAULT_Y_UPPER_LIMIT || centreY - y_amplitude < DEFAULT_Y_LOWER_LIMIT) {
        Debug::msg("Y resulting path would exceed Y limits");
        return false;
    }
    return true;
}
