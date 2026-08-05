#pragma once

#include "../../utils/tweakable_params.h"

class CoarseScan {

public:
    CoarseScan() = default;
    virtual ~CoarseScan() = default;

    virtual void updateXPosition(float dt, float& x_pos) = 0;
    virtual void updateYPosition(float dt, float& y_pos) = 0;
    virtual void updateXVelocity(float dt, float& x_vel) = 0;
    virtual void updateYVelocity(float dt, float& y_vel) = 0;

    virtual bool xCentreWithinLimits(float _x_centre) = 0;
    virtual bool yCentreWithinLimits(float _y_centre) = 0;

    virtual void setXFreq(float _x_frequency) = 0;
    virtual void setYFreq(float _y_frequency) = 0;
    virtual void setXAmplitude(float _x_amplitude) = 0;
    virtual void setYAmplitude(float _y_amplitude) = 0;
    virtual void setXPhase(float _x_phase) = 0;
    virtual void setYPhase(float _y_phase) = 0;

    void resetTime() { t = 0.0f; }
    void progressTime(float dt) { t += dt; }

protected:
    float t = 0.0f;
    const float velocityDelay = DEFAULT_VELOCITY_DELAY_US;
};
