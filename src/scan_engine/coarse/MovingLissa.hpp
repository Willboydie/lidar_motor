#pragma once

#include "../../utils/quick_defs.h"
#include "CoarseScan.hpp"

class MovingLissa : public CoarseScan {

public:
    MovingLissa(float _x_frequency, float _y_frequency, float _x_amplitude, float _y_amplitude, float _x_phase, float _y_phase);
    ~MovingLissa() = default;

    void updateXPosition(float dt, float& x_pos) override;
    void updateYPosition(float dt, float& y_pos) override;
    void updateXVelocity(float dt, float& x_vel) override;
    void updateYVelocity(float dt, float& y_vel) override;

    bool xCentreWithinLimits(float _x_centre) override;
    bool yCentreWithinLimits(float _y_centre) override;

    void setXFreq(float _x_frequency) override { x_frequency = _x_frequency; }
    void setYFreq(float _y_frequency) override { y_frequency = _y_frequency; }
    void setXAmplitude(float _x_amplitude) override;
    void setYAmplitude(float _y_amplitude) override;
    void setXPhase(float _x_phase) override { x_phase = _x_phase; }
    void setYPhase(float _y_phase) override { y_phase = _y_phase; }

private:
    bool xAmplitudeWithinLimits(float amplitude) const;
    bool yAmplitudeWithinLimits(float amplitude) const;

    float x_frequency = DEFAULT_X_SCAN_FREQUENCY;
    float y_frequency = DEFAULT_Y_SCAN_FREQUENCY;
    float x_amplitude = DEFAULT_X_SCAN_AMPLITUDE;
    float y_amplitude = DEFAULT_Y_SCAN_AMPLITUDE;
    float x_phase = DEFAULT_X_SCAN_PHASE;
    float y_phase = DEFAULT_Y_SCAN_PHASE;
};
