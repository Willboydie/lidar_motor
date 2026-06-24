#pragma once

#include "../utils/quick_defs.h"

// Phase and transform current containers shared across current-sense backends.
struct PhaseCurrent_s {
    float a;
    float b;
    float c;
};

struct ABCurrent_s {
    float alpha;
    float beta;
};

struct DQCurrent_s {
    float d;
    float q;
};

// Hardware-agnostic inline current sense base.
// Holds the per-phase ADC gains/offsets and the Clarke/Park maths used by FOC.
// A hardware backend (e.g. InlineCurrentSenseSTM32G4) implements getPhaseCurrents().
class InlineCurrentSense {
public:
    virtual ~InlineCurrentSense() = default;

    // Latest measured phase currents [A]. Implemented by the hardware backend.
    virtual PhaseCurrent_s getPhaseCurrents() = 0;

    // Clarke transform for two measured phases (assumes ia + ib + ic = 0).
    ABCurrent_s getABCurrents(PhaseCurrent_s current);
    // Park transform.
    DQCurrent_s getDQCurrents(ABCurrent_s current, float angle_el);
    // Full FOC current read: phase currents -> Clarke -> Park.
    DQCurrent_s getFOCCurrents(float angle_el);

    // Per-phase ADC gain [A/V] and zero-current offset voltage [V].
    float gain_a = 0.0f;
    float gain_b = 0.0f;
    float gain_c = 0.0f;
    float offset_ia = 0.0f;
    float offset_ib = 0.0f;
    float offset_ic = 0.0f;
};
