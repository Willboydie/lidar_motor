// Zero-current stub backend of InlineCurrentSense.
//
// The Teensy 4.1 build has no inline current-sense hardware wired, so this
// returns zero phase currents. It lets shared FOC / telemetry code (e.g.
// Interface::sendDualMotorStatus) run unchanged on targets without sensing,
// and doubles as a no-op current source for STM32G4 tests that only need the
// voltage / angle channels.
#pragma once

#include "../InlineCurrentSense.hpp"

class InlineCurrentSenseTeensy41 : public InlineCurrentSense {
public:
    PhaseCurrent_s getPhaseCurrents() override { return PhaseCurrent_s{0.0f, 0.0f, 0.0f}; }
    bool init() override { return true; }
};
