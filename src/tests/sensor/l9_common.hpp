#pragma once

#include <Arduino.h>
#include <math.h>
#include "../../foc_controller/FOCController.hpp"
#include "../../motor/GM2804.hpp"
#include "../../sensor/AS5048.hpp"
#include "../../pwm_driver/PlatformPWMDriver.hpp"
#include "../../utils/constants.h"
#include "../../utils/quick_defs.h"
#include "../wiring/wiring_common.hpp"

// L9 — shared low-level helpers for the pole-pair/direction and electrical-offset
// autocal scripts. All drive here is a *static* d-axis field (Vd=0, fixed Vq at a
// fixed electrical angle); the rotor settles into a detent. Nothing here spins a
// continuously rotating field, because these motors are stiction-dominated when
// unloaded and would just ratchet (see docs/motor_calibration_findings.md).

// Clamp a desired drive voltage so the standstill coil current (~Vq/R) never
// exceeds MOTOR_CURRENT_LIMIT. GM2804: R=9 Ohm, limit 0.8 A -> ~7.2 V ceiling.
inline float l9_safe_vq(float desired_vq) {
    const float v_ceiling = MOTOR_CURRENT_LIMIT * GM2804_PHASE_RESISTANCE;
    return desired_vq < v_ceiling ? desired_vq : v_ceiling;
}

// Command a static field at electrical angle `elec` (Vd=0, fixed Vq) and hold it
// for `hold_ms`. The PWM registers latch the vector, so one setPhaseVoltage()
// call plus a delay keeps the field steady while the rotor settles.
inline void l9_hold_field(FOCController& foc, float elec, float vq, uint32_t hold_ms) {
    foc.setVd(0.0f);
    foc.setVq(vq);
    foc.setElectricalAngle(normalise_angle(elec));
    foc.setPhaseVoltage();
    delay(hold_ms);
}

// Slew the commanded electrical angle from `from` to `to` in `steps` sub-steps,
// dwelling `dwell_ms` at each, so the rotor is dragged gently rather than kicked.
inline void l9_creep_field(FOCController& foc, float from, float to,
                           float vq, int steps, uint32_t dwell_ms) {
    if (steps < 1) steps = 1;
    for (int i = 1; i <= steps; i++) {
        const float e = from + (to - from) * (float)i / (float)steps;
        l9_hold_field(foc, e, vq, dwell_ms);
    }
}

// Read the freshest mechanical (rotor) angle in radians [0, 2PI).
inline float l9_read_mech(Sensor& sensor) {
    sensor.update();
    return sensor.getLatestRotorAngle();
}

// Circular mean of two angles (handles the 0/2PI wrap), returned in [0, 2PI).
inline float l9_circular_mean2(float a, float b) {
    const float s = sinf(a) + sinf(b);
    const float c = cosf(a) + cosf(b);
    return normalise_angle(atan2f(s, c));
}

// Running accumulator for a circular mean over many angles, plus the mean
// resultant length R (concentration; 1 = perfectly consistent, 0 = uniform).
struct L9CircularAccumulator {
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;
    uint32_t n = 0;

    void add(float angle) {
        sum_sin += sinf(angle);
        sum_cos += cosf(angle);
        n++;
    }
    float mean() const { return normalise_angle(atan2f(sum_sin, sum_cos)); }
    float R() const {
        if (n == 0) return 0.0f;
        return sqrtf(sum_sin * sum_sin + sum_cos * sum_cos) / (float)n;
    }
    // Circular standard deviation in radians (Mardia): sqrt(-2 ln R).
    float circStd() const {
        const float r = R();
        if (r <= 1e-6f) return 6.2831853f;  // effectively uniform
        if (r >= 1.0f) return 0.0f;
        return sqrtf(-2.0f * logf(r));
    }
};

// Construct the axis PWM driver, force the *other* axis fully low first, and
// report failure on the serial contract. Returns true on success.
inline bool l9_init_driver(PlatformPWMDriver& driver, const char* result_tag) {
    if (!driver.init()) {
        Serial.print("RESULT ");
        Serial.print(result_tag);
        Serial.println(" FAIL driver_init=0");
        return false;
    }
    return true;
}
