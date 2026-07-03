#pragma once

#include <Arduino.h>
#include <math.h>
#include "l9_common.hpp"

// L9 (part 2) — Electrical-offset autocal (stiction-bracket + circular mean).
//
// Method (docs/motor_calibration_findings.md): for each target electrical angle
// theta spread over a full mechanical revolution, approach theta from BOTH sides
// (pull the rotor past by BRACKET_OFFSET, then creep back to theta and settle).
// Friction sticks the rotor just short of true alignment on each side; the
// circular-mean midpoint cancels that symmetric dead-band. The implied offset is
//   offset = wrap(mech_mid * pole_pairs * sensor_direction - theta - PI/2)
// and the offsets over all targets are circular-averaged. Even target spacing
// (6 per pole pair) averages out the inherent +-0.5 rad 2nd-harmonic saliency.
//
// The -PI/2 term is the commutation convention correction. l9_hold_field() drives
// Vd=0,Vq at electrical_angle=theta, and setPhaseVoltage() places the stator field
// 90 deg AHEAD of electrical_angle, so the rotor d-axis settles at theta + PI/2, not
// theta. calculate_electrical_angle() = mech*pp*dir - offset must return the rotor
// d-axis (so the +90 deg in setPhaseVoltage lands current on the q-axis for torque);
// hence offset = mech*pp*dir - (theta + PI/2). Omitting -PI/2 stores an offset PI/2
// too large -> d-axis current injection -> full coil current but ~no torque.
//
// Pass bar (docs/FOC_bringup_plan.md L9):
//   - Prints a per-axis electrical offset + circ_std to store in constants.h.
//   - On a verification re-run (after storing), change vs stored offset ~= 0.
//
// BRACKET_OFFSET must EXCEED the stiction dead-band so the rotor breaks free on
// both sides; shrinking it below the dead-band makes consistency worse, not
// better (circ_std 0.44 -> 1.67 in bench evidence).

static constexpr int      L9_TARGETS_PER_POLE = 6;      // even phases average harmonics 1..5
static constexpr float    L9_CAL_VQ_REQ       = 6.0f;   // requested drive [V] (clamped to ceiling)
static constexpr float    L9_BRACKET_OFFSET   = 4.0f;   // elec rad; must exceed stiction dead-band
static constexpr uint32_t L9_HOLD_SETTLE_MS   = 900;    // settle at the target before capture
static constexpr int      L9_CREEP_STEPS      = 8;      // sub-steps from bracket back to target
static constexpr uint32_t L9_CREEP_DWELL_MS   = 45;     // dwell per creep sub-step

inline int l9_autocal_target_count() {
    return L9_TARGETS_PER_POLE * GM2804_POLE_PAIRS;  // 42 for 7 pole pairs
}

inline void l9_autocal_print_spec(const char* axis, float vq, float stored_offset) {
    Serial.print("SPEC axis=");
    Serial.print(axis);
    Serial.print(" Vq_V=");
    Serial.print(vq, 2);
    Serial.print(" targets=");
    Serial.print(l9_autocal_target_count());
    Serial.print(" bracket_rad=");
    Serial.print(L9_BRACKET_OFFSET, 2);
    Serial.print(" settle_ms=");
    Serial.print(L9_HOLD_SETTLE_MS);
    Serial.print(" pole_pairs=");
    Serial.print(GM2804_POLE_PAIRS);
    Serial.print(" sensor_dir=");
    Serial.print(SENSOR_DIRECTION);
    Serial.print(" stored_offset_rad=");
    Serial.println(stored_offset, 4);
}

inline void l9_autocal_bench_prompt(const char* axis, float stored_offset) {
    Serial.println();
    Serial.print("SAFETY — L9 electrical-offset autocal, axis ");
    Serial.println(axis);
    Serial.println("  Rotor FREE and UNLOADED (>=1 turn clearance). It twitches back and forth; no spin.");
    Serial.println("  VM ON (12 V). Bench supply current limit ~1.0-1.5 A (fold-back armed).");
    Serial.println("  Other axis driver EN must stay LOW. Runs ~1-2 min/axis. E-stop = cut VM / pull EN low.");
    l9_autocal_print_spec(axis, l9_safe_vq(L9_CAL_VQ_REQ), stored_offset);
    wiring_wait_for_confirm("rotor free/unloaded, VM on + current-limited, other axis EN low");
}

// Approach `theta` from one side: overshoot by dir*BRACKET_OFFSET, then creep back
// to theta and hold to settle. Returns the settled mechanical angle [0, 2PI).
inline float l9_autocal_bracket(FOCController& foc, Sensor& sensor,
                                float theta, float vq, int dir) {
    const float overshoot = theta + (float)dir * L9_BRACKET_OFFSET;
    l9_hold_field(foc, overshoot, vq, L9_HOLD_SETTLE_MS);
    l9_creep_field(foc, overshoot, theta, vq, L9_CREEP_STEPS, L9_CREEP_DWELL_MS);
    l9_hold_field(foc, theta, vq, L9_HOLD_SETTLE_MS);
    return l9_read_mech(sensor);
}

// Full per-axis autocal. Returns the measured electrical offset [0, 2PI).
inline float l9_autocal_run(FOCController& foc, PlatformPWMDriver& driver,
                            Sensor& sensor, float stored_offset, const char* axis) {
    const float vq = l9_safe_vq(L9_CAL_VQ_REQ);
    const int targets = l9_autocal_target_count();
    // Spread targets evenly over a full mechanical revolution's worth of
    // electrical angle (= pole_pairs * 2PI), i.e. 6 phases per pole pair.
    const float theta_step = (float)GM2804_POLE_PAIRS * _2PI / (float)targets;

    L9CircularAccumulator acc;

    driver.enable();

    Serial.print("AUTOCAL axis=");
    Serial.print(axis);
    Serial.print(" targets=");
    Serial.println(targets);
    Serial.println("TGT columns: axis,i,theta_elec,mech_lo,mech_hi,mech_mid,implied_offset");

    for (int i = 0; i < targets; i++) {
        const float theta = (float)i * theta_step;

        const float mech_lo = l9_autocal_bracket(foc, sensor, theta, vq, -1);  // from below
        const float mech_hi = l9_autocal_bracket(foc, sensor, theta, vq, +1);  // from above
        const float mech_mid = l9_circular_mean2(mech_lo, mech_hi);

        const float implied = normalise_angle(
            mech_mid * (float)GM2804_POLE_PAIRS * (float)SENSOR_DIRECTION - theta - _PI_2);
        acc.add(implied);

        Serial.print("TGT ");
        Serial.print(axis);
        Serial.print(",");
        Serial.print(i);
        Serial.print(",");
        Serial.print(normalise_angle(theta), 4);
        Serial.print(",");
        Serial.print(mech_lo, 4);
        Serial.print(",");
        Serial.print(mech_hi, 4);
        Serial.print(",");
        Serial.print(mech_mid, 4);
        Serial.print(",");
        Serial.println(implied, 4);
    }

    driver.disable();

    const float offset = acc.mean();
    const float circ_std = acc.circStd();
    const float R = acc.R();
    const float change = findAngularError(offset, stored_offset);

    Serial.print(">>> electrical offset axis=");
    Serial.print(axis);
    Serial.print(" = ");
    Serial.print(offset, 4);
    Serial.println(" rad");
    Serial.print("consistency: circ_std=");
    Serial.print(circ_std, 4);
    Serial.print(" rad (R=");
    Serial.print(R, 4);
    Serial.println("; lower circ_std = better; ~0.44 rad is the inherent saliency band)");
    Serial.print("change vs stored offset = ");
    Serial.print(change, 4);
    Serial.println(" rad  (target ~0 on a verification re-run)");

    Serial.print("RESULT electrical_offset_autocal_");
    Serial.print(axis[0] == 'X' ? "x" : "y");
    Serial.print(" offset_rad=");
    Serial.print(offset, 4);
    Serial.print(" circ_std_rad=");
    Serial.print(circ_std, 4);
    Serial.print(" change_vs_stored_rad=");
    Serial.print(change, 4);
    Serial.println(" — write offset into constants.h, then re-run to verify (change~0).");

    return offset;
}
