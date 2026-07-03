#pragma once

#include <Arduino.h>
#include <math.h>
#include "l9_common.hpp"

// L9 (part 1) — Pole pairs & sensor direction.
//
// Steps a *static* d-axis field quasi-statically through slightly more than one
// full mechanical revolution (= pole_pairs electrical revolutions), dwelling at
// each step so the stiction-dominated rotor settles into the commanded detent.
// At every step it records the (known) cumulative commanded electrical angle
// against the unwrapped measured mechanical angle.
//
// The relationship the whole controller depends on is
//   electrical_angle = mechanical_angle * pole_pairs * sensor_direction - offset
// so the slope d(cmd_elec)/d(mech) over the sweep is exactly
//   pole_pairs * sensor_direction.
// => |slope| = pole_pairs, sign(slope) = sensor_direction.
//
// Pass bar (docs/FOC_bringup_plan.md L9):
//   - Measured pole_pairs = 7 (within tolerance).
//   - sensor_direction = +1 confirmed (or corrected).
//
// A direction error surfaces later as an L10 failure — get the sign right here.

static constexpr float    L9_PP_VQ_REQ    = 5.0f;    // requested drive [V] (clamped to current ceiling)
static constexpr float    L9_PP_STEP_ELEC = _PI_6;   // electrical step per dwell [rad] (30 deg)
static constexpr float    L9_PP_REV_MARGIN = 1.15f;  // sweep length as fraction of one mech rev
static constexpr uint32_t L9_PP_DWELL_MS  = 220;     // settle time per step [ms]
static constexpr uint32_t L9_PP_ALIGN_MS  = 700;     // initial alignment settle [ms]

// Number of electrical steps to cover L9_PP_REV_MARGIN mechanical revolutions.
inline int l9_pp_step_count() {
    const float elec_span = L9_PP_REV_MARGIN * (float)GM2804_POLE_PAIRS * _2PI;
    return (int)ceilf(elec_span / L9_PP_STEP_ELEC);
}

inline void l9_pp_print_spec(const char* axis, float vq) {
    Serial.print("SPEC axis=");
    Serial.print(axis);
    Serial.print(" Vq_V=");
    Serial.print(vq, 2);
    Serial.print(" step_elec_rad=");
    Serial.print(L9_PP_STEP_ELEC, 4);
    Serial.print(" steps=");
    Serial.print(l9_pp_step_count());
    Serial.print(" dwell_ms=");
    Serial.print(L9_PP_DWELL_MS);
    Serial.print(" expected_pole_pairs=");
    Serial.print(GM2804_POLE_PAIRS);
    Serial.print(" expected_sensor_dir=");
    Serial.println(SENSOR_DIRECTION);
}

inline void l9_pp_bench_prompt(const char* axis) {
    Serial.println();
    Serial.print("SAFETY — L9 pole-pair/direction, axis ");
    Serial.println(axis);
    Serial.println("  Rotor FREE and UNLOADED (>=1 turn clearance). Field steps in detents, not a spin.");
    Serial.println("  VM ON (12 V). Bench supply current limit ~1.0-1.5 A (fold-back armed).");
    Serial.println("  Other axis driver EN must stay LOW. E-stop = cut VM / pull EN low.");
    l9_pp_print_spec(axis, l9_safe_vq(L9_PP_VQ_REQ));
    wiring_wait_for_confirm("rotor free/unloaded, VM on + current-limited, other axis EN low");
}

// Sweep the field one direction, fitting cmd_elec (y) vs unwrapped mech (x).
// Fills slope + r2 + endpoint ratio. `dir` is +1 (elec increasing) or -1.
inline void l9_pp_sweep(FOCController& foc, Sensor& sensor, float vq,
                        const char* axis, int dir,
                        float& slope_out, float& r2_out, float& endpoint_ratio_out) {
    const int steps = l9_pp_step_count();

    // Align: park the field at 0 and let the rotor settle to a known detent.
    l9_hold_field(foc, 0.0f, vq, L9_PP_ALIGN_MS);

    float mech_prev = l9_read_mech(sensor);
    float cum_mech = 0.0f;   // unwrapped mechanical [rad]
    float cum_elec = 0.0f;   // cumulative commanded electrical [rad]

    // Least-squares accumulators: x = cum_mech, y = cum_elec.
    double n = 0.0, Sx = 0.0, Sy = 0.0, Sxx = 0.0, Syy = 0.0, Sxy = 0.0;
    float first_mech = 0.0f;
    float last_mech = 0.0f;

    Serial.print("SWEEP axis=");
    Serial.print(axis);
    Serial.print(" dir=");
    Serial.println(dir);
    Serial.println("DBG columns: axis,dir,step,cmd_elec_cum,cmd_elec_wrapped,mech,mech_cum");

    for (int i = 0; i <= steps; i++) {
        if (i > 0) {
            cum_elec += (float)dir * L9_PP_STEP_ELEC;
            l9_hold_field(foc, cum_elec, vq, L9_PP_DWELL_MS);
        }

        const float mech = l9_read_mech(sensor);
        const float d = findAngularError(mech, mech_prev);  // shortest signed step
        cum_mech += d;
        mech_prev = mech;

        if (i == 0) first_mech = 0.0f;  // cum_mech baseline is 0
        last_mech = cum_mech;

        const double x = (double)cum_mech;
        const double y = (double)cum_elec;
        n   += 1.0;
        Sx  += x;   Sy  += y;
        Sxx += x*x; Syy += y*y; Sxy += x*y;

        Serial.print("DBG ");
        Serial.print(axis);
        Serial.print(",");
        Serial.print(dir);
        Serial.print(",");
        Serial.print(i);
        Serial.print(",");
        Serial.print(cum_elec, 4);
        Serial.print(",");
        Serial.print(normalise_angle(cum_elec), 4);
        Serial.print(",");
        Serial.print(mech, 4);
        Serial.print(",");
        Serial.println(cum_mech, 4);
    }

    const double denom = (n * Sxx - Sx * Sx);
    slope_out = denom != 0.0 ? (float)((n * Sxy - Sx * Sy) / denom) : 0.0f;

    const double r_num = (n * Sxy - Sx * Sy);
    const double r_den = (n * Sxx - Sx * Sx) * (n * Syy - Sy * Sy);
    r2_out = r_den > 0.0 ? (float)((r_num * r_num) / r_den) : 0.0f;

    const float mech_travel = last_mech - first_mech;
    endpoint_ratio_out = fabsf(mech_travel) > 1e-4f ? (cum_elec / mech_travel) : 0.0f;
}

// Full per-axis run: forward sweep, reverse sweep, verdict.
inline void l9_pp_run(FOCController& foc, PlatformPWMDriver& driver,
                      Sensor& sensor, const char* axis) {
    const float vq = l9_safe_vq(L9_PP_VQ_REQ);

    driver.enable();

    float slope_f = 0.0f, r2_f = 0.0f, ratio_f = 0.0f;
    l9_pp_sweep(foc, sensor, vq, axis, +1, slope_f, r2_f, ratio_f);

    float slope_r = 0.0f, r2_r = 0.0f, ratio_r = 0.0f;
    l9_pp_sweep(foc, sensor, vq, axis, -1, slope_r, r2_r, ratio_r);

    driver.disable();

    const float pp_fit = 0.5f * (fabsf(slope_f) + fabsf(slope_r));
    const int   dir_fit = (slope_f >= 0.0f) ? +1 : -1;

    Serial.print("FIT axis=");
    Serial.print(axis);
    Serial.print(" fwd_slope=");
    Serial.print(slope_f, 4);
    Serial.print(" fwd_r2=");
    Serial.print(r2_f, 4);
    Serial.print(" fwd_endpoint_ratio=");
    Serial.print(ratio_f, 4);
    Serial.print(" rev_slope=");
    Serial.print(slope_r, 4);
    Serial.print(" rev_r2=");
    Serial.print(r2_r, 4);
    Serial.print(" rev_endpoint_ratio=");
    Serial.println(ratio_r, 4);

    const bool pp_ok  = fabsf(pp_fit - (float)GM2804_POLE_PAIRS) <= 0.7f;
    const bool dir_ok = (dir_fit == SENSOR_DIRECTION);

    Serial.print("RESULT pole_pair_dir_");
    Serial.print(axis[0] == 'X' ? "x" : "y");
    Serial.print(" measured_pole_pairs=");
    Serial.print(pp_fit, 3);
    Serial.print(" sensor_dir=");
    Serial.print(dir_fit);
    Serial.print(" expected_pp=");
    Serial.print(GM2804_POLE_PAIRS);
    Serial.print(" expected_dir=");
    Serial.print(SENSOR_DIRECTION);
    Serial.print(" ");
    Serial.print((pp_ok && dir_ok) ? "PASS" : "CHECK");
    Serial.println(" (operator confirms against rotor motion)");
}
