#pragma once

#include <Arduino.h>
#include <cmath>
#include "../../foc_controller/FOCController.hpp"
#include "../../motor/GM2804.hpp"
#include "../../sensor/AS5048.hpp"
#include "../../pwm_driver/PlatformPWMDriver.hpp"
#include "../wiring/wiring_common.hpp"

// L7 — Modulation math (inverse Park -> Clarke -> midpoint-clamp SVPWM).
//
// Validates FOCController::setPhaseVoltage() two ways:
//   Part 1 (numeric): with the coils idle (EN low), drive Vd=0 and a fixed Vq
//     through a spread of electrical angles, then compare the controller's
//     Va/Vb/Vc against an *independent* double-precision recomputation of the
//     same formula. This isolates a modulation-math bug from anything physical.
//   Part 2 (sweep): with the rotor mechanically clamped and EN high, sweep the
//     electrical angle 0->2π at fixed Vq and stream Va/Vb/Vc for scope capture
//     (120°-shifted line-to-line sinusoids; midpoint-clamp saddle per phase).
//
// The numeric reference deliberately uses std::sin/std::cos, NOT the
// controller's fast_sincos LUT, so the check is genuinely independent. The
// tolerance below covers the LUT approximation while still catching any
// sign/scale/transform error (which are volts, not millivolts, off).

static constexpr float L7_TEST_VQ = 2.0f;       // fixed Vq for the numeric check [V]
static constexpr float L7_SWEEP_VQ = 2.0f;      // fixed Vq for the swept waveform [V]
static constexpr float L7_ABS_TOL_V = 0.05f;    // per-phase match tolerance [V]

static constexpr int L7_SWEEP_STEPS = 72;       // electrical steps per revolution (5°)
static constexpr uint32_t L7_SWEEP_STEP_MS = 50; // dwell per step -> ~3.6 s / electrical rev
static constexpr int L7_SWEEP_REVS = 3;         // electrical revolutions to stream

// Electrical angles spanning all four quadrants.
static constexpr float L7_TEST_ANGLES[] = {
    0.0f,
    _PI_6,          // π/6
    _PI_4,          // π/4
    _PI_3,          // π/3
    _PI_2,          // π/2
    _120_D2R,       // 2π/3
    _PI - _PI_6,    // 5π/6
    _PI,            // π
    _PI + _PI_4,    // 5π/4
    _3PI_2,         // 3π/2
    _2PI - _PI_3,   // 5π/3
    _2PI - _PI_6,   // 11π/6
};
static constexpr size_t L7_TEST_ANGLE_COUNT = sizeof(L7_TEST_ANGLES) / sizeof(L7_TEST_ANGLES[0]);

// Independent recomputation of setPhaseVoltage()'s inverse-Park -> Clarke ->
// midpoint-clamp chain in double precision. Mirrors FOCController.cpp exactly.
inline void modulation_expected(float Vd, float Vq, float angle, float voltage_limit,
                                float& Va, float& Vb, float& Vc) {
    const double sa = sin((double)angle);
    const double ca = cos((double)angle);

    const double Valpha = ca * (double)Vd - sa * (double)Vq;   // inverse Park
    const double Vbeta  = sa * (double)Vd + ca * (double)Vq;

    double a = Valpha;                                          // Clarke
    double b = -0.5 * Valpha + (double)_SQRT3_2 * Vbeta;
    double c = -0.5 * Valpha - (double)_SQRT3_2 * Vbeta;

    double center = (double)voltage_limit / 2.0;               // midpoint clamp
    const double vmin = fmin(a, fmin(b, c));
    const double vmax = fmax(a, fmax(b, c));
    center -= (vmax + vmin) / 2.0;

    Va = (float)(a + center);
    Vb = (float)(b + center);
    Vc = (float)(c + center);
}

inline void modulation_print_spec() {
    Serial.print("SPEC test_Vq_V=");
    Serial.print(L7_TEST_VQ, 2);
    Serial.print(" sweep_Vq_V=");
    Serial.print(L7_SWEEP_VQ, 2);
    Serial.print(" voltage_limit_V=");
    Serial.print(MOTOR_VOLTAGE_LIMIT, 1);
    Serial.print(" abs_tol_V=");
    Serial.println(L7_ABS_TOL_V, 3);
    Serial.println("SPEC modulation=invPark->Clarke->midpoint-clamp SVPWM (Vd=0)");
}

inline void modulation_bench_prompt() {
    Serial.println();
    Serial.println("SAFETY — L7 modulation math + swept waveform:");
    Serial.println("  Part 1 (numeric): coils idle (EN low) — pure math check, no motion.");
    Serial.println("  Part 2 (sweep): EN high, rotor MECHANICALLY CLAMPED, fixed Vq, slow sweep.");
    Serial.println("  VM ON (12 V). Bench supply current limit ~1.0 A.");
    Serial.println("  Scope: two phase outputs + one line-to-line (e.g. N1-N2).");
    Serial.println("  Other axis driver EN must stay LOW.");
    Serial.println("  E-stop: cut VM or pull EN low if anything looks wrong.");
    modulation_print_spec();
    wiring_wait_for_confirm("VM on, supply current-limited, scope ready");
}

// PWMDriver::init() leaves EN HIGH but enabled=false, so a bare disable() is a
// no-op. Toggle enable->disable to guarantee EN is driven LOW (coils idle)
// before the numeric part energises nothing.
inline void modulation_force_disable(PlatformPWMDriver& driver) {
    driver.enable();
    driver.disable();
}

// Part 1: numeric validation of setPhaseVoltage() against the independent model.
inline bool modulation_numeric_check(FOCController& foc, float voltage_limit, const char* axis) {
    Serial.print("MOD axis=");
    Serial.print(axis);
    Serial.print(" phase=numeric Vd=0.00 Vq=");
    Serial.print(L7_TEST_VQ, 2);
    Serial.print(" angles=");
    Serial.println((int)L7_TEST_ANGLE_COUNT);

    foc.setVd(0.0f);
    foc.setVq(L7_TEST_VQ);

    int pass = 0;
    int fail = 0;
    float worst_err = 0.0f;

    for (size_t i = 0; i < L7_TEST_ANGLE_COUNT; ++i) {
        const float angle = L7_TEST_ANGLES[i];
        foc.setElectricalAngle(angle);
        foc.setPhaseVoltage();

        const float Va = foc.getVa();
        const float Vb = foc.getVb();
        const float Vc = foc.getVc();

        float eVa, eVb, eVc;
        modulation_expected(0.0f, L7_TEST_VQ, angle, voltage_limit, eVa, eVb, eVc);

        const float dA = fabsf(Va - eVa);
        const float dB = fabsf(Vb - eVb);
        const float dC = fabsf(Vc - eVc);
        const float err = max(dA, max(dB, dC));
        if (err > worst_err) worst_err = err;
        const bool ok = err <= L7_ABS_TOL_V;
        ok ? ++pass : ++fail;

        Serial.print("MOD axis=");
        Serial.print(axis);
        Serial.print(" i=");
        Serial.print((int)i);
        Serial.print(" angle=");
        Serial.print(angle, 4);
        Serial.print(" Va=");
        Serial.print(Va, 4);
        Serial.print(" Vb=");
        Serial.print(Vb, 4);
        Serial.print(" Vc=");
        Serial.print(Vc, 4);
        Serial.print(" expVa=");
        Serial.print(eVa, 4);
        Serial.print(" expVb=");
        Serial.print(eVb, 4);
        Serial.print(" expVc=");
        Serial.print(eVc, 4);
        Serial.print(" err=");
        Serial.print(err, 4);
        Serial.print(" chk=");
        Serial.println(ok ? "OK" : "FAIL");
    }

    Serial.print("MOD axis=");
    Serial.print(axis);
    Serial.print(" numeric_pass=");
    Serial.print(pass);
    Serial.print(" numeric_fail=");
    Serial.print(fail);
    Serial.print(" worst_err_V=");
    Serial.print(worst_err, 4);
    Serial.print(" tol_V=");
    Serial.println(L7_ABS_TOL_V, 3);

    return fail == 0;
}

inline void modulation_sweep_prompt(const char* axis) {
    Serial.println();
    Serial.print("SAFETY — clamp motor ");
    Serial.print(axis);
    Serial.println(" rotor before the swept-voltage step.");
    Serial.println("SCOPE — probe two phases + one line-to-line for this axis.");
    wiring_wait_for_confirm("rotor clamped + scope on this axis' phases");
}

// Part 2: sweep the electrical angle at fixed Vq for scope inspection.
inline void modulation_sweep(FOCController& foc, PlatformPWMDriver& driver, const char* axis) {
    foc.setVd(0.0f);
    foc.setVq(L7_SWEEP_VQ);

    Serial.print("MOD axis=");
    Serial.print(axis);
    Serial.print(" phase=sweep Vq=");
    Serial.print(L7_SWEEP_VQ, 2);
    Serial.print(" steps=");
    Serial.print(L7_SWEEP_STEPS);
    Serial.print(" revs=");
    Serial.print(L7_SWEEP_REVS);
    Serial.print(" step_ms=");
    Serial.println(L7_SWEEP_STEP_MS);
    Serial.println("SCOPE — phases balanced, 120° apart, clamped in [0, Vlimit]; line-to-line clean sinusoid.");

    driver.enable();

    for (int rev = 0; rev < L7_SWEEP_REVS; ++rev) {
        for (int i = 0; i < L7_SWEEP_STEPS; ++i) {
            const float angle = _2PI * ((float)i / (float)L7_SWEEP_STEPS);
            foc.setElectricalAngle(angle);
            foc.setPhaseVoltage();

            Serial.print("DBG axis=");
            Serial.print(axis);
            Serial.print(" rev=");
            Serial.print(rev);
            Serial.print(" angle=");
            Serial.print(angle, 4);
            Serial.print(" Va=");
            Serial.print(foc.getVa(), 4);
            Serial.print(" Vb=");
            Serial.print(foc.getVb(), 4);
            Serial.print(" Vc=");
            Serial.println(foc.getVc(), 4);

            delay(L7_SWEEP_STEP_MS);
        }
    }

    driver.disable();
    wiring_wait_for_confirm("scope: phases balanced/120° apart/clamped; line-to-line clean sinusoid");
}
