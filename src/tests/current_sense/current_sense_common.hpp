#pragma once

#include <Arduino.h>
#include <math.h>
#include "../../foc_controller/FOCController.hpp"
#include "../../motor/GM2804.hpp"
#include "../../sensor/AS5048.hpp"
#include "../../pwm_driver/PlatformPWMDriver.hpp"
#include "../wiring/wiring_common.hpp"

#if defined(STM32G4xx)
#include "../../current_sense/stm32g4/InlineCurrentSenseSTM32G4.hpp"

// L11 — Current-sense system validation (STM32G4 only).
//
// Proves the ACS712 + ADC/TRGO/DMA inline current sense CAPTURES honestly, in
// isolation, BEFORE any current-target PI (L12) trusts it. Three checks, in order:
//
//   1. BASELINE  — offsets calibrated at zero phase current (setPwm(0,0,0)); the
//      per-phase means must sit near 0 A with small noise, and DMA must actually be
//      streaming (conv count advancing, samples not railed/stuck).
//   2. FIXED     — apply a fixed stationary voltage vector (Vd=0, fixed Vq, fixed
//      electrical_angle); once the resistive current settles, getFOCCurrents() at
//      the SAME commanded angle should read Iq ~= Vq/R and Id ~= 0. Swept over a
//      few electrical angles so the captured vector is seen to rotate with the field.
//   3. SINUSOID  — ramp the commanded electrical angle at fixed Vq; the three phase
//      currents are balanced 120-shifted sinusoids, and Iq/Id (projected at the
//      commanded angle) stay ~Vq/R and ~0 with no capture dropouts.
//
// This is capture validation, NOT control: the loop drives voltages directly via
// setPhaseVoltage() and only READS the current sense. On a free rotor the fixed /
// swept field will move the rotor (expected) — the numbers of interest are the
// captured currents, not motion.
//
// Pass bar (docs/FOC_bringup_plan.md L11):
//   - Offsets in 1.60-1.70 V; DMA sampling valid (conv count moving).
//   - Baseline |mean(i_a)|,|mean(i_b)| < ~0.10 A; noise small.
//   - Fixed/sinusoid: measured Iq tracks Vq/R within tolerance, Id ~ 0, phase
//     currents balanced (|i_a+i_b+i_c| ~ 0), all under MOTOR_CURRENT_LIMIT.
//
// Abort: railed/stuck ADC samples, no conv-count movement, offsets out of band, or
// Iq that does not follow the commanded field => sensing/ADC/DMA fault, fix here.

static constexpr float    L11_VQ_REQ     = 2.0f;  // fixed drive [V] (clamped to current ceiling)
static constexpr float    L11_VQ_STEP    = 0.5f;  // '+'/'-' Vq magnitude step [V]
static constexpr uint32_t L11_STREAM_MS  = 50;    // DBG stream period [ms] (~20 Hz)
static constexpr uint32_t L11_BASELINE_MS = 1500; // baseline capture window [ms]
static constexpr uint32_t L11_FIXED_DWELL_MS = 2000; // dwell per fixed-vector angle [ms]
static constexpr float    L11_ELEC_RATE_RADS = _2PI; // sinusoid sweep rate [rad/s] (1 elec rev/s)

// Baseline mean tolerance [A] and Iq tracking tolerance (fraction of Vq/R).
static constexpr float    L11_BASELINE_TOL_A = 0.10f;
static constexpr float    L11_IQ_TOL_FRAC    = 0.40f;  // generous: BEMF/saliency/uncal gain

inline float current_sense_safe_vq(float desired_vq) {
    const float v_ceiling = MOTOR_CURRENT_LIMIT * GM2804_PHASE_RESISTANCE;
    return desired_vq < v_ceiling ? desired_vq : v_ceiling;
}

inline float current_sense_elec_angle(float mechanical_angle, float electrical_offset) {
    return normalise_angle(
        mechanical_angle * (float)GM2804_POLE_PAIRS * (float)SENSOR_DIRECTION - electrical_offset);
}

// Bring up driver + inline current sense: init hardware, validate DMA streaming,
// then calibrate offsets at zero current. Returns false (with a RESULT FAIL) on any
// hardware fault so the caller can bail safely.
inline bool current_sense_bringup(PlatformPWMDriver& driver, InlineCurrentSenseSTM32G4& cs,
                                  const char* result_tag) {
    if (!driver.init()) {
        Serial.print("RESULT ");
        Serial.print(result_tag);
        Serial.println(" FAIL driver_init=0");
        return false;
    }
    if (!cs.init()) {  // hardware TRGO + DMA
        Serial.print("RESULT ");
        Serial.print(result_tag);
        Serial.println(" FAIL current_sense_init=0");
        return false;
    }
    if (!cs.validateDmaSampling(50)) {
        Serial.print("RESULT ");
        Serial.print(result_tag);
        Serial.println(" FAIL dma_sampling_invalid (no conv movement / railed samples)");
        return false;
    }

    // Calibrate offsets at zero phase current: driver enabled, all duties 0.
    driver.enable();
    driver.setPwm(0.0f, 0.0f, 0.0f);
    delay(200);
    cs.calibrateOffsets(1000);
    driver.disable();

    const bool offsets_ok = (cs.offset_ia > 1.60f && cs.offset_ia < 1.70f) &&
                            (cs.offset_ib > 1.60f && cs.offset_ib < 1.70f);
    Serial.print("CAL ");
    Serial.print(result_tag);
    Serial.print(" offset_ia_V=");
    Serial.print(cs.offset_ia, 4);
    Serial.print(" offset_ib_V=");
    Serial.print(cs.offset_ib, 4);
    Serial.print(" gain_a_ApV=");
    Serial.print(cs.gain_a, 4);
    Serial.print(" offsets_in_band=");
    Serial.println(offsets_ok ? 1 : 0);
    return true;
}

inline void current_sense_print_spec(const char* axis, float electrical_offset) {
    const float vq = current_sense_safe_vq(L11_VQ_REQ);
    Serial.print("SPEC axis=");
    Serial.print(axis);
    Serial.print(" Vq_V=");
    Serial.print(vq, 2);
    Serial.print(" expected_Iq_A=");
    Serial.print(vq / GM2804_PHASE_RESISTANCE, 3);
    Serial.print(" R_ohm=");
    Serial.print(GM2804_PHASE_RESISTANCE, 3);
    Serial.print(" offset_rad=");
    Serial.print(electrical_offset, 4);
    Serial.print(" current_limit_A=");
    Serial.println(MOTOR_CURRENT_LIMIT, 2);
}

inline void current_sense_bench_prompt(const char* axis, float electrical_offset) {
    Serial.println();
    Serial.print("SAFETY — L11 current-sense validation, axis ");
    Serial.println(axis);
    Serial.println("  Rotor FREE and UNLOADED. Fixed/swept fields WILL move the free rotor — expected.");
    Serial.println("  VM ON (12 V). Bench supply current limit ~1.0-1.5 A (fold-back armed).");
    Serial.println("  Other axis driver EN must stay LOW. E-stop: ' ' or 'x' (or cut VM / pull EN low).");
    current_sense_print_spec(axis, electrical_offset);
    Serial.println("CONTROLS: 'b'=baseline  'f'=fixed sweep  'w'=sinusoid sweep  '+'/'-'=Vq  ' '/'x'=E-STOP  'q'=end axis.");
    wiring_wait_for_confirm("rotor free/unloaded, VM on + current-limited, other axis EN low");
}

// Zero-current baseline: with all duties 0, capture phase-current means/peaks over a
// window and verify they sit near 0 A (offset calibration good, no stray bias).
inline void current_sense_baseline(PlatformPWMDriver& driver, InlineCurrentSenseSTM32G4& cs,
                                   const char* axis) {
    driver.enable();
    driver.setPwm(0.0f, 0.0f, 0.0f);

    double sum_a = 0.0, sum_b = 0.0, sum_c = 0.0;
    float peak_a = 0.0f, peak_b = 0.0f, peak_c = 0.0f;
    uint32_t n = 0;
    const uint32_t t_end = millis() + L11_BASELINE_MS;
    while ((int32_t)(millis() - t_end) < 0) {
        const PhaseCurrent_s i = cs.getPhaseCurrents();
        sum_a += i.a; sum_b += i.b; sum_c += i.c;
        peak_a = max(peak_a, fabsf(i.a));
        peak_b = max(peak_b, fabsf(i.b));
        peak_c = max(peak_c, fabsf(i.c));
        n++;
        delayMicroseconds(200);
    }
    driver.disable();

    const float mean_a = n ? (float)(sum_a / n) : 0.0f;
    const float mean_b = n ? (float)(sum_b / n) : 0.0f;
    const float mean_c = n ? (float)(sum_c / n) : 0.0f;
    const bool ok = fabsf(mean_a) < L11_BASELINE_TOL_A && fabsf(mean_b) < L11_BASELINE_TOL_A;

    Serial.print("BASELINE axis=");
    Serial.print(axis);
    Serial.print(" n=");
    Serial.print(n);
    Serial.print(" mean_a_A=");
    Serial.print(mean_a, 4);
    Serial.print(" mean_b_A=");
    Serial.print(mean_b, 4);
    Serial.print(" mean_c_A=");
    Serial.print(mean_c, 4);
    Serial.print(" peak_a_A=");
    Serial.print(peak_a, 4);
    Serial.print(" peak_b_A=");
    Serial.print(peak_b, 4);
    Serial.print(" pass=");
    Serial.println(ok ? 1 : 0);
}

// Apply a fixed stationary field (Vd=0, fixed Vq) at a set of electrical angles,
// dwell for the resistive current to settle, then read getFOCCurrents() projected
// at the SAME commanded angle. With the field stationary Iq ~= Vq/R and Id ~= 0.
inline void current_sense_fixed_sweep(FOCController& foc, PlatformPWMDriver& driver,
                                      InlineCurrentSenseSTM32G4& cs, const char* axis, float vq_mag) {
    const float elec_angles[] = {0.0f, _120_D2R, 2.0f * _120_D2R};
    const float expect_iq = vq_mag / GM2804_PHASE_RESISTANCE;

    foc.setVd(0.0f);
    foc.setVq(vq_mag);
    driver.enable();

    Serial.print("FIXED axis=");
    Serial.print(axis);
    Serial.print(" Vq_V=");
    Serial.print(vq_mag, 2);
    Serial.print(" expected_Iq_A=");
    Serial.println(expect_iq, 3);

    for (float ea : elec_angles) {
        foc.setElectricalAngle(ea);
        foc.setPhaseVoltage();
        // settle the resistive current + let the free rotor align
        const uint32_t t_end = millis() + L11_FIXED_DWELL_MS;
        while ((int32_t)(millis() - t_end) < 0) {
            foc.setElectricalAngle(ea);
            foc.setPhaseVoltage();
            if (Serial.available()) {
                const char c = (char)Serial.read();
                if (c == ' ' || c == 'x' || c == 'X') { driver.disable(); Serial.println("ESTOP"); return; }
            }
            delayMicroseconds(200);
        }
        const PhaseCurrent_s i = cs.getPhaseCurrents();
        const DQCurrent_s dq = cs.getFOCCurrents(ea);
        const bool iq_ok = fabsf(dq.q - expect_iq) < (L11_IQ_TOL_FRAC * expect_iq + 0.05f);
        Serial.print("FIXPT axis=");
        Serial.print(axis);
        Serial.print(" elec_rad=");
        Serial.print(ea, 4);
        Serial.print(" i_a=");
        Serial.print(i.a, 4);
        Serial.print(" i_b=");
        Serial.print(i.b, 4);
        Serial.print(" i_c=");
        Serial.print(i.c, 4);
        Serial.print(" Iq=");
        Serial.print(dq.q, 4);
        Serial.print(" Id=");
        Serial.print(dq.d, 4);
        Serial.print(" iq_track=");
        Serial.println(iq_ok ? 1 : 0);
    }
    foc.setVq(0.0f);
    foc.setPhaseVoltage();
    driver.disable();
}

// Ramp the commanded electrical angle at a fixed rate (fixed Vq): stream captured
// phase currents + Iq/Id projected at the commanded angle. Balanced sinusoids and a
// steady Iq ~ Vq/R, Id ~ 0 prove the capture path over the full electrical cycle.
inline void current_sense_sinusoid(FOCController& foc, PlatformPWMDriver& driver,
                                   InlineCurrentSenseSTM32G4& cs, const char* axis, float vq_mag) {
    foc.setVd(0.0f);
    foc.setVq(vq_mag);
    driver.enable();

    Serial.print("SINU axis=");
    Serial.print(axis);
    Serial.print(" Vq_V=");
    Serial.print(vq_mag, 2);
    Serial.print(" elec_rate_rads=");
    Serial.println(L11_ELEC_RATE_RADS, 3);
    Serial.println("DBG columns: axis,t_ms,cmd_elec,i_a,i_b,i_c,Iq,Id,sum_abc");

    float cmd = 0.0f;
    uint32_t last_us = micros();
    uint32_t last_stream_ms = millis();
    while (true) {
        const uint32_t now_us = micros();
        float dt = (float)(now_us - last_us) * 1e-6f;
        last_us = now_us;
        if (dt < 0.0f || dt > 0.1f) dt = 0.0f;
        cmd = normalise_angle(cmd + L11_ELEC_RATE_RADS * dt);
        foc.setElectricalAngle(cmd);
        foc.setPhaseVoltage();

        const uint32_t now_ms = millis();
        if (now_ms - last_stream_ms >= L11_STREAM_MS) {
            last_stream_ms = now_ms;
            const PhaseCurrent_s i = cs.getPhaseCurrents();
            const DQCurrent_s dq = cs.getFOCCurrents(cmd);
            Serial.print("DBG ");
            Serial.print(axis);
            Serial.print(",");
            Serial.print(now_ms);
            Serial.print(",");
            Serial.print(cmd, 4);
            Serial.print(",");
            Serial.print(i.a, 4);
            Serial.print(",");
            Serial.print(i.b, 4);
            Serial.print(",");
            Serial.print(i.c, 4);
            Serial.print(",");
            Serial.print(dq.q, 4);
            Serial.print(",");
            Serial.print(dq.d, 4);
            Serial.print(",");
            Serial.println(i.a + i.b + i.c, 4);
        }

        if (Serial.available()) {
            const char c = (char)Serial.read();
            if (c == 'q' || c == 'Q' || c == ' ' || c == 'x' || c == 'X') {
                Serial.println(c == 'q' || c == 'Q' ? "CMD end_sinusoid" : "ESTOP");
                break;
            }
        }
    }
    foc.setVq(0.0f);
    foc.setPhaseVoltage();
    driver.disable();
}

// Core L11 harness: bring up sense, calibrate, then operator-selectable checks.
// Returns true when the operator ends the axis (verdict is logged from the numeric
// pass flags above; operator confirms overall).
inline bool current_sense_run(FOCController& foc, PlatformPWMDriver& driver,
                              InlineCurrentSenseSTM32G4& cs, Sensor& sensor,
                              float electrical_offset, const char* axis) {
    (void)sensor;
    (void)electrical_offset;
    float vq_mag = current_sense_safe_vq(L11_VQ_REQ);

    Serial.print("MOD axis=");
    Serial.print(axis);
    Serial.println(" phase=current_sense_validation — running baseline, then 'f'/'w' sweeps.");

    // Auto baseline first (zero-current), then hand control to the operator.
    current_sense_baseline(driver, cs, axis);

    while (true) {
        if (Serial.available()) {
            const char c = (char)Serial.read();
            if (c == 'b' || c == 'B') {
                current_sense_baseline(driver, cs, axis);
            } else if (c == 'f' || c == 'F') {
                current_sense_fixed_sweep(foc, driver, cs, axis, vq_mag);
            } else if (c == 'w' || c == 'W') {
                current_sense_sinusoid(foc, driver, cs, axis, vq_mag);
            } else if (c == '+' || c == '=') {
                vq_mag = current_sense_safe_vq(vq_mag + L11_VQ_STEP);
                Serial.print("CMD vq_mag_V=");
                Serial.println(vq_mag, 2);
            } else if (c == '-' || c == '_') {
                vq_mag -= L11_VQ_STEP;
                if (vq_mag < 0.0f) vq_mag = 0.0f;
                Serial.print("CMD vq_mag_V=");
                Serial.println(vq_mag, 2);
            } else if (c == ' ' || c == 'x' || c == 'X') {
                driver.disable();
                Serial.print("ESTOP axis=");
                Serial.println(axis);
            } else if (c == 'q' || c == 'Q') {
                Serial.print("CMD axis=");
                Serial.print(axis);
                Serial.println(" end");
                break;
            }
        }
        delay(2);
    }

    driver.disable();

    Serial.print("RESULT current_sense_");
    Serial.print(axis[0] == 'X' ? "x" : "y");
    Serial.println(" CHECK (operator confirms: offsets in band, baseline ~0, fixed Iq~Vq/R & Id~0, "
                   "sinusoid balanced, all under current limit)");
    return true;
}

#endif // STM32G4xx
