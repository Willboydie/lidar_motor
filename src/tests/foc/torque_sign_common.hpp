#pragma once

#include <Arduino.h>
#include <math.h>
#include "../../foc_controller/FOCController.hpp"
#include "../../motor/GM2804.hpp"
#include "../../sensor/AS5048.hpp"
#include "../../pwm_driver/PlatformPWMDriver.hpp"
#include "../wiring/wiring_common.hpp"

// L10 — Closed-loop commutation sign / torque test.
//
// FIRST use of the REAL encoder angle to commutate. Every loop iteration reads
// the live sensor, computes the electrical angle exactly as the controller does
//   electrical_angle = normalise_angle(mech * pole_pairs * sensor_direction - offset)
// sets Vd=0 and a small FIXED Vq (both PIDs bypassed), and calls setPhaseVoltage().
// With the right sign+offset this energises the q-axis 90 electrical degrees ahead
// of the rotor, so +Vq produces torque in one consistent direction and holding
// torque that resists a hand perturbation; flipping Vq reverses it.
//
// On a FREE, UNLOADED rotor a correct fixed +Vq produces CONSTANT torque, which
// spins the rotor continuously and accelerates it — this is EXPECTED, not a fault.
// There is no position loop here to hold it still (that is L12). So this harness
// does NOT auto-abort on rotation; it is operator-driven with a manual e-stop.
//
// Interaction model (all keys on serial):
//   - Starts IDLE (Vq=0, zero torque) so nothing moves until you command it.
//   - 'g' toggles CONTINUOUS drive: rotor should spin one way; 's' then reverses it.
//   - 'p' fires a bounded torque PULSE then coasts (pulse-then-hold): the rotor gets
//     a directional kick you can see/feel without a full spin-up; the net mechanical
//     displacement + its sign are printed. Flip with 's' and pulse again to confirm
//     the torque reverses. This is the controlled perturbation/direction check.
//   - '+'/'-' dial Vq; ' ' or 'x' = manual E-STOP (hard driver disable, stays safe).
//
// Pass bar (docs/FOC_bringup_plan.md L10, interpreted for a free rotor):
//   - Continuous +Vq spins one consistent direction; 's' cleanly reverses it.
//   - A +Vq pulse kicks the rotor one way; the flipped-sign pulse kicks it the other.
//   - No violent/uncontrolled divergence, no direction that contradicts the Vq sign.
//   - PHASE current stays under MOTOR_CURRENT_LIMIT.
//
// Abort/rollback: direction contradicts the Vq sign, or motion is chaotic/uncontrolled
// => wrong sensor_direction or offset, back to L9. Weak/erratic torque => re-check L5-L8.
//
// CURRENT NOTE: there is no current feedback in this controller. The printed
// est_phase_i_A is the standstill PHASE current (~Vq/R) that makes torque. The
// bench-SUPPLY (DC-bus) current reads much LOWER than this: a static holding
// vector does no mechanical work, so the bus only carries the averaged I^2R loss.
// Do NOT expect the bench meter to match est_phase_i_A — a few tenths of an amp on
// the supply is normal at these Vq levels. The ceiling clamp below bounds the
// PHASE current to MOTOR_CURRENT_LIMIT, which is the quantity that can cook coils.
//
// DRIVE LEVEL: default is a gentle 2 V so continuous drive spins the free rotor
// slowly and pulses give a modest kick. Use '+'/'-' to dial torque live; the value
// is always clamped to the current ceiling (0.8 A * 9 Ohm ~= 7.2 V).

static constexpr float    L10_VQ_REQ    = 2.0f;   // default fixed drive [V] (clamped to current ceiling)
static constexpr float    L10_VQ_STEP   = 0.5f;   // live '+'/'-' Vq magnitude step [V]
static constexpr uint32_t L10_STREAM_MS = 50;     // DBG stream period [ms] (~20 Hz)
static constexpr uint32_t L10_PULSE_MS  = 250;    // 'p' torque-pulse duration [ms] before coasting

// Clamp desired Vq so the standstill coil current (~Vq/R) stays under
// MOTOR_CURRENT_LIMIT. GM2804: R=9 Ohm, 0.8 A limit -> ~7.2 V ceiling.
inline float torque_sign_safe_vq(float desired_vq) {
    const float v_ceiling = MOTOR_CURRENT_LIMIT * GM2804_PHASE_RESISTANCE;
    return desired_vq < v_ceiling ? desired_vq : v_ceiling;
}

// Encoder-derived electrical angle used to COMMUTATE here (mirrors
// FOCController::calculate_electrical_angle(), which is private).
inline float torque_sign_elec_angle(float mechanical_angle, float electrical_offset) {
    return normalise_angle(
        mechanical_angle * (float)GM2804_POLE_PAIRS * (float)SENSOR_DIRECTION - electrical_offset);
}

// Construct the axis PWM driver; report failure on the serial contract.
inline bool torque_sign_init_driver(PlatformPWMDriver& driver, const char* result_tag) {
    if (!driver.init()) {
        Serial.print("RESULT ");
        Serial.print(result_tag);
        Serial.println(" FAIL driver_init=0");
        return false;
    }
    return true;
}

inline void torque_sign_print_spec(const char* axis, float electrical_offset) {
    const float vq = torque_sign_safe_vq(L10_VQ_REQ);
    Serial.print("SPEC axis=");
    Serial.print(axis);
    Serial.print(" Vq_V=");
    Serial.print(vq, 2);
    Serial.print(" est_phase_i_A=");
    Serial.print(vq / GM2804_PHASE_RESISTANCE, 3);
    Serial.print(" (bus current reads lower) pole_pairs=");
    Serial.print(GM2804_POLE_PAIRS);
    Serial.print(" sensor_dir=");
    Serial.print(SENSOR_DIRECTION);
    Serial.print(" offset_rad=");
    Serial.print(electrical_offset, 4);
    Serial.print(" pulse_ms=");
    Serial.println(L10_PULSE_MS);
    Serial.print("SPEC current_limit_A=");
    Serial.print(MOTOR_CURRENT_LIMIT, 2);
    Serial.print(" voltage_limit_V=");
    Serial.println(MOTOR_VOLTAGE_LIMIT, 1);
}

inline void torque_sign_bench_prompt(const char* axis, float electrical_offset) {
    Serial.println();
    Serial.print("SAFETY — L10 closed-loop torque sign, axis ");
    Serial.println(axis);
    Serial.println("  Rotor FREE and UNLOADED (>=1 turn clearance). The REAL encoder now commutates.");
    Serial.println("  Continuous drive SPINS the free rotor — that is expected, not a fault.");
    Serial.println("  VM ON (12 V). Bench supply current limit ~1.0-1.5 A (fold-back armed).");
    Serial.println("  Other axis driver EN must stay LOW. E-stop: ' ' or 'x' (or cut VM / pull EN low).");
    torque_sign_print_spec(axis, electrical_offset);
    Serial.println("CONTROLS (serial): 'g'=toggle spin  'p'=torque pulse  's'=flip sign  '+'/'-'=Vq  ' '/'x'=E-STOP  'q'=end axis.");
    wiring_wait_for_confirm("rotor free/unloaded, VM on + current-limited, other axis EN low");
}

// Apply a bounded torque pulse (sign*vq_mag) for L10_PULSE_MS while commutating off
// the live encoder, then coast (Vq=0). Prints the net signed mechanical displacement
// so the operator can confirm torque direction without a full spin-up. This is the
// controlled perturbation/direction check ("pulse-then-hold").
inline void torque_sign_pulse(FOCController& foc, Sensor& sensor,
                              float electrical_offset, const char* axis,
                              int sign, float vq_mag) {
    sensor.update();
    float prev = sensor.getLatestRotorAngle();
    float net = 0.0f;

    const uint32_t t_end = millis() + L10_PULSE_MS;
    while ((int32_t)(millis() - t_end) < 0) {
        sensor.update();
        const float m = sensor.getLatestRotorAngle();
        net += findAngularError(m, prev);  // shortest signed step, wrap-safe
        prev = m;
        foc.setVq((float)sign * vq_mag);
        foc.setElectricalAngle(torque_sign_elec_angle(m, electrical_offset));
        foc.setPhaseVoltage();
    }

    // Coast: hold zero torque (field parked at the rotor, Vq=0) so it settles.
    foc.setVq(0.0f);
    foc.setElectricalAngle(torque_sign_elec_angle(prev, electrical_offset));
    foc.setPhaseVoltage();

    Serial.print("PULSE axis=");
    Serial.print(axis);
    Serial.print(" vq_sign=");
    Serial.print(sign);
    Serial.print(" vq_mag_V=");
    Serial.print(vq_mag, 2);
    Serial.print(" net_disp_rad=");
    Serial.print(net, 4);
    Serial.print(" kick_dir=");
    Serial.println(net > 0.0f ? "+" : (net < 0.0f ? "-" : "0"));
}

// Core closed-loop torque-sign harness. Commutates off the live encoder; operator
// drives it (idle / continuous spin / bounded pulse) and confirms the pass bar by
// observation. No auto-abort on motion — rotation under fixed Vq is expected on a
// free rotor. E-stop (' '/'x') hard-disables the driver; 'q' ends the axis. Always
// returns true (verdict is the operator's, logged as CHECK).
inline bool torque_sign_run(FOCController& foc, PlatformPWMDriver& driver,
                            Sensor& sensor, float electrical_offset, const char* axis) {
    const float vq_ceiling = torque_sign_safe_vq(1e9f);  // current-limited voltage ceiling
    float vq_mag = torque_sign_safe_vq(L10_VQ_REQ);
    int sign = +1;
    bool continuous = false;   // continuous-spin toggle ('g')
    bool driver_live = true;   // false after a manual e-stop until re-armed by a drive cmd

    foc.setVd(0.0f);

    Serial.print("MOD axis=");
    Serial.print(axis);
    Serial.print(" phase=torque_sign vq_sign=");
    Serial.print(sign);
    Serial.println(" — IDLE. 'g' spins, 'p' pulses, 's' flips sign. Rotation is expected.");
    Serial.println("DBG columns: axis,t_ms,mode,vq_sign,Vq,mech_rad,elec_rad,filt_vel_rads,est_phase_i_A");

    driver.enable();

    uint32_t last_stream_ms = millis();

    while (true) {
        sensor.update();
        const float mech = sensor.getLatestRotorAngle();
        const float elec = torque_sign_elec_angle(mech, electrical_offset);
        const float vq = (driver_live && continuous) ? (float)sign * vq_mag : 0.0f;

        foc.setVq(vq);
        foc.setElectricalAngle(elec);
        foc.setPhaseVoltage();

        const float filt_vel = sensor.getFilteredAngularVelocity();

        const uint32_t now_ms = millis();
        if (now_ms - last_stream_ms >= L10_STREAM_MS) {
            last_stream_ms = now_ms;
            const char* mode = !driver_live ? "estop" : (continuous ? "cont" : "idle");
            Serial.print("DBG ");
            Serial.print(axis);
            Serial.print(",");
            Serial.print(now_ms);
            Serial.print(",");
            Serial.print(mode);
            Serial.print(",");
            Serial.print(sign);
            Serial.print(",");
            Serial.print(vq, 3);
            Serial.print(",");
            Serial.print(mech, 4);
            Serial.print(",");
            Serial.print(elec, 4);
            Serial.print(",");
            Serial.print(filt_vel, 4);
            Serial.print(",");
            Serial.println(fabsf(vq) / GM2804_PHASE_RESISTANCE, 3);
        }

        if (Serial.available()) {
            const char c = (char)Serial.read();
            if (c == 's' || c == 'S') {
                sign = -sign;
                Serial.print("CMD axis=");
                Serial.print(axis);
                Serial.print(" flip vq_sign=");
                Serial.println(sign);
            } else if (c == 'g' || c == 'G') {
                if (!driver_live) {
                    driver.enable();
                    driver_live = true;
                }
                continuous = !continuous;
                Serial.print("CMD axis=");
                Serial.print(axis);
                Serial.print(" continuous=");
                Serial.println(continuous ? 1 : 0);
            } else if (c == 'p' || c == 'P') {
                if (!driver_live) {
                    driver.enable();
                    driver_live = true;
                }
                continuous = false;
                Serial.print("CMD axis=");
                Serial.print(axis);
                Serial.println(" pulse");
                torque_sign_pulse(foc, sensor, electrical_offset, axis, sign, vq_mag);
            } else if (c == '+' || c == '=') {
                vq_mag = torque_sign_safe_vq(vq_mag + L10_VQ_STEP);
                Serial.print("CMD axis=");
                Serial.print(axis);
                Serial.print(" vq_mag_V=");
                Serial.print(vq_mag, 2);
                Serial.print(" (ceiling ");
                Serial.print(vq_ceiling, 2);
                Serial.println(")");
            } else if (c == '-' || c == '_') {
                vq_mag -= L10_VQ_STEP;
                if (vq_mag < 0.0f) vq_mag = 0.0f;
                Serial.print("CMD axis=");
                Serial.print(axis);
                Serial.print(" vq_mag_V=");
                Serial.println(vq_mag, 2);
            } else if (c == ' ' || c == 'x' || c == 'X') {
                continuous = false;
                driver.disable();
                driver_live = false;
                Serial.print("ESTOP axis=");
                Serial.print(axis);
                Serial.println(" — driver disabled. 'g'/'p' re-arms, 'q' ends.");
            } else if (c == 'q' || c == 'Q') {
                Serial.print("CMD axis=");
                Serial.print(axis);
                Serial.println(" end");
                break;
            }
        }
    }

    driver.disable();

    Serial.print("RESULT torque_sign_");
    Serial.print(axis[0] == 'X' ? "x" : "y");
    Serial.print(" vq_mag_V=");
    Serial.print(vq_mag, 2);
    Serial.print(" est_phase_i_A=");
    Serial.print(vq_mag / GM2804_PHASE_RESISTANCE, 3);
    Serial.println(" CHECK (operator confirms: continuous +Vq spins one way, 's' reverses, "
                   "+/- pulse kicks flip with sign, phase I<limit)");

    return true;
}
