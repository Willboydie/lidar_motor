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
#include "../current_sense/current_sense_common.hpp"
#endif

// L14 — Angle (position) loop PID (per axis).
//
// Closes the outer cascade: angle error -> PI -> velocity setpoint -> velocity
// loop (L13) -> inner current loop (STM32G4: measured PI; Teensy: estimate_Vq).
// ControlMode = ANGLE; findAngularError() gives shortest-path wrap handling and
// the angle PID output is clamped to VELOCITY_LOWER/UPPER_LIMIT.
//
// Operator commands angle steps in both directions (including across the 0/2pi
// wrap), then hand-perturbs the rotor to confirm it holds. Stream target vs
// measured angle, angular error, and inner-loop signals; tune ANGLE_P/I/D live.
//
// Pass bar (docs/FOC_bringup_plan.md L14):
//   - Settles to target within tolerance, both directions, no sustained oscillation.
//   - Holds position under a modest disturbance.
//   - Wrap handled correctly (shortest path; no full-turn unwind). Record final gains.
//
// Entry: L13 passed (stable velocity loop) for this axis/target.

static constexpr float    L14_ANGLE_STEP  = _PI_2;   // '1'/'2' step size [rad]
static constexpr float    L14_ANGLE_NUDGE = 0.1f;    // '+'/'-' nudge [rad]
static constexpr uint32_t L14_STREAM_MS   = 50;      // DBG stream period [ms]

inline void angle_loop_print_spec(const char* axis) {
    Serial.print("SPEC axis=");
    Serial.print(axis);
    Serial.print(" angle_step_rad=");
    Serial.print(L14_ANGLE_STEP, 3);
    Serial.print(" vel_limit_rads=");
    Serial.print(VELOCITY_UPPER_LIMIT, 1);
    Serial.print(" ANGLE_P=");
    Serial.print(ANGLE_P, 3);
    Serial.print(" ANGLE_I=");
    Serial.print(ANGLE_I, 3);
    Serial.print(" ANGLE_D=");
    Serial.println(ANGLE_D, 3);
    Serial.print("SPEC loop_Hz current=");
    Serial.print(CURRENT_LOOP_HZ);
    Serial.print(" velocity=");
    Serial.print(VELOCITY_LOOP_HZ);
    Serial.print(" angle=");
    Serial.println(ANGLE_LOOP_HZ);
}

inline void angle_loop_bench_prompt(const char* axis) {
    Serial.println();
    Serial.print("SAFETY — L14 angle loop, axis ");
    Serial.println(axis);
    Serial.println("  Rotor FREE and UNLOADED (>=1 turn clearance). Angle steps WILL slew it.");
    Serial.println("  VM ON (12 V). Bench supply current limit ~1.0-1.5 A.");
    Serial.println("  Other axis EN must stay LOW. E-stop: ' ' or 'x'.");
    angle_loop_print_spec(axis);
    Serial.println("CONTROLS: '0'=hold-here  '1'=+step  '2'=-step  'w'=+wrap-step  "
                   "'+'/'-'=nudge  'd'=disturb-hint  'p'/'i'/'k'=bump P/I/D  "
                   "' '/'x'=E-STOP  'q'=end.");
    wiring_wait_for_confirm("rotor free/unloaded, VM on + current-limited, other axis EN low");
}

inline void angle_loop_run(FOCController& foc, PlatformPWMDriver& driver, const char* axis) {
    float ang_p = ANGLE_P;
    float ang_i = ANGLE_I;
    float ang_d = ANGLE_D;

    foc.setControlMode(ANGLE);
    driver.enable();
    foc.enable();

    // Hold the position we powered up at; commands step relative to this.
    if (!foc.loop()) {
        foc.disable();
        driver.disable();
        Serial.println("RESULT angle_loop FAIL loop=0");
        return;
    }
    float ang_cmd = foc.getMechanicalAngle();
    foc.setTargetAngle(ang_cmd);

    Serial.print("MOD axis=");
    Serial.print(axis);
    Serial.println(" phase=angle_loop — ANGLE mode, full cascade (shortest-path wrap).");
    Serial.println("DBG columns: axis,t_ms,tgt_ang,mech_ang,ang_err,tgt_vel,meas_vel,tgt_iq,Vq");

    uint32_t last_stream_ms = millis();

    while (true) {
        if (!foc.loop()) {
            foc.disable();
            driver.disable();
            Serial.println("RESULT angle_loop FAIL loop=0");
            return;
        }

        const uint32_t now_ms = millis();
        if (now_ms - last_stream_ms >= L14_STREAM_MS) {
            last_stream_ms = now_ms;
            Serial.print("DBG ");
            Serial.print(axis);
            Serial.print(",");
            Serial.print(now_ms);
            Serial.print(",");
            Serial.print(foc.getTargetAngle(), 4);
            Serial.print(",");
            Serial.print(foc.getMechanicalAngle(), 4);
            Serial.print(",");
            Serial.print(foc.getAngularError(), 4);
            Serial.print(",");
            Serial.print(foc.getTargetVelocity(), 4);
            Serial.print(",");
            Serial.print(foc.getAngularVelocity(), 4);
            Serial.print(",");
            Serial.print(foc.getTargetQCurrent(), 4);
            Serial.print(",");
            Serial.println(foc.getVq(), 3);
        }

        if (Serial.available()) {
            const char c = (char)Serial.read();
            if (c == '0') {
                ang_cmd = normalise_angle(foc.getMechanicalAngle());
                foc.setTargetAngle(ang_cmd);
                Serial.print("CMD hold tgt_ang="); Serial.println(ang_cmd, 4);
            } else if (c == '1') {
                ang_cmd = normalise_angle(ang_cmd + L14_ANGLE_STEP);
                foc.setTargetAngle(ang_cmd);
                Serial.print("CMD step+ tgt_ang="); Serial.println(ang_cmd, 4);
            } else if (c == '2') {
                ang_cmd = normalise_angle(ang_cmd - L14_ANGLE_STEP);
                foc.setTargetAngle(ang_cmd);
                Serial.print("CMD step- tgt_ang="); Serial.println(ang_cmd, 4);
            } else if (c == 'w' || c == 'W') {
                // Jump the target to just past the 0/2pi seam relative to the live
                // mechanical angle, forcing a shortest-path crossing of the wrap.
                ang_cmd = normalise_angle(foc.getMechanicalAngle() + L14_ANGLE_STEP);
                foc.setTargetAngle(ang_cmd);
                Serial.print("CMD wrap-step tgt_ang="); Serial.println(ang_cmd, 4);
            } else if (c == '+' || c == '=') {
                ang_cmd = normalise_angle(ang_cmd + L14_ANGLE_NUDGE);
                foc.setTargetAngle(ang_cmd);
                Serial.print("CMD nudge tgt_ang="); Serial.println(ang_cmd, 4);
            } else if (c == '-' || c == '_') {
                ang_cmd = normalise_angle(ang_cmd - L14_ANGLE_NUDGE);
                foc.setTargetAngle(ang_cmd);
                Serial.print("CMD nudge tgt_ang="); Serial.println(ang_cmd, 4);
            } else if (c == 'd' || c == 'D') {
                Serial.println("HINT hand-push the rotor off target now; it should return and hold.");
            } else if (c == 'p' || c == 'P') {
                ang_p += 0.1f;
                foc.setAngleP(ang_p);
                Serial.print("CMD ANGLE_P="); Serial.println(ang_p, 3);
            } else if (c == 'i' || c == 'I') {
                ang_i += 0.1f;
                foc.setAngleI(ang_i);
                Serial.print("CMD ANGLE_I="); Serial.println(ang_i, 3);
            } else if (c == 'k' || c == 'K') {
                ang_d += 0.001f;
                foc.setAngleD(ang_d);
                Serial.print("CMD ANGLE_D="); Serial.println(ang_d, 4);
            } else if (c == ' ' || c == 'x' || c == 'X') {
                foc.disable();
                driver.disable();
                Serial.println("ESTOP");
            } else if (c == 'q' || c == 'Q') {
                break;
            }
        }
    }

    foc.disable();
    driver.disable();

    Serial.print("RESULT angle_loop_");
    Serial.print(axis[0] == 'X' ? "x" : "y");
    Serial.print(" ANGLE_P=");
    Serial.print(ang_p, 3);
    Serial.print(" ANGLE_I=");
    Serial.print(ang_i, 3);
    Serial.print(" ANGLE_D=");
    Serial.print(ang_d, 4);
    Serial.println(" CHECK (operator confirms: settles both dirs, holds under disturbance, wrap OK)");
}
