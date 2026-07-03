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

// L12 — Current-target PI validation (STM32G4 only).
//
// L10 proved commutation sign with fixed Vq. L11 proved the current sense CAPTURES.
// This gate closes the inner measured-current PI: commanded Iq/Id -> PI -> Vd/Vq ->
// setPhaseVoltage(), commutating off the live encoder. The velocity/angle cascade
// is bypassed via FOCController::currentLoopStep().
//
// Operator feeds Iq steps and ramps; the measured Iq (from getFOCCurrents) should
// track within tolerance, with Id ~ 0. On a free rotor, holding a non-zero Iq will
// spin the rotor (expected) — the pass bar is current tracking, not holding still.
//
// Pass bar (docs/FOC_bringup_plan.md L12):
//   - Measured Iq tracks commanded Iq within tolerance, both signs.
//   - Id stays near 0; Vq/Vd and phase current under limits; no runaway oscillation.
//   - Record final CURRENT_P/I gains.
//
// Abort: Iq diverges or oscillates uncontrollably => back to L11 (capture) or L10
// (commutation sign). Tune CURRENT_P/I live with 'P'/'I' keys.

static constexpr float    L12_IQ_REQ      = 0.15f;  // default Iq command [A]
static constexpr float    L12_IQ_STEP     = 0.05f;  // '+'/'-' Iq step [A]
static constexpr uint32_t L12_STREAM_MS   = 50;     // DBG stream period [ms]
static constexpr uint32_t L12_STEP_DWELL_MS = 2000; // auto Iq step dwell [ms]

inline void current_loop_pi_print_spec(const char* axis) {
    Serial.print("SPEC axis=");
    Serial.print(axis);
    Serial.print(" Iq_req_A=");
    Serial.print(L12_IQ_REQ, 3);
    Serial.print(" current_limit_A=");
    Serial.print(MOTOR_CURRENT_LIMIT, 2);
    Serial.print(" CURRENT_P=");
    Serial.print(CURRENT_P, 3);
    Serial.print(" CURRENT_I=");
    Serial.println(CURRENT_I, 1);
}

inline void current_loop_pi_bench_prompt(const char* axis) {
    Serial.println();
    Serial.print("SAFETY — L12 current-target PI, axis ");
    Serial.println(axis);
    Serial.println("  Rotor FREE and UNLOADED. Non-zero Iq WILL spin the free rotor — expected.");
    Serial.println("  VM ON (12 V). Bench supply current limit ~1.0-1.5 A (fold-back armed).");
    Serial.println("  Other axis EN must stay LOW. E-stop: ' ' or 'x'.");
    current_loop_pi_print_spec(axis);
    Serial.println("CONTROLS: '0'=Iq=0  '1'/'2'/'3'=Iq steps (+/-)  'r'=ramp  'P'/'I'=tune PI  '+'/'-'=Iq  ' '/'x'=E-STOP  'q'=end.");
    wiring_wait_for_confirm("rotor free/unloaded, VM on + current-limited, other axis EN low");
}

inline void current_loop_pi_run(FOCController& foc, PlatformPWMDriver& driver,
                                InlineCurrentSenseSTM32G4& cs, const char* axis) {
    float iq_cmd = L12_IQ_REQ;
    int sign = +1;
    bool ramp = false;
    const float ramp_rate = 0.05f;  // A/s

    foc.setCurrentSense(cs);
    driver.enable();
    foc.enable();

    Serial.print("MOD axis=");
    Serial.print(axis);
    Serial.println(" phase=current_pi — closed-loop Iq via measured-current PI.");
    Serial.println("DBG columns: axis,t_ms,mode,iq_cmd_A,Iq_meas_A,Id_meas_A,Vq,Vd,mech_rad,filt_vel");

    uint32_t last_stream_ms = millis();
    uint32_t last_ramp_ms = millis();

    while (true) {
        if (ramp) {
            const uint32_t now_ms = millis();
            const float dt = (float)(now_ms - last_ramp_ms) * 1e-3f;
            last_ramp_ms = now_ms;
            iq_cmd += (float)sign * ramp_rate * dt;
            iq_cmd = _constrain(iq_cmd, -MOTOR_CURRENT_LIMIT, MOTOR_CURRENT_LIMIT);
        }

        if (!foc.currentLoopStep(iq_cmd, 0.0f)) {
            driver.disable();
            Serial.println("RESULT current_loop_pi FAIL sensor_update=0");
            return;
        }

        const float Iq = foc.getMeasuredQCurrent();
        const float Id = foc.getMeasuredDCurrent();

        const uint32_t now_ms = millis();
        if (now_ms - last_stream_ms >= L12_STREAM_MS) {
            last_stream_ms = now_ms;
            const char* mode = ramp ? "ramp" : "step";
            Serial.print("DBG ");
            Serial.print(axis);
            Serial.print(",");
            Serial.print(now_ms);
            Serial.print(",");
            Serial.print(mode);
            Serial.print(",");
            Serial.print(iq_cmd, 4);
            Serial.print(",");
            Serial.print(Iq, 4);
            Serial.print(",");
            Serial.print(Id, 4);
            Serial.print(",");
            Serial.print(foc.getVq(), 3);
            Serial.print(",");
            Serial.print(foc.getVd(), 3);
            Serial.print(",");
            Serial.print(foc.getMechanicalAngle(), 4);
            Serial.print(",");
            Serial.println(foc.getAngularVelocity(), 4);
        }

        if (Serial.available()) {
            const char c = (char)Serial.read();
            if (c == '0') {
                iq_cmd = 0.0f; ramp = false;
                Serial.println("CMD Iq=0");
            } else if (c == '1') {
                iq_cmd = (float)sign * L12_IQ_REQ; ramp = false;
                Serial.print("CMD Iq_A="); Serial.println(iq_cmd, 3);
            } else if (c == '2') {
                iq_cmd = (float)sign * 2.0f * L12_IQ_REQ; ramp = false;
                Serial.print("CMD Iq_A="); Serial.println(iq_cmd, 3);
            } else if (c == '3') {
                iq_cmd = (float)sign * 3.0f * L12_IQ_REQ; ramp = false;
                Serial.print("CMD Iq_A="); Serial.println(iq_cmd, 3);
            } else if (c == 's' || c == 'S') {
                sign = -sign;
                iq_cmd = -iq_cmd;
                Serial.print("CMD flip sign, Iq_A="); Serial.println(iq_cmd, 3);
            } else if (c == 'r' || c == 'R') {
                ramp = !ramp;
                last_ramp_ms = millis();
                Serial.print("CMD ramp="); Serial.println(ramp ? 1 : 0);
            } else if (c == '+' || c == '=') {
                iq_cmd = _constrain(iq_cmd + (float)sign * L12_IQ_STEP, -MOTOR_CURRENT_LIMIT, MOTOR_CURRENT_LIMIT);
                ramp = false;
                Serial.print("CMD Iq_A="); Serial.println(iq_cmd, 3);
            } else if (c == '-' || c == '_') {
                iq_cmd = _constrain(iq_cmd - (float)sign * L12_IQ_STEP, -MOTOR_CURRENT_LIMIT, MOTOR_CURRENT_LIMIT);
                ramp = false;
                Serial.print("CMD Iq_A="); Serial.println(iq_cmd, 3);
            } else if (c == 'p' || c == 'P') {
                static float cp = CURRENT_P;
                cp += 0.1f;
                foc.setCurrentP(cp);
                Serial.print("CMD CURRENT_P="); Serial.println(cp, 3);
            } else if (c == 'i' || c == 'I') {
                static float ci = CURRENT_I;
                ci += 5.0f;
                foc.setCurrentI(ci);
                Serial.print("CMD CURRENT_I="); Serial.println(ci, 1);
            } else if (c == ' ' || c == 'x' || c == 'X') {
                foc.disable();
                driver.disable();
                Serial.println("ESTOP — re-enable with any drive cmd or 'q' to end.");
            } else if (c == 'q' || c == 'Q') {
                break;
            }
        }
    }

    foc.disable();
    driver.disable();

    Serial.print("RESULT current_loop_pi_");
    Serial.print(axis[0] == 'X' ? "x" : "y");
    Serial.println(" CHECK (operator confirms: Iq tracks command both signs, Id~0, "
                   "no runaway, under limits — record CURRENT_P/I)");
}

#endif // STM32G4xx
