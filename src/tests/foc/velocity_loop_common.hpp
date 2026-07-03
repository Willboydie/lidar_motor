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

// L13 — Velocity loop PID (per axis).
//
// Closes the middle cascade: velocity error -> PI -> target_q_current -> inner
// current loop (STM32G4: measured PI; Teensy: estimate_Vq voltage model).
// ControlMode = VELOCITY; angle loop bypassed.
//
// Operator feeds velocity steps and ramps; stream target vs measured velocity,
// velocity error, target_q_current, Vq. Tune VELOCITY_P/I live.
//
// Pass bar (docs/FOC_bringup_plan.md L13):
//   - Tracks velocity setpoints within tolerance, both directions.
//   - Settles without sustained oscillation/limit-cycle; Vq/current under limits.
//   - Record final VELOCITY_P/I gains.
//
// STM32G4 entry: L12 passed (current PI proven). Teensy entry: L10 passed (skips L11/L12).

static constexpr float    L13_VEL_REQ     = 2.0f;   // default velocity setpoint [rad/s]
static constexpr float    L13_VEL_STEP    = 0.5f;   // '+'/'-' velocity step [rad/s]
static constexpr uint32_t L13_STREAM_MS   = 50;     // DBG stream period [ms]
static constexpr float    L13_RAMP_RATE   = 1.0f;   // velocity ramp [rad/s^2]

inline void velocity_loop_print_spec(const char* axis) {
    Serial.print("SPEC axis=");
    Serial.print(axis);
    Serial.print(" vel_req_rads=");
    Serial.print(L13_VEL_REQ, 2);
    Serial.print(" vel_limit_rads=");
    Serial.print(VELOCITY_UPPER_LIMIT, 1);
    Serial.print(" VELOCITY_P=");
    Serial.print(VELOCITY_P, 3);
    Serial.print(" VELOCITY_I=");
    Serial.println(VELOCITY_I, 3);
    Serial.print("SPEC loop_Hz current=");
    Serial.print(CURRENT_LOOP_HZ);
    Serial.print(" velocity=");
    Serial.print(VELOCITY_LOOP_HZ);
    Serial.print(" angle=");
    Serial.println(ANGLE_LOOP_HZ);
}

inline void velocity_loop_bench_prompt(const char* axis) {
    Serial.println();
    Serial.print("SAFETY — L13 velocity loop, axis ");
    Serial.println(axis);
    Serial.println("  Rotor FREE and UNLOADED (>=1 turn clearance). Non-zero velocity WILL spin it.");
    Serial.println("  VM ON (12 V). Bench supply current limit ~1.0-1.5 A.");
    Serial.println("  Other axis EN must stay LOW. E-stop: ' ' or 'x'.");
    velocity_loop_print_spec(axis);
    Serial.println("CONTROLS: '0'=vel=0  '1'/'2'=vel steps  'r'=ramp  's'=flip  '+'/'-'=vel  "
                   "'p'/'i'=bump P/I  ' '/'x'=E-STOP  'q'=end.");
    wiring_wait_for_confirm("rotor free/unloaded, VM on + current-limited, other axis EN low");
}

inline void velocity_loop_run(FOCController& foc, PlatformPWMDriver& driver, const char* axis) {
    float vel_cmd = 0.0f;
    int sign = +1;
    bool ramp = false;
    float vel_p = VELOCITY_P;
    float vel_i = VELOCITY_I;

    foc.setControlMode(VELOCITY);
    driver.enable();
    foc.enable();

    Serial.print("MOD axis=");
    Serial.print(axis);
    Serial.println(" phase=velocity_loop — VELOCITY mode, full cascade (minus angle).");
    Serial.println("DBG columns: axis,t_ms,mode,tgt_vel,meas_vel,vel_err,tgt_iq,Vq,mech_rad");

    uint32_t last_stream_ms = millis();
    uint32_t last_ramp_ms = millis();

    while (true) {
        if (ramp) {
            const uint32_t now_ms = millis();
            const float dt = (float)(now_ms - last_ramp_ms) * 1e-3f;
            last_ramp_ms = now_ms;
            vel_cmd += (float)sign * L13_RAMP_RATE * dt;
            vel_cmd = _constrain(vel_cmd, VELOCITY_LOWER_LIMIT, VELOCITY_UPPER_LIMIT);
            foc.setTargetVelocity(vel_cmd);
        }

        if (!foc.loop()) {
            foc.disable();
            driver.disable();
            Serial.println("RESULT velocity_loop FAIL loop=0");
            return;
        }

        const uint32_t now_ms = millis();
        if (now_ms - last_stream_ms >= L13_STREAM_MS) {
            last_stream_ms = now_ms;
            const char* mode = ramp ? "ramp" : "step";
            Serial.print("DBG ");
            Serial.print(axis);
            Serial.print(",");
            Serial.print(now_ms);
            Serial.print(",");
            Serial.print(mode);
            Serial.print(",");
            Serial.print(foc.getTargetVelocity(), 4);
            Serial.print(",");
            Serial.print(foc.getAngularVelocity(), 4);
            Serial.print(",");
            Serial.print(foc.getVelocityError(), 4);
            Serial.print(",");
            Serial.print(foc.getTargetQCurrent(), 4);
            Serial.print(",");
            Serial.print(foc.getVq(), 3);
            Serial.print(",");
            Serial.println(foc.getMechanicalAngle(), 4);
        }

        if (Serial.available()) {
            const char c = (char)Serial.read();
            if (c == '0') {
                vel_cmd = 0.0f; ramp = false;
                foc.setTargetVelocity(0.0f);
                Serial.println("CMD vel=0");
            } else if (c == '1') {
                vel_cmd = (float)sign * L13_VEL_REQ; ramp = false;
                foc.setTargetVelocity(vel_cmd);
                Serial.print("CMD vel_rads="); Serial.println(vel_cmd, 3);
            } else if (c == '2') {
                vel_cmd = (float)sign * 2.0f * L13_VEL_REQ; ramp = false;
                foc.setTargetVelocity(vel_cmd);
                Serial.print("CMD vel_rads="); Serial.println(vel_cmd, 3);
            } else if (c == 's' || c == 'S') {
                sign = -sign;
                vel_cmd = -vel_cmd;
                foc.setTargetVelocity(vel_cmd);
                Serial.print("CMD flip vel_rads="); Serial.println(vel_cmd, 3);
            } else if (c == 'r' || c == 'R') {
                ramp = !ramp;
                last_ramp_ms = millis();
                Serial.print("CMD ramp="); Serial.println(ramp ? 1 : 0);
            } else if (c == '+' || c == '=') {
                vel_cmd = _constrain(vel_cmd + (float)sign * L13_VEL_STEP, VELOCITY_LOWER_LIMIT, VELOCITY_UPPER_LIMIT);
                ramp = false;
                foc.setTargetVelocity(vel_cmd);
                Serial.print("CMD vel_rads="); Serial.println(vel_cmd, 3);
            } else if (c == '-' || c == '_') {
                vel_cmd = _constrain(vel_cmd - (float)sign * L13_VEL_STEP, VELOCITY_LOWER_LIMIT, VELOCITY_UPPER_LIMIT);
                ramp = false;
                foc.setTargetVelocity(vel_cmd);
                Serial.print("CMD vel_rads="); Serial.println(vel_cmd, 3);
            } else if (c == 'p' || c == 'P') {
                vel_p += 0.1f;
                foc.setVelocityP(vel_p);
                Serial.print("CMD VELOCITY_P="); Serial.println(vel_p, 3);
            } else if (c == 'i' || c == 'I') {
                vel_i += 1.0f;
                foc.setVelocityI(vel_i);
                Serial.print("CMD VELOCITY_I="); Serial.println(vel_i, 1);
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

    Serial.print("RESULT velocity_loop_");
    Serial.print(axis[0] == 'X' ? "x" : "y");
    Serial.print(" VELOCITY_P=");
    Serial.print(vel_p, 3);
    Serial.print(" VELOCITY_I=");
    Serial.print(vel_i, 1);
    Serial.println(" CHECK (operator confirms: tracks vel steps both dirs, settles, under limits)");
}
