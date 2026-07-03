#pragma once

#include <Arduino.h>
#include <cmath>
#include "../../foc_controller/FOCController.hpp"
#include "../../motor/GM2804.hpp"
#include "../../sensor/AS5048.hpp"
#include "../../pwm_driver/PlatformPWMDriver.hpp"
#include "../wiring/wiring_common.hpp"

// L8 — Open-loop forced commutation (first motion).
//
// Drives a *synthesized* rotating electrical angle (Vd=0, fixed Vq) straight
// into setPhaseVoltage(). The encoder is IGNORED for commutation — it is read
// only as a reference so the operator can see the commanded field versus the
// encoder-implied electrical angle and their alignment error. This isolates
// two things most likely wrong in a fresh build: phase wiring ORDER and the
// modulation chain, on the real motor.
//
// Pass bar (per docs/FOC_bringup_plan.md L8):
//   - Smooth rotation, no 60° ratcheting that signals a swapped phase.
//   - Direction reverses when the commanded ramp reverses.
//   - Current stays under MOTOR_CURRENT_LIMIT at the chosen Vq.
//
// Expected caveat: these motors are stiction-dominated unloaded and will
// *ratchet* rather than lock synchronously to a continuously rotating field.
// The bar is "smooth, reversible motion proving phase order + modulation",
// NOT perfect synchronous lock. Precise offset/alignment is L9's job.

static constexpr float L8_VQ = 2.0f;              // fixed Vq for forced commutation [V]
static constexpr float L8_ELEC_RATE_RADS = _2PI;  // commanded electrical rate [rad/s] (1 elec rev/s)
static constexpr uint32_t L8_STREAM_MS = 50;      // DBG stream period [ms] (~20 Hz)

// Encoder-implied electrical angle, for reference only (mirrors
// FOCController::calculate_electrical_angle()). NOT used to commutate here.
inline float open_loop_encoder_elec(float mechanical_angle, float electrical_offset) {
    return normalise_angle(
        mechanical_angle * (float)GM2804_POLE_PAIRS * (float)SENSOR_DIRECTION - electrical_offset);
}

inline void open_loop_print_spec(const char* axis, float electrical_offset) {
    const float mech_rate = L8_ELEC_RATE_RADS / (float)GM2804_POLE_PAIRS;  // rad/s at the shaft
    Serial.print("SPEC axis=");
    Serial.print(axis);
    Serial.print(" Vq_V=");
    Serial.print(L8_VQ, 2);
    Serial.print(" elec_rate_rads=");
    Serial.print(L8_ELEC_RATE_RADS, 3);
    Serial.print(" mech_rate_rads=");
    Serial.print(mech_rate, 3);
    Serial.print(" pole_pairs=");
    Serial.print(GM2804_POLE_PAIRS);
    Serial.print(" sensor_dir=");
    Serial.print(SENSOR_DIRECTION);
    Serial.print(" offset_rad=");
    Serial.println(electrical_offset, 4);
    Serial.print("SPEC current_limit_A=");
    Serial.print(MOTOR_CURRENT_LIMIT, 2);
    Serial.print(" voltage_limit_V=");
    Serial.println(MOTOR_VOLTAGE_LIMIT, 1);
}

inline void open_loop_bench_prompt(const char* axis) {
    Serial.println();
    Serial.print("SAFETY — L8 open-loop forced commutation, axis ");
    Serial.println(axis);
    Serial.println("  Rotor FREE and UNLOADED (>=1 turn of clearance). Encoder ignored for drive.");
    Serial.println("  VM ON (12 V). Bench supply current limit ~1.0-1.5 A (fold-back armed).");
    Serial.println("  Scope: two phase outputs (optional). Other axis driver EN must stay LOW.");
    Serial.println("  E-stop: cut VM or pull EN low if the rotor runs away or current pegs the limit.");
    open_loop_print_spec(axis, axis[0] == 'X' ? SENSOR_OFFSET_X : SENSOR_OFFSET_Y);
    Serial.println("CONTROLS (send on serial): 'r' = reverse ramp direction, 'q' = end this axis.");
    wiring_wait_for_confirm("rotor free/unloaded, VM on + current-limited, other axis EN low");
}

// Core forced-commutation loop. Ramps the commanded electrical angle at a fixed
// rate, streams commanded vs encoder-implied elec angle + alignment error, and
// reverses on 'r' / ends on 'q'. Returns after the operator ends the axis.
inline void open_loop_run(FOCController& foc, PlatformPWMDriver& driver,
                          Sensor& sensor, bool sensor_ok,
                          float electrical_offset, const char* axis) {
    foc.setVd(0.0f);
    foc.setVq(L8_VQ);

    // Start the commanded field at the encoder-implied angle (if we have it) so
    // the very first energisation is a small step, not a full jump.
    float cmd_angle = 0.0f;
    if (sensor_ok) {
        sensor.update();
        cmd_angle = open_loop_encoder_elec(sensor.getLatestRotorAngle(), electrical_offset);
    }

    int dir = +1;

    Serial.print("MOD axis=");
    Serial.print(axis);
    Serial.print(" phase=open_loop dir=");
    Serial.print(dir);
    Serial.println(" — ramping commanded electrical angle. Watch the rotor.");
    Serial.println("DBG columns: axis,t_ms,dir,cmd_elec,enc_elec,align_err_rad,mech_rad,filt_vel_rads");

    driver.enable();

    uint32_t last_us = micros();
    uint32_t last_stream_ms = millis();

    while (true) {
        const uint32_t now_us = micros();
        float dt = (float)(now_us - last_us) * 1e-6f;
        last_us = now_us;
        if (dt < 0.0f || dt > 0.1f) dt = 0.0f;  // guard rollover / stalls

        cmd_angle = normalise_angle(cmd_angle + (float)dir * L8_ELEC_RATE_RADS * dt);
        foc.setElectricalAngle(cmd_angle);
        foc.setPhaseVoltage();

        const uint32_t now_ms = millis();
        if (now_ms - last_stream_ms >= L8_STREAM_MS) {
            last_stream_ms = now_ms;

            float enc_elec = 0.0f;
            float mech = 0.0f;
            float filt_vel = 0.0f;
            float align_err = 0.0f;
            if (sensor_ok) {
                sensor.update();
                mech = sensor.getLatestRotorAngle();
                filt_vel = sensor.getFilteredAngularVelocity();
                enc_elec = open_loop_encoder_elec(mech, electrical_offset);
                align_err = findAngularError(cmd_angle, enc_elec);
            }

            Serial.print("DBG ");
            Serial.print(axis);
            Serial.print(",");
            Serial.print(now_ms);
            Serial.print(",");
            Serial.print(dir);
            Serial.print(",");
            Serial.print(cmd_angle, 4);
            Serial.print(",");
            if (sensor_ok) {
                Serial.print(enc_elec, 4);
                Serial.print(",");
                Serial.print(align_err, 4);
                Serial.print(",");
                Serial.print(mech, 4);
                Serial.print(",");
                Serial.println(filt_vel, 4);
            } else {
                Serial.println("NA,NA,NA,NA");
            }
        }

        if (Serial.available()) {
            const char c = (char)Serial.read();
            if (c == 'r' || c == 'R') {
                dir = -dir;
                Serial.print("CMD axis=");
                Serial.print(axis);
                Serial.print(" reverse dir=");
                Serial.println(dir);
            } else if (c == 'q' || c == 'Q') {
                Serial.print("CMD axis=");
                Serial.print(axis);
                Serial.println(" end");
                break;
            }
        }
    }

    driver.disable();
}
