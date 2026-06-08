#pragma once

#include "../../sensor/AS5048.hpp"
#include "../../pwm_driver/PWMDriver.hpp"
#include "../../motor/GM2804.hpp"
#include "../../utils/constants.h"
#include "../../utils/tweakable_params.h"
#include "../../utils/quick_defs.h"
#include <Arduino.h>
#include <cmath>

#if defined(STM32G4xx)
#include "../../pwm_driver/stm32g4/PWMDriverSTM32G4.hpp"
#else
#include "../../pwm_driver/teensy41/PWMDriverTeensy41.hpp"
#endif

// ============================================================================
// Autonomous electrical-offset calibration (stiction-bracket method)
// ----------------------------------------------------------------------------
// Why not continuous rotation: runtime evidence showed this unloaded rotor
// cannot be held in synchronous lock by an open-loop rotating field. Static
// friction (stiction) exceeds the available motor torque, so the breakaway
// dead-band is wider than pi electrical; the field just slips past and ratchets
// the rotor in ONE fixed direction regardless of commanded direction. So we
// never rotate continuously.
//
// Instead, at each target electrical angle theta we HOLD a static d-axis field
// (one stable detent) and bracket the true alignment:
//   * approach theta from BELOW (pull the rotor under theta, creep up, settle)
//       -> rotor sticks just below true alignment            -> mech_lo
//   * approach theta from ABOVE (pull the rotor over theta, creep down, settle)
//       -> rotor sticks just above true alignment            -> mech_hi
// The midpoint (mech_lo, mech_hi) cancels the (symmetric) stiction, giving the
// true aligned mechanical angle at theta. The implied offset is then
//
//       implied = wrap(mech_mid * pole_pairs * sensor_direction - theta)
//
// Targets are spread across a full MECHANICAL revolution (several per pole pair)
// and the implied offsets are CIRCULAR-averaged, so eccentricity / per-pole
// variation also averages out. This is the autonomous version of the manual
// nudge procedure in electrical_offset_test.hpp.
//
// Diagnostics (parseable, prefix "TGT"): per target the two bracket readings,
// the bracket width (=stiction span), the midpoint, the implied offset, and the
// running circular-mean. A small final circ_std (vs ~0.9 for the spin method)
// is the success criterion.
// ============================================================================

namespace electrical_offset_autocal {

constexpr int TARGETS_PER_POLE = 6;          // targets per pole pair (fine: averages e(theta) harmonics 1-5)
constexpr float CAL_VOLTAGE = 6.0f;          // requested drive voltage (clamped to current ceiling)

// A large nudge (> stiction dead-band) is needed to reliably break the rotor
// free on BOTH sides; smaller offsets leave one side stuck -> biased midpoint.
constexpr float BRACKET_OFFSET = 4.0f;       // elec rad pulled to each side
constexpr float SLEW_STEP = 0.02f;           // elec rad per micro-step when repositioning
constexpr unsigned long SLEW_GAP_MS = 3;
constexpr float APPROACH_STEP = 0.01f;       // slow final creep onto the target (low overshoot)
constexpr unsigned long APPROACH_GAP_MS = 8;
constexpr unsigned long HOLD_SETTLE_MS = 900;  // settle at the target before capturing
constexpr int CAPTURE_SAMPLES = 20;
constexpr unsigned long CAPTURE_GAP_MS = 3;
constexpr unsigned long INITIAL_SNAP_MS = 1500;

struct TargetResult {
    float implied;
    float mech_mid;
    float mech_lo;
    float mech_hi;
    float bracket_width;  // mechanical rad between the two bracket readings
};

// Mirror of FOCController::setPhaseVoltage with Vq = 0: applied field at elec_angle.
inline void applyAlignVoltage(PWMDriver& driver, float elec_angle, float Vd) {
    float s, c;
    fast_sincos(normalise_angle(elec_angle), &s, &c);

    float Valpha = c * Vd;  // Vq = 0
    float Vbeta = s * Vd;

    float Va = Valpha;
    float Vb = -0.5f * Valpha + _SQRT3_2 * Vbeta;
    float Vc = -0.5f * Valpha - _SQRT3_2 * Vbeta;

    float center = driver.getVoltageLimit() / 2.0f;
    float Vmin = min(Va, min(Vb, Vc));
    float Vmax = max(Va, max(Vb, Vc));
    center -= (Vmax + Vmin) / 2.0f;

    driver.setPwm(Va + center, Vb + center, Vc + center);
}

inline float circularMean2(float a, float b) {
    return normalise_angle(atan2f(sinf(a) + sinf(b), cosf(a) + cosf(b)));
}

// Move the held field angle from -> to in small steps so the rotor is dragged
// along. Absolute (cumulative) angles; applyAlignVoltage normalises internally.
inline float slewField(PWMDriver& driver, float from, float to, float V,
                       float step, unsigned long gap_ms) {
    float a = from;
    float delta = to - from;
    int steps = (int)(fabsf(delta) / step) + 1;
    float inc = delta / (float)steps;
    for (int i = 0; i < steps; i++) {
        a += inc;
        applyAlignVoltage(driver, a, V);
        delay(gap_ms);
    }
    applyAlignVoltage(driver, to, V);
    return to;
}

// Circular mean of the raw mechanical angle (robust to the 0/2pi wrap).
inline float captureMech(AS5048& sensor) {
    float sc = 0.0f, ss = 0.0f;
    for (int i = 0; i < CAPTURE_SAMPLES; i++) {
        sensor.update();
        float r = sensor.getLatestRotorAngle();
        sc += cosf(r);
        ss += sinf(r);
        delay(CAPTURE_GAP_MS);
    }
    return normalise_angle(atan2f(ss, sc));
}

// Bracket the true alignment at electrical angle theta from both sides.
inline TargetResult bracketTarget(AS5048& sensor, PWMDriver& driver, Motor& motor,
                                  float& current_phi, float theta, float V) {
    const float pp = motor.pole_pairs;
    const float sdir = motor.sensor_direction;

    // Approach from BELOW: pull the rotor well under theta, then creep up and settle.
    current_phi = slewField(driver, current_phi, theta - BRACKET_OFFSET, V, SLEW_STEP, SLEW_GAP_MS);
    current_phi = slewField(driver, current_phi, theta, V, APPROACH_STEP, APPROACH_GAP_MS);
    delay(HOLD_SETTLE_MS);
    float mech_lo = captureMech(sensor);

    // Approach from ABOVE: pull the rotor well over theta, then creep down and settle.
    current_phi = slewField(driver, current_phi, theta + BRACKET_OFFSET, V, SLEW_STEP, SLEW_GAP_MS);
    current_phi = slewField(driver, current_phi, theta, V, APPROACH_STEP, APPROACH_GAP_MS);
    delay(HOLD_SETTLE_MS);
    float mech_hi = captureMech(sensor);

    float mech_mid = circularMean2(mech_lo, mech_hi);
    float implied = normalise_angle(mech_mid * pp * sdir - theta);

    TargetResult tr;
    tr.implied = implied;
    tr.mech_mid = mech_mid;
    tr.mech_lo = mech_lo;
    tr.mech_hi = mech_hi;
    tr.bracket_width = findAngularError(mech_hi, mech_lo);
    return tr;
}

inline float runMotor(const char* label, AS5048& sensor, PWMDriver& driver, Motor& motor) {
    Serial.println();
    Serial.println(String("################  ") + label + " auto-calibration (stiction-bracket)  ################");

    const float v_ceiling = MOTOR_CURRENT_LIMIT * motor.phase_resistance;  // back-EMF ~0 (no spin)
    float V = _constrain(CAL_VOLTAGE, 0.0f, min(v_ceiling, (float)MOTOR_VOLTAGE_LIMIT));
    const int pp = (int)motor.pole_pairs;
    const int n_targets = pp * TARGETS_PER_POLE;
    const float increment = (float)pp * _2PI / (float)n_targets;  // spans one mechanical rev

    Serial.println(String("drive=") + String(V, 2) + "V (ceiling " + String(v_ceiling, 2) +
                   "V @ " + String(MOTOR_CURRENT_LIMIT, 2) + "A) | pp=" + String(pp) + " dir=" +
                   String((int)motor.sensor_direction) + " | targets=" + String(n_targets) +
                   " | stored offset=" + String(motor.electrical_offset, 4));
    Serial.println("  TGT k theta mech_lo mech_hi bracket mech_mid implied run_mean");

    driver.enable();
    float current_phi = 0.0f;
    applyAlignVoltage(driver, current_phi, V);
    delay(INITIAL_SNAP_MS);  // snap to the first detent

    float sum_cos = 0.0f, sum_sin = 0.0f;
    float sum_bracket = 0.0f, max_bracket = 0.0f;

    for (int k = 0; k < n_targets; k++) {
        float theta = (float)k * increment;
        TargetResult tr = bracketTarget(sensor, driver, motor, current_phi, theta, V);

        sum_cos += cosf(tr.implied);
        sum_sin += sinf(tr.implied);
        sum_bracket += fabsf(tr.bracket_width);
        if (fabsf(tr.bracket_width) > max_bracket) max_bracket = fabsf(tr.bracket_width);

        float run_mean = normalise_angle(atan2f(sum_sin, sum_cos));
        Serial.println(String("  TGT ") + String(k) + " " + String(normalise_angle(theta), 3) +
                       " " + String(tr.mech_lo, 4) + " " + String(tr.mech_hi, 4) +
                       " " + String(tr.bracket_width, 4) + " " + String(tr.mech_mid, 4) +
                       " " + String(tr.implied, 4) + " " + String(run_mean, 4));
    }

    driver.setPwm(0.0f, 0.0f, 0.0f);
    driver.disable();

    float offset = normalise_angle(atan2f(sum_sin, sum_cos));
    float R = sqrtf(sum_cos * sum_cos + sum_sin * sum_sin) / (float)n_targets;
    float circ_std = sqrtf(-2.0f * logf(R > 1e-6f ? R : 1e-6f));

    Serial.println();
    Serial.println(String("==== ") + label + " RESULT ====");
    Serial.println(String(">>> electrical offset = ") + String(offset, 4) + " rad");
    Serial.println(String("consistency: circ_std=") + String(circ_std, 4) + " rad (R=" +
                   String(R, 4) + "; lower=better, spin method was ~0.9)");
    Serial.println(String("mean stiction bracket=") + String(sum_bracket / (float)n_targets, 4) +
                   " rad mech (max=" + String(max_bracket, 4) +
                   "); electrical ~" + String((sum_bracket / (float)n_targets) * pp, 3) + " rad");
    Serial.println(String("change vs stored offset = ") +
                   String(findAngularError(offset, motor.electrical_offset), 4) + " rad");
    return offset;
}

inline void beginSerial() {
#if defined(STM32G4xx)
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        delay(10);
    }
#else
    Serial.begin(BAUD_RATE);
    while (!Serial) {
        delay(100);
    }
#endif
}

}  // namespace electrical_offset_autocal

inline void electrical_offset_autocal_test() {
    using namespace electrical_offset_autocal;

    beginSerial();
    delay(1000);
#if defined(STM32G4xx)
    Serial.println("Platform: STM32G4");
#else
    Serial.println("Platform: Teensy 4.1");
#endif
    Serial.println("=== Autonomous Electrical Offset Calibration (stiction-bracket) ===");
    Serial.println("Motors must be UNLOADED and free to wiggle ~1 turn. No continuous spin.");

    AS5048 sensorX(SENSOR_SPI_CS_X);
    AS5048 sensorY(SENSOR_SPI_CS_Y);
#if defined(STM32G4xx)
    PWMDriverSTM32G4 driverX(N1_X, N2_X, N3_X, EN_X);
    PWMDriverSTM32G4 driverY(N1_Y, N2_Y, N3_Y, EN_Y);
#else
    PWMDriverTeensy41 driverX(N1_X, N2_X, N3_X, EN_X);
    PWMDriverTeensy41 driverY(N1_Y, N2_Y, N3_Y, EN_Y);
#endif
    GM2804 motorX(1, SENSOR_OFFSET_X);
    GM2804 motorY(2, SENSOR_OFFSET_Y);

    if (!sensorX.init() || !sensorY.init()) {
        Serial.println("Sensor init failed.");
        return;
    }
    if (!driverX.init() || !driverY.init()) {
        Serial.println("Driver init failed.");
        return;
    }

    driverX.enable();
    driverX.setPwm(0.0f, 0.0f, 0.0f);
    driverY.enable();
    driverY.setPwm(0.0f, 0.0f, 0.0f);

    float offsetX = runMotor("MOTOR X", sensorX, driverX, motorX);
    float offsetY = runMotor("MOTOR Y", sensorY, driverY, motorY);

    Serial.println();
    Serial.println("================ CALIBRATION SUMMARY ================");
    Serial.println(String("MOTOR X electrical offset: ") + String(offsetX, 4) +
                   " rad  (-> constants.h SENSOR_OFFSET_X)");
    Serial.println(String("MOTOR Y electrical offset: ") + String(offsetY, 4) +
                   " rad  (-> constants.h SENSOR_OFFSET_Y)");
    Serial.println("Both motors de-energised.");

    while (true) {
        delay(1000);
    }
}
