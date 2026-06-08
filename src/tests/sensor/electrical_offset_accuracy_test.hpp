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
// Electrical offset ACCURACY test (automated, no hand-rotation)
// ----------------------------------------------------------------------------
// electrical_offset_test.hpp *measures* the offset. This test *validates* it.
//
// Method: inject a pure d-axis voltage at a commanded electrical angle. With
// Vq=0 the rotor's permanent-magnet d-axis aligns to the commanded angle, so
// the rotor's TRUE electrical angle becomes the commanded angle (ground truth).
// We then ask the controller's own formula
//
//     estimate = normalise(mech * pole_pairs * sensor_direction - offset)
//
// what it thinks the electrical angle is, and compare it against the command.
//
// Diagnostics printed per step (parseable, prefix "DBG"):
//   cmd     - commanded (true) electrical angle              [rad, 0..2pi)
//   mech    - raw mechanical angle from the sensor           [rad, 0..2pi)
//   est     - controller estimate using the STORED offset    [rad, 0..2pi)
//   err     - wrap(est - cmd) into [-pi, pi]                  [rad]
//   implied - normalise(mech*pp*dir - cmd) = offset this step implies [rad]
//
// Interpretation:
//   * implied roughly CONSTANT across the sweep but != stored offset
//       -> stored offset constant is wrong (e.g. wrap-around averaging bug). H1/H3
//   * implied DRIFTS ~linearly (slope ~= -2) with cmd, |err| grows
//       -> sensor_direction sign is inverted. H2
//   * implied wraps several times per electrical revolution
//       -> pole-pair / period mismatch. H4
// A perfect offset => err ~= 0 and implied ~= stored offset at every step.
// ============================================================================

namespace electrical_offset_accuracy {

constexpr int NUM_PROBES = 24;            // probe points across one electrical rev
constexpr float ELEC_STEP = _2PI / NUM_PROBES;
constexpr float RAMP_STEP = 0.01f;        // rad per micro-step when slewing the field
constexpr unsigned long RAMP_GAP_MS = 3;  // delay between micro-steps while slewing
constexpr unsigned long SETTLE_MS = 350;  // dwell so the rotor catches the command
constexpr int SETTLE_SAMPLES = 20;        // sensor reads averaged at each probe
constexpr unsigned long SAMPLE_GAP_MS = 3;

// Mirror of FOCController::setPhaseVoltage with Vq = 0 (pure d-axis hold).
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

// Exactly the controller's estimator, reproduced so the test is self-contained.
inline float estimateElectricalAngle(float mech, float pole_pairs, float dir, float offset) {
    return normalise_angle(mech * pole_pairs * dir - offset);
}

// Slew the field smoothly from -> to so the rotor physically tracks it without
// slipping a pole. Returns the (normalised) angle actually commanded last.
inline float slewField(PWMDriver& driver, float from, float to, float Vd) {
    float a = from;
    float delta = findAngularError(to, from);  // shortest signed path, [-pi, pi]
    int steps = (int)(fabsf(delta) / RAMP_STEP) + 1;
    float inc = delta / (float)steps;
    for (int i = 0; i < steps; i++) {
        a += inc;
        applyAlignVoltage(driver, a, Vd);
        delay(RAMP_GAP_MS);
    }
    applyAlignVoltage(driver, to, Vd);
    return normalise_angle(to);
}

inline float averageMech(AS5048& sensor) {
    float sum = 0.0f;
    for (int i = 0; i < SETTLE_SAMPLES; i++) {
        sensor.update();
        sum += sensor.getLatestRotorAngle();
        delay(SAMPLE_GAP_MS);
    }
    return sum / (float)SETTLE_SAMPLES;
}

// Run the full sweep for one motor. Prints per-step DBG lines plus a summary.
inline void runMotor(const char* label, AS5048& sensor, PWMDriver& driver, Motor& motor) {
    Serial.println();
    Serial.println(String("################  ") + label + " offset accuracy  ################");
    Serial.println(String("stored offset = ") + String(motor.electrical_offset, 4) +
                   " rad | pole_pairs = " + String((int)motor.pole_pairs) +
                   " | sensor_direction = " + String((int)motor.sensor_direction));

    float Vd = MOTOR_CURRENT_LIMIT * 0.75f * motor.phase_resistance;
    Vd = _constrain(Vd, 0.0f, MOTOR_VOLTAGE_LIMIT);
    Serial.println(String("Vd hold = ") + String(Vd, 2) + " V");

    driver.enable();

    // Charge at electrical 0 and let the rotor snap to the nearest detent.
    applyAlignVoltage(driver, 0.0f, Vd);
    delay(1200);

    Serial.println("DBG_HEADER cmd mech est err implied");

    float prev_cmd = 0.0f;
    float first_mech = 0.0f;
    float last_mech = 0.0f;

    // Circular accumulators for the implied offset (mean + spread).
    float sum_cos = 0.0f, sum_sin = 0.0f;
    float max_abs_err = 0.0f;

    for (int i = 0; i < NUM_PROBES; i++) {
        float cmd = normalise_angle(i * ELEC_STEP);

        prev_cmd = slewField(driver, prev_cmd, cmd, Vd);
        delay(SETTLE_MS);

        float mech = averageMech(sensor);
        float est = estimateElectricalAngle(mech, motor.pole_pairs, motor.sensor_direction,
                                            motor.electrical_offset);
        float err = findAngularError(est, cmd);            // [-pi, pi]
        float implied = normalise_angle(mech * motor.pole_pairs * motor.sensor_direction - cmd);

        sum_cos += cosf(implied);
        sum_sin += sinf(implied);
        if (fabsf(err) > max_abs_err) max_abs_err = fabsf(err);

        if (i == 0) first_mech = mech;
        last_mech = mech;

        Serial.println(String("DBG ") + String(cmd, 4) + " " + String(mech, 4) + " " +
                       String(est, 4) + " " + String(err, 4) + " " + String(implied, 4));
    }

    // Circular mean of the implied offsets (correct way to average angles).
    float mean_implied = normalise_angle(atan2f(sum_sin, sum_cos));
    float R = sqrtf(sum_cos * sum_cos + sum_sin * sum_sin) / (float)NUM_PROBES;  // 1=tight, 0=spread
    float circ_std = sqrtf(-2.0f * logf(R > 1e-6f ? R : 1e-6f));                 // circular stddev [rad]

    // Net mechanical travel over the +2pi electrical command (sign => direction).
    float mech_travel = findAngularError(last_mech, first_mech);
    // Expected magnitude if the rotor tracks: ~ (2pi - one step) / pole_pairs.
    float expected_travel_mag = ((NUM_PROBES - 1) * ELEC_STEP) / motor.pole_pairs;

    Serial.println();
    Serial.println(String("---- ") + label + " summary ----");
    Serial.println(String("circular-mean implied offset = ") + String(mean_implied, 4) +
                   " rad   (stored = " + String(motor.electrical_offset, 4) + ")");
    Serial.println(String("offset error (mean implied - stored) = ") +
                   String(findAngularError(mean_implied, motor.electrical_offset), 4) + " rad");
    Serial.println(String("implied offset circular stddev = ") + String(circ_std, 4) +
                   " rad   (R = " + String(R, 4) + ")");
    Serial.println(String("max |est - cmd| error = ") + String(max_abs_err, 4) + " rad");
    Serial.println(String("net mech travel = ") + String(mech_travel, 4) +
                   " rad  (expected magnitude ~ " + String(expected_travel_mag, 4) +
                   ", sign should be +)");

    driver.setPwm(0.0f, 0.0f, 0.0f);
    driver.disable();
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

}  // namespace electrical_offset_accuracy

inline void electrical_offset_accuracy_test() {
    using namespace electrical_offset_accuracy;

    beginSerial();
    delay(1000);
#if defined(STM32G4xx)
    Serial.println("Platform: STM32G4");
#else
    Serial.println("Platform: Teensy 4.1");
#endif
    Serial.println("=== Electrical Offset ACCURACY Test ===");
    Serial.println("Rotor will rotate ~1/7 turn per motor under power. Keep shafts free.");

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

    runMotor("MOTOR X", sensorX, driverX, motorX);
    runMotor("MOTOR Y", sensorY, driverY, motorY);

    Serial.println();
    Serial.println("================ ACCURACY TEST COMPLETE ================");
    Serial.println("Both motors de-energised.");

    while (true) {
        delay(1000);
    }
}
