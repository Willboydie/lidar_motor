#pragma once

#include "../../sensor/AS5048.hpp"
#include "../../pwm_driver/PWMDriver.hpp"
#include "../../motor/GM2804.hpp"
#include "../../utils/constants.h"
#include "../../utils/tweakable_params.h"
#include "../../utils/quick_defs.h"
#include <Arduino.h>

#if defined(STM32G4xx)
#include "../../pwm_driver/stm32g4/PWMDriverSTM32G4.hpp"
#else
#include "../../pwm_driver/teensy41/PWMDriverTeensy41.hpp"
#endif

// ============================================================================
// Electrical offset measurement (bench procedure)
// Teensy 4.1: enable via main.cpp (#include + electrical_offset_test()).
// STM32G4:     built from _stm32g4_main_stub.cpp ([env:stm32g4]).
// ----------------------------------------------------------------------------
// Inspired by finding_offset.hpp, but instead of letting the rotor snap to a
// single electrical zero, the bench engineer walks the rotor through every
// pole pair by hand and brackets the true zero at each cog.
//
//   1. The motor is "charged": a pure d-axis voltage is injected at a fixed
//      electrical angle (0). This creates a holding torque with one stable
//      detent per pole pair (7 for the GM2804) around a mechanical revolution.
//   2. The engineer hand-rotates to each detent (one per pole pair).
//   3. At each detent they nudge the rotor CLOCKWISE, let it settle, capture;
//      then nudge ANTICLOCKWISE, let it settle, capture. The two readings sit
//      symmetrically either side of the true electrical zero (stiction bias).
//   4. The live angle and the current stage (pole pair + displacement) are
//      printed continuously so the engineer always knows where they are.
//   5. Pressing SPACE grabs the next CAPTURE_SAMPLES readings, averages them,
//      and stores the implied electrical offset for that stage.
//
// 7 pole pairs x 2 displacements = 14 values per motor. The mean of the 14 is
// that motor's electrical offset (equivalently the mean of the 7 per-pole-pair
// midpoints). Drop the printed offsets into py_utils/offset_calc.py to verify.
// ============================================================================

namespace electrical_offset {

constexpr int NUM_POLE_PAIRS = GM2804_POLE_PAIRS;  // detents per revolution
constexpr int NUM_DISPLACEMENTS = 2;               // clockwise + anticlockwise
constexpr int NUM_STAGES = NUM_POLE_PAIRS * NUM_DISPLACEMENTS;  // 14
constexpr int CAPTURE_SAMPLES = 10;                // readings averaged per stage
constexpr unsigned long LIVE_PRINT_MS = 150;       // live readout cadence
constexpr unsigned long CAPTURE_SAMPLE_GAP_MS = 5; // spacing between captures
constexpr int CLOCKWISE = 0;
constexpr int ANTICLOCKWISE = 1;

inline const char* displacementName(int d) {
    return d == CLOCKWISE ? "CLOCKWISE" : "ANTICLOCKWISE";
}

// Hold the rotor at electrical angle 0 with a pure d-axis voltage.
// Mirrors FOCController::setPhaseVoltage (inverse Park + Clarke + midpoint
// clamp) so this bench test stays self-contained.
inline void applyHoldingVoltage(PWMDriver& driver, float Vd) {
    float s, c;
    fast_sincos(0.0f, &s, &c);  // electrical angle held at 0 -> s=0, c=1

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

// At a cog the applied electrical angle is 0, so the implied offset is the
// mechanical angle mapped into electrical space: wrap(theta * pole_pairs * dir).
inline float offsetFromAngle(float mech_angle, float pole_pairs, float sensor_direction) {
    return normalise_angle(mech_angle * pole_pairs * sensor_direction);
}

// Returns true (and drains the buffer) when a space character is seen.
inline bool spacePressed() {
    bool pressed = false;
    while (Serial.available()) {
        if (Serial.read() == ' ') {
            pressed = true;
        }
    }
    return pressed;
}

// Live-print the angle/stage until SPACE, then average the next
// CAPTURE_SAMPLES readings and return the implied electrical offset.
inline float captureStage(const char* motor_label, int pole_pair, int displacement,
                          AS5048& sensor, Motor& motor) {
    Serial.println();
    Serial.println(String("=== ") + motor_label + " | pole pair " + String(pole_pair) +
                   "/" + String(NUM_POLE_PAIRS) + " | " + displacementName(displacement) +
                   " displacement ===");
    Serial.println(String("Rotate to the next detent, nudge ") + displacementName(displacement) +
                   " against the holding torque, let it settle, then press SPACE.");

    unsigned long last_print = 0;
    while (!spacePressed()) {
        sensor.update();
        if (millis() - last_print >= LIVE_PRINT_MS) {
            last_print = millis();
            float angle = sensor.getLatestRotorAngle();
            float offset = offsetFromAngle(angle, motor.pole_pairs, motor.sensor_direction);
            Serial.println(String(motor_label) + " | pp " + String(pole_pair) + "/" +
                           String(NUM_POLE_PAIRS) + " | " + displacementName(displacement) +
                           " | angle: " + String(angle, 4) + " rad | live offset: " +
                           String(offset, 4) + " rad");
        }
    }

    // Grab the next CAPTURE_SAMPLES readings and average using circular statistics
    // to protect against detents that sit exactly on the 0/2pi wrap.
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;
    for (int i = 0; i < CAPTURE_SAMPLES; i++) {
        sensor.update();
        float angle = sensor.getLatestRotorAngle();
        sum_sin += sinf(angle);
        sum_cos += cosf(angle);
        delay(CAPTURE_SAMPLE_GAP_MS);
    }
    float avg_angle = normalise_angle(atan2f(sum_sin, sum_cos));
    float offset = offsetFromAngle(avg_angle, motor.pole_pairs, motor.sensor_direction);
    Serial.println(String(">>> Captured. avg angle: ") + String(avg_angle, 4) +
                   " rad -> electrical offset: " + String(offset, 4) + " rad");
    return offset;
}

inline void waitForSpace(const String& prompt) {
    Serial.println(prompt);
    while (!spacePressed()) {
        delay(10);
    }
}

// Run the full 14-stage procedure for one motor and return its mean offset.
inline float measureMotor(const char* motor_label, AS5048& sensor,
                          PWMDriver& driver, Motor& motor) {
    Serial.println();
    Serial.println(String("################  ") + motor_label + "  ################");

    float Vd = MOTOR_CURRENT_LIMIT * 0.75f * motor.phase_resistance;
    Vd = _constrain(Vd, 0.0f, MOTOR_VOLTAGE_LIMIT);

    driver.enable();
    Serial.println(String("Charging ") + motor_label + " - holding voltage Vd = " +
                   String(Vd, 2) + " V");
    applyHoldingVoltage(driver, Vd);
    delay(1500);  // let the rotor snap into the first detent

    float offsets[NUM_POLE_PAIRS][NUM_DISPLACEMENTS];
    for (int pp = 0; pp < NUM_POLE_PAIRS; pp++) {
        for (int d = 0; d < NUM_DISPLACEMENTS; d++) {
            applyHoldingVoltage(driver, Vd);  // keep the holding torque alive
            offsets[pp][d] = captureStage(motor_label, pp + 1, d, sensor, motor);
        }
    }

    driver.setPwm(0.0f, 0.0f, 0.0f);  // de-energise this motor, measurement done

    // Summarise. Circular mean of the 14 values == circular mean of the 7 midpoints.
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;
    Serial.println();
    Serial.println(String("---- ") + motor_label + " results ----");
    Serial.println("[  # [clockwise, anticlockwise] per pole pair");
    for (int pp = 0; pp < NUM_POLE_PAIRS; pp++) {
        float cw = offsets[pp][CLOCKWISE];
        float ccw = offsets[pp][ANTICLOCKWISE];

        // Circular average of the two bounds for the local midpoint
        float mid = normalise_angle(atan2f(sinf(cw) + sinf(ccw), cosf(cw) + cosf(ccw)));

        // Accumulate vectors for the global motor offset
        sum_sin += sinf(cw) + sinf(ccw);
        sum_cos += cosf(cw) + cosf(ccw);
        Serial.println(String("  [") + String(cw, 4) + ", " + String(ccw, 4) + "]" +
                       (pp < NUM_POLE_PAIRS - 1 ? "," : " ") + "  # pp " + String(pp + 1) +
                       " midpoint: " + String(mid, 4));
    }
    Serial.println("]");

    // Extract the final offset from the accumulated 14 displacement vectors
    float mean_offset = normalise_angle(atan2f(sum_sin, sum_cos));
    Serial.println(String(motor_label) + " electrical offset (circular mean of " + String(NUM_STAGES) +
                   " values): " + String(mean_offset, 4) + " rad");
    return mean_offset;
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

}  // namespace electrical_offset

inline void electrical_offset_test() {
    using namespace electrical_offset;

    beginSerial();
    delay(1000);
#if defined(STM32G4xx)
    Serial.println("Platform: STM32G4");
#else
    Serial.println("Platform: Teensy 4.1");
#endif
    Serial.println("=== Electrical Offset Measurement ===");
    Serial.println("Charge a motor, hand-rotate to each detent (one per pole pair),");
    Serial.println("nudge CLOCKWISE then ANTICLOCKWISE, press SPACE at each settled point.");
    Serial.println(String("Per motor: ") + String(NUM_POLE_PAIRS) + " pole pairs x " +
                   String(NUM_DISPLACEMENTS) + " displacements = " + String(NUM_STAGES) +
                   " captures.");

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

    // Ensure both motors start de-energised (zero field on every phase).
    driverX.enable();
    driverX.setPwm(0.0f, 0.0f, 0.0f);
    driverY.enable();
    driverY.setPwm(0.0f, 0.0f, 0.0f);

    waitForSpace("\nPress SPACE to begin measuring MOTOR X.");
    float offsetX = measureMotor("MOTOR X", sensorX, driverX, motorX);

    waitForSpace("\nMOTOR X done. Press SPACE to begin measuring MOTOR Y.");
    float offsetY = measureMotor("MOTOR Y", sensorY, driverY, motorY);

    driverX.disable();
    driverY.disable();

    Serial.println();
    Serial.println("================ SUMMARY ================");
    Serial.println(String("MOTOR X electrical offset: ") + String(offsetX, 4) +
                   " rad  (-> constants.h SENSOR_OFFSET_1)");
    Serial.println(String("MOTOR Y electrical offset: ") + String(offsetY, 4) +
                   " rad  (-> constants.h SENSOR_OFFSET_2)");
    Serial.println("Measurement complete. Both motors de-energised.");

    while (true) {
        delay(1000);
    }
}
