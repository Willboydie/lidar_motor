#pragma once

// ScanEngine bench for pytools/realtime_plotter.py.
//
// Purpose: drive FOCController target angles via ScanEngine and stream them in
// ControlPacket (0x55). Motors / FOC loops are intentionally not run — only
// targets matter. Interface.commandListener() enacts host commands (enable
// ScanEngine, coarse / P2P, centre, amplitudes, etc.).
//
// Host protocol (same as production / realtime_plotter.py):
//   RX: [cmd:1][len_lo:1][len_hi:1][payload]  (Parser / cmd_defs.h)
//   TX: ControlPacket 0x55 @ CONTROL_PACKET_PERIOD_US
//       StatusPacket  0xAA @ STATUS_PACKET_PERIOD_MS
//
// Flash:
//   pio run -e teensy41 --target upload
//   # entry: src/_teensy_main_stub.cpp → scan_engine_test()

#include <Arduino.h>

#include "../../interface/Interface.hpp"
#include "../../state_machine/StateMachine.hpp"
#include "../../motor/GM2804.hpp"
#include "../../foc_controller/FOCController.hpp"
#include "../../sensor/AS5048.hpp"
#include "../../pwm_driver/teensy41/PWMDriverTeensy41.hpp"
#include "../../current_sense/teensy41/InlineCurrentSenseTeensy41.hpp"
#include "../../utils/tweakable_params.h"
#include "../../utils/constants.h"
#include "../../ldm/LDM.hpp"
#include "../../angle_tracker/AngleTracker.hpp"
#include "../../fault_manager/FaultManager.hpp"
#include "../../system_manager/SystemManager.hpp"
#include "../../scan_engine/ScanEngine.hpp"
#include "../../scan_engine/coarse/MovingLissa.hpp"
#include "../../scan_engine/s_curve/SCurve.hpp"
#include "../../debug/Debug.hpp"

// No Garmin on the ScanEngine / plotter bench — Interface still needs an LDM.
class ScanEngineBenchLDM : public LDM {
public:
    ScanEngineBenchLDM() : LDM(GARMIN_SCL, GARMIN_SDA, GARMIN_MODE_PIN) {}

    void init() override {
        markInitialised();
        Debug::msg("LDM bench stub initialised (no hardware)");
    }

    void reconfigure(int config_num) override {
        (void)config_num;
        Debug::msg("LDM bench stub reconfigure ignored");
    }

private:
    void requestDistance() override {}
    void readLatestDistance() override { latest_distance_cm = 0; }
};

inline void scan_engine_test() {
    Serial.begin(BAUD_RATE);
    while (!Serial && millis() < 3000) {
        delay(10);
    }
    delay(200);
    Serial.println("SCAN ENGINE TEST — FOC target stream for realtime_plotter.py");
    Serial.print("BAUD=");
    Serial.print(BAUD_RATE);
    Serial.print(" CONTROL_us=");
    Serial.print(CONTROL_PACKET_PERIOD_US);
    Serial.print(" STATUS_ms=");
    Serial.println(STATUS_PACKET_PERIOD_MS);

    PWMDriverTeensy41::clearRegisteredSubmodules();

    ScanEngineBenchLDM ldm;
    StateMachine stateMachine;
    AS5048 sensorX(SENSOR_SPI_CS_X);
    AS5048 sensorY(SENSOR_SPI_CS_Y);
    PWMDriverTeensy41 pwmDriverX(N1_X, N2_X, N3_X, EN_X);
    PWMDriverTeensy41 pwmDriverY(N1_Y, N2_Y, N3_Y, EN_Y);
    GM2804 motorX(1, SENSOR_OFFSET_X);
    GM2804 motorY(2, SENSOR_OFFSET_Y);
    FOCController focControllerX(sensorX, pwmDriverX, motorX);
    FOCController focControllerY(sensorY, pwmDriverY, motorY);
    SCurve sCurveX;
    SCurve sCurveY;
    MovingLissa movingLissa(
        DEFAULT_X_SCAN_FREQUENCY, DEFAULT_Y_SCAN_FREQUENCY,
        DEFAULT_X_SCAN_AMPLITUDE, DEFAULT_Y_SCAN_AMPLITUDE,
        DEFAULT_X_SCAN_PHASE, DEFAULT_Y_SCAN_PHASE);
    ScanEngine scanEngine(focControllerX, focControllerY, movingLissa, sCurveX, sCurveY);
    InlineCurrentSenseTeensy41 currentSenseX;
    InlineCurrentSenseTeensy41 currentSenseY;
    AngleTracker angleTrackerX(sensorX, faultManager, FAULT_TRACKER_X_OVERFLOW);
    AngleTracker angleTrackerY(sensorY, faultManager, FAULT_TRACKER_Y_OVERFLOW);
    SystemManager systemManager(
        focControllerX, focControllerY,
        sensorX, sensorY,
        currentSenseX, currentSenseY,
        ldm, pwmDriverX, pwmDriverY,
        faultManager, stateMachine, scanEngine);
    Interface interface(
        stateMachine,
        focControllerX, focControllerY,
        sensorX, sensorY,
        currentSenseX, currentSenseY,
        ldm, angleTrackerX, angleTrackerY,
        faultManager, systemManager,
        movingLissa, scanEngine);

    // Best-effort hardware init so Interface enable/status still work when the
    // bench is wired. ScanEngine target updates do not depend on it.
    systemManager.init();
    if (systemManager.isInitialised()) {
        PWMDriverTeensy41::alignAllRegisteredSubmodules();
        focControllerX.setControlMode(ANGLE);
        focControllerY.setControlMode(ANGLE);
        stateMachine.controlMode = ANGLE;
        Debug::msg("SYSTEM INITIALISED");
    } else {
        Debug::msg("SYSTEM INIT SKIPPED/FAILED — ScanEngine targets still stream");
        focControllerX.setTargetAngle(DEFAULT_NEUTRAL_X_POS);
        focControllerY.setTargetAngle(DEFAULT_NEUTRAL_Y_POS);
        stateMachine.controlMode = ANGLE;
    }

    Debug::msg("Connect realtime_plotter.py — Enable motors, Scan Engine On, then Begin Coarse");

    uint32_t control_us = micros();
    uint32_t status_ms = millis();
    uint32_t pos_us = micros();
    uint32_t vel_us = micros();
    uint32_t cur_us = micros();
    uint32_t last_us = micros();

    while (true) {
        interface.commandListener();

        // Position path (~angle-loop rate): ScanEngine writes FOC targets.s
        if ((uint32_t)(micros() - pos_us) >= ANGLE_LOOP_DT_US) {
            const uint32_t now = micros();
            const float dt = (now - last_us) * 1e-6f;
            pos_us = now;
            last_us = now;
            scanEngine.updatePosition(dt);
            focControllerX.angle_loop();
            focControllerY.angle_loop();
        }

        // Velocity path (faster): feed-forward / SCurve velocity only.
        if ((uint32_t)(micros() - vel_us) >= VELOCITY_LOOP_DT_US) {
            const uint32_t now = micros();
            const float dt = (now - last_us) * 1e-6f;
            vel_us = now;
            last_us = now;
            // scanEngine.updateVelocity(dt);
            focControllerX.velocity_loop();
            focControllerY.velocity_loop();
        }

        if ((uint32_t)(micros() - cur_us) >= CURRENT_LOOP_DT_US) {
            const uint32_t now = micros();
            cur_us = now;
            focControllerX.current_loop();
            focControllerY.current_loop();
        }

        if ((uint32_t)(millis() - status_ms) >= STATUS_PACKET_PERIOD_MS) {
            status_ms = millis();
            interface.sendStatusPacket();
        }

        if ((uint32_t)(micros() - control_us) >= CONTROL_PACKET_PERIOD_US) {
            control_us = micros();
            interface.sendControlPacket();
        }
    }
}