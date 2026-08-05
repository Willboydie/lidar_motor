// Entry point for [env:teensy41] — ScanEngine FOC-target bench for
// pytools/realtime_plotter.py. See src/tests/scan/scan_engine_test.hpp.
#if defined(ARDUINO_TEENSY41)

#include <Arduino.h>
#include "interface/Interface.hpp"
#include "state_machine/StateMachine.hpp"
#include "motor/GM2804.hpp"
#include "foc_controller/FOCController.hpp"
#include "sensor/AS5048.hpp"
#include "pwm_driver/teensy41/PWMDriverTeensy41.hpp"
#include "current_sense/teensy41/InlineCurrentSenseTeensy41.hpp"
#include "utils/tweakable_params.h"
#include "utils/constants.h"
#include "ldm/GarminV3HP.hpp"
#include "angle_tracker/AngleTracker.hpp"
#include "fault_manager/FaultManager.hpp"
#include "system_manager/SystemManager.hpp"
#include "scan_engine/ScanEngine.hpp"
#include "scan_engine/coarse/MovingLissa.hpp"
#include "scan_engine/s_curve/SCurve.hpp"
#include "debug/Debug.hpp"

GarminV3HP ldm(GARMIN_SDA, GARMIN_SCL, GARMIN_MODE_PIN);
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
    faultManager, stateMachine);
Interface interface(
    stateMachine,
    focControllerX, focControllerY,
    sensorX, sensorY,
    currentSenseX, currentSenseY,
    ldm, angleTrackerX, angleTrackerY,
    faultManager, systemManager,
    movingLissa, scanEngine);

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial && millis() < 3000) {
        delay(10);
    }
    delay(200);
    systemManager.init();

    focControllerX.setControlMode(ANGLE);
    focControllerY.setControlMode(ANGLE);
    stateMachine.controlMode = ANGLE;

    focControllerX.calibrate();
    focControllerY.calibrate();
}

void loop() {
    static uint32_t control_us = micros();
    static uint32_t status_ms = millis();
    static uint32_t pos_us = micros();
    static uint32_t vel_us = micros();
    static uint32_t cur_us = micros();

    interface.commandListener();

    if (ldm.isDataReady()) {
        angleTrackerX.calculateMedianAverage();
        angleTrackerY.calculateMedianAverage();
        ldm.triggerMeasurement();
        ldm.readDistance();
    }

    // Position path (~angle-loop rate): ScanEngine writes FOC targets
    if ((uint32_t)(micros() - pos_us) >= ANGLE_LOOP_DT_US) {
        const uint32_t now = micros();
        const float dt = (now - pos_us) * 1e-6f;
        pos_us = now;
        scanEngine.updatePosition(dt);
        focControllerX.angle_loop();
        focControllerY.angle_loop();
        angleTrackerX.update();
        angleTrackerY.update();
    }

    // Velocity path (faster)
    if ((uint32_t)(micros() - vel_us) >= VELOCITY_LOOP_DT_US) {
        const uint32_t now = micros();
        vel_us = now;
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

#endif