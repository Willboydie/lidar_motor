// Entry point for [env:teensy41] — dual-axis FOC serving UI/realtime_plotter.py.
//
// Host protocol (docs/serial_protocol.md / UI/realtime_plotter.py):
//   RX: 5-byte commands <Bf>  (TARGET_X/Y, enable, PID, …)
//   TX: ControlPacket 0x55 @ 500 Hz (19 B) + StatusPacket 0xAA @ 10 Hz (52 B)
//   Debug::msg framed as 0xFF 0x04 0x06 + text line
//
// Path setpoints are host-generated (TARGET_X_SET / TARGET_Y_SET). Onboard
// ScanPath is constructed for Interface completeness but is not driven here.
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
#include "ldm/LDM.hpp"
#include "scan_path/ScanPath.hpp"
#include "angle_tracker/AngleTracker.hpp"
#include "fault_manager/FaultManager.hpp"
#include "system_manager/SystemManager.hpp"
#include "debug/Debug.hpp"

// No Garmin on the FOC / plotter bench — Interface still needs an LDM reference.
class BenchStubLDM : public LDM {
public:
    BenchStubLDM() : LDM(GARMIN_SCL, GARMIN_SDA, GARMIN_MODE_PIN) {}

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

BenchStubLDM ldm;
StateMachine stateMachine;
AS5048 sensorX(SENSOR_SPI_CS_X);
AS5048 sensorY(SENSOR_SPI_CS_Y);
PWMDriverTeensy41 pwmDriverX(N1_X, N2_X, N3_X, EN_X);
PWMDriverTeensy41 pwmDriverY(N1_Y, N2_Y, N3_Y, EN_Y);
GM2804 motorX(1, SENSOR_OFFSET_X);
GM2804 motorY(2, SENSOR_OFFSET_Y);
FOCController focControllerX(sensorX, pwmDriverX, motorX);
FOCController focControllerY(sensorY, pwmDriverY, motorY);
InlineCurrentSenseTeensy41 currentSenseX;
InlineCurrentSenseTeensy41 currentSenseY;
SinusoidalScanPath scanPath(THETA_INC, PHI_INC, THETA_AMPLITUDE, PHI_AMPLITUDE, THETA_CENTRE, PHI_CENTRE);
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
    scanPath, faultManager, systemManager);

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial && millis() < 3000) {
        delay(10);
    }
    Serial.println("realtime_plotter stub — Teensy 4.1 dual FOC");
    Serial.print("BAUD=");
    Serial.print(BAUD_RATE);
    Serial.print(" CONTROL_us=");
    Serial.print(CONTROL_PACKET_PERIOD_US);
    Serial.print(" STATUS_ms=");
    Serial.println(STATUS_PACKET_PERIOD_MS);

    PWMDriverTeensy41::clearRegisteredSubmodules();

    systemManager.init();
    if (!systemManager.isInitialised()) {
        Debug::msg("SYSTEM INIT FAILED");
        return;
    }

    PWMDriverTeensy41::alignAllRegisteredSubmodules();
    Debug::msg("PWM submodule align done");

    focControllerX.setControlMode(ANGLE);
    focControllerY.setControlMode(ANGLE);
    stateMachine.controlMode = ANGLE;
    stateMachine.scanning = false;

    focControllerX.calibrate();
    focControllerY.calibrate();

    Debug::msg("SYSTEM INITIALISED — connect UI/realtime_plotter.py then Enable");
}

void loop() {
    static uint32_t control_us = micros();
    static uint32_t status_ms = millis();

    interface.commandListener();

    if ((uint32_t)(millis() - status_ms) >= STATUS_PACKET_PERIOD_MS) {
        status_ms = millis();
        interface.sendStatusPacket();
    }

    // Keep sensors fresh so ControlPacket mechanical angles are valid even when disabled.
    if (!stateMachine.enabled) {
        sensorX.update();
        sensorY.update();
    } else {
        focControllerX.loop();
        focControllerY.loop();
    }

    if ((uint32_t)(micros() - control_us) >= CONTROL_PACKET_PERIOD_US) {
        control_us = micros();
        interface.sendControlPacket();
    }
}

#endif
