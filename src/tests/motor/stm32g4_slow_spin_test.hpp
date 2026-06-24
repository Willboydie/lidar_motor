#pragma once

#include "Arduino.h"
#include "interface/Interface.hpp"
#include "state_machine/StateMachine.hpp"
#include "motor/Motor.hpp"
#include "motor/GM2804.hpp"
#include "foc_controller/FOCController.hpp"
#include "sensor/AS5048.hpp"
#include "pwm_driver/stm32g4/PWMDriverSTM32G4.hpp"
#include "current_sense/teensy41/InlineCurrentSenseTeensy41.hpp"
#include "ldm/GarminV3HP.hpp"
#include "scan_path/ScanPath.hpp"
#include "utils/tweakable_params.h"
#include "utils/constants.h"

// ============ Configure here ============
#define SPIN_MOTOR_X  1          // 1 = spin motor X, 0 = spin motor Y
constexpr float SPIN_MECH_RATE_RAD_S = 0.5f;  // mechanical rad/s (slow continuous spin)
constexpr unsigned long FOC_PRINT_PERIOD_MS = 200;
// ========================================

inline void stm32g4_slow_spin_test() {
    Serial.begin(BAUD_RATE);
    delay(2000);

    GarminV3HP ldm = GarminV3HP(GARMIN_SCL, GARMIN_SDA, GARMIN_MODE_PIN);
    StateMachine stateMachine;
    AS5048 sensorX = AS5048(SENSOR_SPI_CS_X);
    AS5048 sensorY = AS5048(SENSOR_SPI_CS_Y);
    PWMDriverSTM32G4 pwmDriverX = PWMDriverSTM32G4(N1_X, N2_X, N3_X, EN_X);
    PWMDriverSTM32G4 pwmDriverY = PWMDriverSTM32G4(N1_Y, N2_Y, N3_Y, EN_Y);
    GM2804 motorX(1, SENSOR_OFFSET_X);
    GM2804 motorY(2, SENSOR_OFFSET_Y);

    FOCController focControllerX(sensorX, pwmDriverX, motorX);
    FOCController focControllerY(sensorY, pwmDriverY, motorY);
    InlineCurrentSenseTeensy41 currentSenseX;
    InlineCurrentSenseTeensy41 currentSenseY;
    SinusoidalScanPath scanPath(THETA_INC, PHI_INC, THETA_AMPLITUDE, PHI_AMPLITUDE, THETA_CENTRE, PHI_CENTRE);

    Interface interface(stateMachine, focControllerX, focControllerY, sensorX, sensorY, currentSenseX, currentSenseY, ldm, scanPath);

    focControllerX.init();
    focControllerY.init();

#if SPIN_MOTOR_X
    FOCController& spinFoc = focControllerX;
    Sensor& spinSensor = sensorX;
    const char* spinLabel = "X";
#else
    FOCController& spinFoc = focControllerY;
    Sensor& spinSensor = sensorY;
    const char* spinLabel = "Y";
#endif

    spinFoc.setControlMode(VELOCITY);
    float targetVelocity = SPIN_MECH_RATE_RAD_S;
    spinFoc.setTargetVelocity(targetVelocity);
    spinFoc.enable();
    stateMachine.enabled = true;

    Serial.println("Slow spin test: motor " + String(spinLabel)
        + " @ " + String(SPIN_MECH_RATE_RAD_S, 3) + " rad/s (mechanical)");

    uint32_t lastSpinUs = micros();
    unsigned long lastStatusMs = millis();
    unsigned long lastPrintMs = millis();

    while (true) {
        const uint32_t nowUs = micros();
        const float dt = (nowUs - lastSpinUs) * 1e-6f;
        lastSpinUs = nowUs;

        spinFoc.loop();

        if (millis() - lastStatusMs >= FOC_STATUS_PERIOD_MS) {
            lastStatusMs = millis();
            interface.sendDualMotorStatus();
        }

        if (millis() - lastPrintMs >= FOC_PRINT_PERIOD_MS) {
            lastPrintMs = millis();
            Serial.print("M");
            Serial.print(spinLabel);
            Serial.print(" | ");
            interface.printFOCStatus(spinFoc, spinSensor);
        }
    }
}
