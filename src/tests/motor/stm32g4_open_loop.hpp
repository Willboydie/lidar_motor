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
#include "utils/quick_defs.h"

inline void stm32g4_open_loop() {

    Serial.begin(460800);
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

    // No current-sense hardware needed for open loop — zero stub feeds the
    // ABC/DQ channels so sendDualMotorStatus() can be exercised end-to-end.
    InlineCurrentSenseTeensy41 currentSenseX;
    InlineCurrentSenseTeensy41 currentSenseY;
    SinusoidalScanPath scanPath(THETA_INC, PHI_INC, THETA_AMPLITUDE, PHI_AMPLITUDE, THETA_CENTRE, PHI_CENTRE);

    Interface interface(stateMachine, focControllerX, focControllerY, sensorX, sensorY, currentSenseX, currentSenseY, ldm, scanPath);

    focControllerX.init();
    focControllerY.init();

    float elec_angle_x = 0.0f;
    float elec_angle_y = 0.0f;
    float Vd_x = 0.0f;
    float Vd_y = 0.0f;
    float Vq_x = 5.0f;
    float Vq_y = 5.0f;
    
    focControllerX.setElectricalAngle(elec_angle_x);
    focControllerY.setElectricalAngle(elec_angle_y);
    focControllerX.setVd(Vd_x);
    focControllerY.setVd(Vd_y);
    focControllerX.setVq(Vq_x);
    focControllerY.setVq(Vq_y);

    float elec_inc = 0.0001f;

    // #region agent log
    float prev_dbg_mech_x = 0.0f;
    float prev_dbg_elec_x = elec_angle_x;
    long lastDbgMs = millis();
    // #endregion

    while (true) {
        elec_angle_x += elec_inc;
        elec_angle_y += elec_inc;

        elec_angle_x = normalise_angle(elec_angle_x);
        elec_angle_y = normalise_angle(elec_angle_y);

        focControllerX.setElectricalAngle(elec_angle_x);
        focControllerY.setElectricalAngle(elec_angle_y);

        focControllerX.setPhaseVoltage();
        focControllerY.setPhaseVoltage();

        sensorX.update();
        sensorY.update();

        delayMicroseconds(50);

        static long lastTime = millis();
        if (millis() - lastTime > 10) {
            lastTime = millis();
            interface.sendDualMotorStatus();
            // Serial.println("Angle X: " + String(elec_angle_x) + "  |  Angle Y: " + String(elec_angle_y));
        }

        // #region agent log
        if (millis() - lastDbgMs > 100) {
            lastDbgMs = millis();
            float mech_x = sensorX.getLatestRotorAngle();
            float elec_est_x = normalise_angle(
                mech_x * GM2804_POLE_PAIRS * GM2804_SENSOR_DIRECTION - SENSOR_OFFSET_X);
            float align_err_x = findAngularError(elec_angle_x, elec_est_x);
            float d_mech_dbg = findAngularError(mech_x, prev_dbg_mech_x);
            float d_elec_dbg = findAngularError(elec_angle_x, prev_dbg_elec_x);
            prev_dbg_mech_x = mech_x;
            prev_dbg_elec_x = elec_angle_x;
            Serial.print("DBG ");
            Serial.print(elec_angle_x, 5);
            Serial.print(",");
            Serial.print(mech_x, 5);
            Serial.print(",");
            Serial.print(elec_est_x, 5);
            Serial.print(",");
            Serial.print(align_err_x, 5);
            Serial.print(",");
            Serial.print(d_mech_dbg, 5);
            Serial.print(",");
            Serial.println(d_elec_dbg, 5);
        }
        // #endregion

    }
}
