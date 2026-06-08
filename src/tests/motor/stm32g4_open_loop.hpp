#include "Arduino.h"
#include "interface/Interface.hpp"
#include "state_machine/StateMachine.hpp"
#include "motor/Motor.hpp"
#include "motor/GM2804.hpp"
#include "foc_controller/FOCController.hpp"
#include "sensor/AS5048.hpp"
#include "pwm_driver/stm32g4/PWMDriverSTM32G4.hpp"
#include "utils/tweakable_params.h"
#include "utils/constants.h"
#include "utils/quick_defs.h"

inline void stm32g4_open_loop() {
    Serial.begin(115200);
    
    StateMachine stateMachine;
    AS5048 sensorX = AS5048(SENSOR_SPI_CS_X);
    AS5048 sensorY = AS5048(SENSOR_SPI_CS_Y);
    PWMDriverSTM32G4 pwmDriverX = PWMDriverSTM32G4(N1_X, N2_X, N3_X, EN_X);
    PWMDriverSTM32G4 pwmDriverY = PWMDriverSTM32G4(N1_Y, N2_Y, N3_Y, EN_Y);
    GM2804 motorX(1, SENSOR_OFFSET_X);
    GM2804 motorY(2, SENSOR_OFFSET_Y);

    FOCController focControllerX(sensorX, pwmDriverX, motorX);
    FOCController focControllerY(sensorY, pwmDriverY, motorY);

    focControllerX.init();
    focControllerY.init();

    float elec_angle_x = 0.0f;
    float elec_angle_y = 0.0f;
    float Vd_x = 0.0f;
    float Vd_y = 0.0f;
    float Vq_x = 2.5f;
    float Vq_y = 5.0f;
    
    focControllerX.setElectricalAngle(elec_angle_x);
    focControllerY.setElectricalAngle(elec_angle_y);
    focControllerX.setVd(Vd_x);
    focControllerY.setVd(Vd_y);
    focControllerX.setVq(Vq_x);
    focControllerY.setVq(Vq_y);

    float elec_inc = 0.001f;

    while (true) {
        elec_angle_x += elec_inc;
        elec_angle_y += elec_inc;

        focControllerX.setElectricalAngle(elec_angle_x);
        focControllerY.setElectricalAngle(elec_angle_y);

        focControllerX.setPhaseVoltage();
        focControllerY.setPhaseVoltage();

        delayMicroseconds(50);
        elec_inc += 0.00001f;

        static long lastTime = millis();
        if (millis() - lastTime > 500) {
            lastTime = millis();
            Serial.println("Electrical Angle: " + String(normalise_angle(elec_angle_x)));
        }

    }
}