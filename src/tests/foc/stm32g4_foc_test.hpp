#include "Arduino.h"
#include "interface/Interface.hpp"
#include "state_machine/StateMachine.hpp"
#include "motor/Motor.hpp"
#include "motor/GM2804.hpp"
#include "foc_controller/FOCController.hpp"
#include "sensor/AS5048.hpp"
#include "pwm_driver/stm32g4/PWMDriverSTM32G4.hpp"
#include "current_sense/teensy41/InlineCurrentSenseTeensy41.hpp"
#include "utils/tweakable_params.h"
#include "utils/constants.h"
#include "ldm/GarminV3HP.hpp"
#include "scan_path/ScanPath.hpp"

inline void stm32g4_foc_test() {

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

    Serial.begin(BAUD_RATE);
    delay(2000);
    focControllerX.init();
    focControllerY.init();
    Serial.println("System initialised.");

    while (true) {
        static unsigned long cmdTime = millis();
        if (millis() - cmdTime > 200) {
            cmdTime = millis();
            interface.commandListener();
        }

        static unsigned long statusTime = millis();
        if (millis() - statusTime >= FOC_STATUS_PERIOD_MS) {
            statusTime = millis();
            interface.printFOCStatus(focControllerX, sensorX);
        }

        if ( stateMachine.enabled ) {
            focControllerX.loop();
        }
    }
}