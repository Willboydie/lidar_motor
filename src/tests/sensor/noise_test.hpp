#pragma once

#include "../../sensor/AS5048.hpp"
#include "../../utils/tweakable_params.h"
#include "../../motor/Motor.hpp"
#include "../../motor/GM2804.hpp"
#include "../../foc_controller/FOCController.hpp"
#include "../../sensor/AS5048.hpp"
#include "../../pwm_driver/PWMDriverTeensy41.hpp"
#include "Arduino.h"
#include "../../utils/constants.h"
#include "../../interface/Interface.hpp"
#include "../../scan_path/ScanPath.hpp"
#include "../../state_machine/StateMachine.hpp"
#include "../../ldm/GarminV3HP.hpp"

/**
 * Noise test function that reads raw sensor data from AS5048 at ~5KHz
 * and sends it via serial as binary packets.
 * 
 * Usage: Call this function from setup() or loop() when you want to run the noise test.
 * The function will run indefinitely, sending sensor readings at 5KHz.
 * 
 * Data format: Each packet contains (little-endian):
 * - uint8_t command_token: Packet identifier (0xAB) (1 byte)
 * - float unfiltered_rotor_angle: Unfiltered rotor angle in radians (4 bytes)
 * - float rotor_angle: Filtered rotor angle in radians (4 bytes)
 * - float angular_velocity: Angular velocity in rad/s (4 bytes)
 * - uint16_t sensor_magnitude: Sensor magnitude reading (2 bytes)
 * - uint16_t agc_value: AGC value (2 bytes)
 * Total: 17 bytes per packet
 */
void noise_test() {
    Serial.begin(BAUD_RATE);
    StateMachine stateMachine;
    GarminV3HP ldm = GarminV3HP(GARMIN_SCL, GARMIN_SDA, GARMIN_MODE_PIN);
    AS5048 sensor1 = AS5048(SENSOR_SPI_CS_1);
    AS5048 sensor2 = AS5048(SENSOR_SPI_CS_2);
    PWMDriverTeensy41 pwmDriver1 = PWMDriverTeensy41(N1_1, N2_1, N3_1, EN_1);
    PWMDriverTeensy41 pwmDriver2 = PWMDriverTeensy41(N1_2, N2_2, N3_2, EN_2);
    GM2804 motor1(1, SENSOR_OFFSET_1);
    GM2804 motor2(2, SENSOR_OFFSET_2);
    FOCController focController1(sensor1, pwmDriver1, motor1);
    FOCController focController2(sensor2, pwmDriver2, motor2);
    SinusoidalScanPath scanPath(THETA_INC, PHI_INC, THETA_AMPLITUDE, PHI_AMPLITUDE, THETA_CENTRE, PHI_CENTRE);
    Interface interface(Serial, stateMachine, focController1, focController2, sensor1, sensor2, ldm, scanPath);
    // Motor doesn't have an init() method - it's initialized via constructor
    if (!focController2.init()) {
        Serial.println("Failed to initialize foc controller");
        return;
    }
    
    focController2.calibrate();
    
    Serial.println("Starting noise test at 5KHz...");
    Serial.println("Sending raw uint16_t binary data packets");
    delay(1000);
    
    while (true) {

        static unsigned long time1 = micros();
        if (micros() - time1 >= SENSOR_UPDATE_DT_US) {
            time1 = micros();
            if (!sensor2.update()) {
                Serial.println("Failed to update sensor");
                return;
            }
        
            // Read value after Mode and Jitter filters are applied
            float unfiltered_rotor_angle = sensor2.getRawAngle();
            float rotor_angle = sensor2.getLatestRotorAngle();
            float angular_velocity = sensor2.getAngularVelocity();
            uint16_t sensor_magnitude = sensor2.getSensorMagnitude();
            uint16_t agc_value = sensor2.getAGCValue();
            
            // Send packet as binary (little-endian)
            // uint8_t command_token (1 byte)
            Serial.write(0xAB);
            
            // float unfiltered_rotor_angle (4 bytes)
            uint8_t* unfiltered_rotor_angle_bytes = (uint8_t*)&unfiltered_rotor_angle;
            Serial.write(unfiltered_rotor_angle_bytes, 4);
            
            // float rotor_angle (4 bytes)
            uint8_t* angle_bytes = (uint8_t*)&rotor_angle;
            Serial.write(angle_bytes, 4);
            
            // float angular_velocity (4 bytes)
            uint8_t* velocity_bytes = (uint8_t*)&angular_velocity;
            Serial.write(velocity_bytes, 4);

            // // uint16_t sensor_magnitude (2 bytes)
            // uint8_t* sensor_magnitude_bytes = (uint8_t*)&sensor_magnitude;
            // Serial.write(sensor_magnitude_bytes, 2);

            // // uint16_t agc_value (2 bytes)
            // uint8_t* agc_value_bytes = (uint8_t*)&agc_value;
            // Serial.write(agc_value_bytes, 2);
        }
        delayMicroseconds(1);
    
        static unsigned long cmd_time = millis();
        if (millis() - cmd_time > 500 ) {
            cmd_time = millis();
            interface.commandListener();       
        }
    }
}
