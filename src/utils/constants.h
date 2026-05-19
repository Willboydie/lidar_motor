#ifndef CONSTANTS_H
#define CONSTANTS_H

// Pins
#if defined(STM32G4xx)
// motor 1
#define N1_X PA7  // D11
#define N2_X PC7 // D9
#define N3_X PB3 // D3
#define EN_X PA8 // D7
#define SENSOR_SPI_CS_X PB2 // white
// motor 2
#define N1_Y PA5  // D13
#define N2_Y PB6 // D10
#define N3_Y PB4 // D5
#define EN_Y PA9 // D8
#define SENSOR_SPI_CS_Y PB1 // white
// SPI (SimpleFOC shield): MISO=PB14, MOSI=PB15, SCK=PB13
#define SENSOR_SPI_MISO PB14
#define SENSOR_SPI_MOSI PB15
#define SENSOR_SPI_SCK PB13

// Garmin Pins
#define GARMIN_SCL PB8 // green
#define GARMIN_SDA PB9 // blue
#define GARMIN_MODE_PIN PC9  // MODE/STATUS pin for interrupt-driven readings (yellow)

#define ADC_CS_X_1 PA0 // A0
#define ADC_CS_X_2 PA4 // A2
#define ADC_CS_Y_1 PA1 // A1
#define ADC_CS_Y_2 PB0 // A3

#elif defined(ARDUINO_TEENSY41)

// motor 1
#define N1_1 6  // orange
#define N2_1 36 // yellow
#define N3_1 37 // green
#define EN_1 22 // blue
#define SENSOR_SPI_CS_1 10 // white
// motor 2
#define N1_2 7  // orange
#define N2_2 8 // yellow
#define N3_2 9 // green
#define EN_2 23 // blue
#define SENSOR_SPI_CS_2 15 // white
// SPI: MISO=12, MOSI=11, SCK=13 (Arduino default SPI bus)

// Garmin Pins
#define GARMIN_SCL 19 // green
#define GARMIN_SDA 18 // blue
#define GARMIN_MODE_PIN 20  // MODE/STATUS pin for interrupt-driven readings (yellow)

// Shared names used by main.cpp (same motor indices as STM32G4xx block)
#define N1_X N1_1
#define N2_X N2_1
#define N3_X N3_1
#define EN_X EN_1
#define N1_Y N1_2
#define N2_Y N2_2
#define N3_Y N3_2
#define EN_Y EN_2
#define SENSOR_SPI_CS_X SENSOR_SPI_CS_1
#define SENSOR_SPI_CS_Y SENSOR_SPI_CS_2

#endif

// Motor
#define MOTOR_VOLTAGE_LIMIT 12.0f
#define MOTOR_CURRENT_LIMIT 0.8f
#define VOLTAGE_POWER_SUPPLY 12.0f
#define SENSOR_DIRECTION 1 // CCW = 1, CW = -1. GM2804 with AS5048A is CCW.

// Sensor
#define SENSOR_OFFSET_1  5.67f // electrical offset in radians
#define SENSOR_OFFSET_2 4.02f
#define BIT_RESOLUTION 14
#define CPR (1 << BIT_RESOLUTION) // 14 bit resolution


// Calibration
#define ALIGNMENT_VOLTAGE 3.0f // V
#define WORLD_OFFSET_X 5.28f
#define WORLD_OFFSET_Y 2.11f


// PWM Driver
// NB: stm32duino's pins_arduino.h defines PWM_FREQUENCY=1000 (default for
// analogWrite). Prefix our project's macro to avoid that collision.
#define MOTOR_PWM_FREQUENCY 25000 // 25khz
#define MOTOR_PWM_FREQUENCY_MAX 50000 // 50khz


// Angle Tracking
#define ANGLE_TRACKING_LIMIT 5000
#define ANGLE_TRACKING_WINDOW_SIZE 100


// Scan Path
#define SCAN_PATH_UPDATE_PERIOD_US 2000 // 2ms / 500Hz


// Serial Parameters
#define BAUD_RATE 460800


// EXACT VALUES
#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _PI_4 0.7853981634f
#define _2PI 6.28318530718f
#define _4PI 12.56637061436f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f
#define _RPM_TO_RADS 0.10471975512f


#endif