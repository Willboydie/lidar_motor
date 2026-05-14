#ifndef CONSTANTS_H
#define CONSTANTS_H

// Pins
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
// // miso = 12 (yellow), mosi = 11 (black), sck = 13 (red)

// Garmin Pins
#define GARMIN_SCL 19 // green
#define GARMIN_SDA 18 // blue
#define GARMIN_MODE_PIN 20  // MODE/STATUS pin for interrupt-driven readings (yellow)


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