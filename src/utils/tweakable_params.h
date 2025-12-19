#ifndef TWEAKABLE_PARAMS_H
#define TWEAKABLE_PARAMS_H

// Pins
// #define N1 2  // Adjust based on your driver connection
// #define N2 3 // Adjust based on your driver connection
// #define N3 6 // Adjust based on your driver connection
// #define EN 22 // If your driver has an enable pin
// #define SENSOR_SPI_CS 10
#define N1 7  // Adjust based on your driver connection
#define N2 8 // Adjust based on your driver connection
#define N3 9 // Adjust based on your driver connection
#define EN 23 // If your driver has an enable pin
#define SENSOR_SPI_CS 15
// // miso = 12, mosi = 11, sck = 13

// Motor Parameters
#define POLE_PAIRS 7
#define PHASE_RESISTANCE 9.0f
#define K_V 169.0f
#define K_E 0.057f
#define MOTOR_VOLTAGE_LIMIT 12.0f
#define MOTOR_CURRENT_LIMIT 0.8f
#define VOLTAGE_POWER_SUPPLY 12.0f
#define SENSOR_DIRECTION 1 // CCW = 1, CW = -1. GM2804 with AS5048A is CCW.


// PID Controllers 
#define VELOCITY_P 1.0f
#define VELOCITY_I 1.0f
#define VELOCITY_D 0.0f
#define VELOCITY_RAMP 0.0f   // limit acceleration
#define CURRENT_UPPER_LIMIT MOTOR_CURRENT_LIMIT
#define CURRENT_LOWER_LIMIT -MOTOR_CURRENT_LIMIT
// -------------------------------------------------
#define ANGLE_P 10.0f
#define ANGLE_I 0.0f
#define ANGLE_D 0.0f
#define ANGLE_RAMP 0.0f
#define VELOCITY_UPPER_LIMIT 0.5f   // rad/s
#define VELOCITY_LOWER_LIMIT -0.5f


// Sensor Parameters
#define SENSOR_OFFSET 4.03f // rad. Hardcoded offset for loaded motor.
#define VELOCITY_MIN_TIME_INCREMENT 100 // 100us
#define ANGULAR_VELOCITY_LPF_TAU 0.5f
#define FOC_VOLTAGE_LPF_TAU 0.0f


// Calibration Parameters
#define ALIGNMENT_VOLTAGE 3.0f // V


// PWM Driver Parameters
#define PWM_FREQUENCY 25000 // 25khz
#define PWM_FREQUENCY_MAX 50000 // 50khz


#endif

// 0.1 rad diff -> 1 rad/s target vel -> 0.8 A q current