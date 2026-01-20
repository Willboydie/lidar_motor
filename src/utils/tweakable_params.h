#ifndef TWEAKABLE_PARAMS_H
#define TWEAKABLE_PARAMS_H

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

// Motor Parameters
#define MOTOR_VOLTAGE_LIMIT 12.0f
#define MOTOR_CURRENT_LIMIT 0.8f
#define VOLTAGE_POWER_SUPPLY 12.0f
#define SENSOR_DIRECTION 1 // CCW = 1, CW = -1. GM2804 with AS5048A is CCW.


// PID Controllers 
#define VELOCITY_P 0.2f
#define VELOCITY_I 0.2f
#define VELOCITY_D 0.0f
#define VELOCITY_RAMP 0.0f   // limit acceleration
#define CURRENT_UPPER_LIMIT MOTOR_CURRENT_LIMIT
#define CURRENT_LOWER_LIMIT -MOTOR_CURRENT_LIMIT
// -------------------------------------------------
#define ANGLE_P 10.0f
#define ANGLE_I 0.0f
#define ANGLE_D 0.0f
#define ANGLE_RAMP 0.0f
#define VELOCITY_UPPER_LIMIT 1000.0f   // rad/s
#define VELOCITY_LOWER_LIMIT -1000.0f


// FOC Timing Parameters
#define VELOCITY_LOOP_HZ  1000    // 1 kHz
#define ANGLE_LOOP_HZ     200     // 200 Hz


// Sensor Parameters
#define SENSOR_OFFSET_1  5.67f // angle units
#define SENSOR_OFFSET_2 4.02f // angle units
#define VELOCITY_MIN_TIME_INCREMENT 100 // 100us
#define ANGULAR_VELOCITY_LPF_TAU 0.01f
#define FOC_VOLTAGE_LPF_TAU 0.001f
#define BIT_RESOLUTION 14
#define CPR (1 << BIT_RESOLUTION) // 14 bit resolution


// Calibration Parameters
#define ALIGNMENT_VOLTAGE 3.0f // V
#define WORLD_OFFSET_X 5.30f
#define WORLD_OFFSET_Y 0.57f


// PWM Driver Parameters
#define PWM_FREQUENCY 25000 // 25khz
#define PWM_FREQUENCY_MAX 50000 // 50khz

// Angle Tracking Parameters
#define ANGLE_TRACKING_LIMIT 5000
#define ANGLE_TRACKING_WINDOW_SIZE 100

// Serial Parameters
#define BAUD_RATE 460800

// Scan Path Parameters
#define UPDATE_PERIOD_US 2000 // 2ms / 500Hz
#define THETA_INC 1500007
#define PHI_INC 3200003
#define THETA_AMPLITUDE 0.79
#define PHI_AMPLITUDE 0.3
#define THETA_CENTRE 0.0
#define PHI_CENTRE -0.3

#endif

// 0.1 rad diff -> 1 rad/s target vel -> 0.8 A q current