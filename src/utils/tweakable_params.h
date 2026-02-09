#ifndef TWEAKABLE_PARAMS_H
#define TWEAKABLE_PARAMS_H

#include "constants.h"

// PID Controllers 
#define VELOCITY_P 0.0f
#define VELOCITY_I 0.0f
#define VELOCITY_D 0.0f
#define VELOCITY_RAMP 0.0f   // limit acceleration
#define CURRENT_UPPER_LIMIT MOTOR_CURRENT_LIMIT
#define CURRENT_LOWER_LIMIT -MOTOR_CURRENT_LIMIT
// -------------------------------------------------
#define ANGLE_P 0.0f
#define ANGLE_I 0.0f
#define ANGLE_D 0.0f
#define ANGLE_RAMP 0.0f
#define VELOCITY_UPPER_LIMIT 15.0f   // rad/s
#define VELOCITY_LOWER_LIMIT -15.0f


// Scan Path Parameters
#define UPDATE_PERIOD_US 2000 // 2ms / 500Hz
#define THETA_INC 1500007
#define PHI_INC 3200003
#define THETA_AMPLITUDE 0.79
#define PHI_AMPLITUDE 0.3
#define THETA_CENTRE 0.0
#define PHI_CENTRE -0.3


//Filtering
#define ANGLE_LOW_PASS_FILTER_TAU 0.0f  // 250Hz cutoff frequency
#define VELOCITY_LOW_PASS_FILTER_TAU 0.5f  // 150Hz cutoff frequency
#define FOC_VOLTAGE_LPF_TAU 0.00f

#define SOFT_DEADBAND_WIDTH 0.05f
#define HARD_DEADBAND_WIDTH 0.001f

#define FILTER_WINDOW_SIZE 80 // N * sensor update rate = window size in seconds
#define MAX_FILTER_WINDOW_SIZE 128

#define DEFAULT_JITTER_AMPLITUDE 3
#define DEFAULT_JITTER_PERIOD 1

#define DEFAULT_KALMAN_QA 1e-4f
#define DEFAULT_KALMAN_R 3e-7f

#define NOTCH_FILTER_FREQ 330.0f // Removing noise centred at 330Hz
#define NOTCH_FILTER_Q 20.0f // Q = f0 / bandwidth
#define NOTCH_FILTER_SAMPLE_RATE 80000.0f 

#define SENSOR_DISCARD_BITS 3
#define ADAPTED_CPR (CPR / (1 << SENSOR_DISCARD_BITS))

#define ACCEL_LIMIT 1000.0f

// FOC Timing
// #define ANGLE_LOOP_HZ     50000
// #define VELOCITY_LOOP_HZ   50000
// #define CURRENT_LOOP_HZ   50000
#define SENSOR_UPDATE_DT_US 20 // the update rate of the AS5048, 11250Hz
#define VELOCITY_ESTIMATION_DT_US 89
#define VELOCITY_ESTIMATE_N (VELOCITY_ESTIMATION_DT_US / SENSOR_UPDATE_DT_US)


#endif