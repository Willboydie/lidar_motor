#ifndef TWEAKABLE_PARAMS_H
#define TWEAKABLE_PARAMS_H

#include "constants.h"

// PID Controllers
#define VELOCITY_P_X 1.0f
#define VELOCITY_I_X 0.0f
#define VELOCITY_D_X 0.0f
#define VELOCITY_P_Y 1.0f
#define VELOCITY_I_Y 0.0f
#define VELOCITY_D_Y 0.0f
#define VELOCITY_RAMP 0.0f   // limit acceleration
#define CURRENT_UPPER_LIMIT MOTOR_CURRENT_LIMIT
#define CURRENT_LOWER_LIMIT -MOTOR_CURRENT_LIMIT
// -------------------------------------------------
// Inner current-loop PI (STM32G4 only — measured-current Vd/Vq control).
// Output is a phase voltage [V], clamped to MOTOR_VOLTAGE_LIMIT (anti-windup).
// Conservative starting point; tune live in the L12 current-PI bench test.
#define CURRENT_P 1.0f      // V per A of current error
#define CURRENT_I 50.0f     // V per (A*s) — the workhorse against R*I drop
#define CURRENT_D 0.0f
#define CURRENT_RAMP 0.0f
// -------------------------------------------------
#define ANGLE_P_X 6.0f
#define ANGLE_I_X 12.0f
#define ANGLE_D_X 0.0f
#define ANGLE_P_Y 6.0f
#define ANGLE_I_Y 12.0f
#define ANGLE_D_Y 0.0f
#define ANGLE_RAMP 0.0f
#define VELOCITY_UPPER_LIMIT 15.0f   // rad/s
#define VELOCITY_LOWER_LIMIT -15.0f

// Feedforward
#define DEFAULT_VELOCITY_FEEDFORWARD_GAIN 0.0f
#define DEFAULT_ACCELERATION_FEEDFORWARD_GAIN 0.0f
#define DEFAULT_VELOCITY_DELAY_US 100

// S-Curve Parameters
#define JERK_MAX 10.0f
#define ACCEL_MAX 5.0f

// Scan Path Parameters
#define DEFAULT_X_SCAN_FREQUENCY 1.5f
#define DEFAULT_Y_SCAN_FREQUENCY 3.0f
#define DEFAULT_X_SCAN_AMPLITUDE 0.2f
#define DEFAULT_Y_SCAN_AMPLITUDE 0.2f
#define DEFAULT_X_SCAN_PHASE 0.0f
#define DEFAULT_Y_SCAN_PHASE 0.0f

#define X_SCAN_AMPLITUDE_LIMIT 0.6f
#define Y_SCAN_AMPLITUDE_LIMIT 0.6f

// Neutral Scan Position
#define DEFAULT_NEUTRAL_X_POS 3.54f
#define DEFAULT_NEUTRAL_Y_POS 3.39f

//Filtering
#define ANGLE_LOW_PASS_FILTER_TAU 0.0f
#define VELOCITY_LOW_PASS_FILTER_TAU 0.2f
#define FOC_VOLTAGE_LPF_TAU 0.0f

#define SOFT_DEADBAND_WIDTH 0.05f
#define HARD_DEADBAND_WIDTH 0.001f

#define FILTER_WINDOW_SIZE 80 // N * sensor update rate = window size in seconds
#define MAX_FILTER_WINDOW_SIZE 128
#define MEAN_FILTER_WINDOW_SIZE 20

#define DEFAULT_JITTER_AMPLITUDE 3
#define DEFAULT_JITTER_PERIOD 1

#define DEFAULT_KALMAN_QA 1e-4f
#define DEFAULT_KALMAN_R 3e-7f

#define NOTCH_FILTER_FREQ 330.0f // Removing noise centred at 330Hz
#define NOTCH_FILTER_Q 20.0f // Q = f0 / bandwidth
#define NOTCH_FILTER_SAMPLE_RATE 80000.0f 

#define SENSOR_DISCARD_BITS 2
#define ADAPTED_CPR (CPR / (1 << SENSOR_DISCARD_BITS))

#define ACCEL_LIMIT 1000.0f

// Telemetry
#define FOC_STATUS_PERIOD_MS 5 // legacy bench plotter period (dual_foc_test)
#define CONTROL_PACKET_PERIOD_US 1000  // 1KHz closed-loop telemetry
#define STATUS_PACKET_PERIOD_MS 100    // 10 Hz housekeeping telemetry

// Angle Limits
#define DEFAULT_X_LOWER_LIMIT 2.5f
#define DEFAULT_X_UPPER_LIMIT 4.2f
#define DEFAULT_Y_LOWER_LIMIT 2.6f
#define DEFAULT_Y_UPPER_LIMIT 3.9f

#define DEFAULT_ANGLE_ERROR_MEAN_WINDOW_LENGTH 20
#define TARGET_ANGLE_TOLERANCE 0.01f

// Current sense (SimpleFOC Shield 3.2, ACS712-05B)
// ACS712-05B is 185 mV/A at the sensor; the shield maps its 0-5 V swing into
// 0-3.3 V at the MCU pin, so the effective at-pin sensitivity is 185 * 3.3/5.
// Nominal only — Stage 9 offset calibration / polarity check refines this.
#define ACS712_MV_PER_AMP (185.0f * 3.3f / 5.0f) // = 122.1 mV/A at the ADC pin

// FOC Timing
// Cascaded-loop rates. loop() is called as fast as the superloop allows and each
// cascade stage is rate-gated off micros() to these targets (standard FOC layout:
// fast inner current loop, slower velocity, slowest outer angle). Edit freely.
#define CURRENT_LOOP_HZ  25000 // innermost: commutation + current PI / voltage model
#define VELOCITY_LOOP_HZ  5000 // middle: velocity error -> target q current
#define ANGLE_LOOP_HZ     1000 // outermost: angle error -> target velocity
#define SENSOR_UPDATE_DT_US 5 // the update rate of the AS5048, 11250Hz
#define VELOCITY_ESTIMATION_DT_US 89
#define VELOCITY_ESTIMATE_N (VELOCITY_ESTIMATION_DT_US / SENSOR_UPDATE_DT_US)

#endif