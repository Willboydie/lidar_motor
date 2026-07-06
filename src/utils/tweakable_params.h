#ifndef TWEAKABLE_PARAMS_H
#define TWEAKABLE_PARAMS_H

#include "constants.h"

// PID Controllers 
#define VELOCITY_P 1.0f
#define VELOCITY_I 0.0f
#define VELOCITY_D 0.0f
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
#define ANGLE_P 1.0f
#define ANGLE_I 0.5f
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
#define ANGLE_LOW_PASS_FILTER_TAU 0.0f
#define VELOCITY_LOW_PASS_FILTER_TAU 0.2f
#define FOC_VOLTAGE_LPF_TAU 0.0f

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

// Telemetry
#define FOC_STATUS_PERIOD_MS 5 // legacy bench plotter period (dual_foc_test)
#define CONTROL_PACKET_PERIOD_US UPDATE_PERIOD_US  // 500 Hz closed-loop telemetry
#define STATUS_PACKET_PERIOD_MS 100                // 10 Hz housekeeping telemetry

// Current sense (SimpleFOC Shield 3.2, ACS712-05B)
// ACS712-05B is 185 mV/A at the sensor; the shield maps its 0-5 V swing into
// 0-3.3 V at the MCU pin, so the effective at-pin sensitivity is 185 * 3.3/5.
// Nominal only — Stage 9 offset calibration / polarity check refines this.
#define ACS712_MV_PER_AMP (185.0f * 3.3f / 5.0f) // = 122.1 mV/A at the ADC pin

// FOC Timing
// Cascaded-loop rates. loop() is called as fast as the superloop allows and each
// cascade stage is rate-gated off micros() to these targets (standard FOC layout:
// fast inner current loop, slower velocity, slowest outer angle). Edit freely.
#define CURRENT_LOOP_HZ  20000 // innermost: commutation + current PI / voltage model
#define VELOCITY_LOOP_HZ  5000 // middle: velocity error -> target q current
#define ANGLE_LOOP_HZ     1000 // outermost: angle error -> target velocity
#define SENSOR_UPDATE_DT_US 5 // the update rate of the AS5048, 11250Hz
#define VELOCITY_ESTIMATION_DT_US 89
#define VELOCITY_ESTIMATE_N (VELOCITY_ESTIMATION_DT_US / SENSOR_UPDATE_DT_US)


#endif