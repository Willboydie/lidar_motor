#pragma once

#include <Arduino.h>
#include <cmath>
#include "../sensor/Sensor.hpp"
#include "../pwm_driver/PWMDriver.hpp"
#include "../pid_control/PIDControl.hpp"
#include "../motor/Motor.hpp"
#include "../filter/LowPassFilter.hpp"
#include "../utils/constants.h"
#include "../utils/quick_defs.h"
#include "../utils/tweakable_params.h"
#include "../state_machine/StateMachine.hpp"

constexpr unsigned long VELOCITY_DT_US  = (1000000UL / VELOCITY_LOOP_HZ);
constexpr unsigned long ANGLE_DT_US     = (1000000UL / ANGLE_LOOP_HZ);
constexpr unsigned long CURRENT_DT_US   = (1000000UL / CURRENT_LOOP_HZ);

class FOCController {
public:
    FOCController(Sensor& _sensor, PWMDriver& _pwm_driver, Motor& _motor);
    ~FOCController() { deinit(); }
    bool init();
    void deinit();
    void enable();
    void disable();
    void calibrate();
    bool loop();

    float getAngularVelocity() { return angular_velocity; }
    float getMechanicalAngle() { return mechanical_angle; }
    float getElectricalAngle() { return electrical_angle; }
    float getAngularError() { return angular_error; }
    float getVelocityError() { return velocity_error; }
    float getTargetAngle() { return target_angle; }
    float getTargetVelocity() { return target_velocity; }
    float getTargetQCurrent() { return target_q_current; }
    float getTargetDCurrent() { return target_d_current; }
    float getVq() { return Vq; }
    float getVd() { return Vd; }
    float getMechanicalOffset() { return motor_electrical_offset; }
    float getElectricalOffset() { return motor_electrical_offset; }

    void setControlMode(ControlMode _controlMode);
    void setTargetAngle(float _target_world_angle);
    void setTargetVelocity(float _target_velocity) { target_velocity = _target_velocity; }
    void setVelocityP(float _velocity_p) { PID_velocity_controller.setP(_velocity_p); }
    void setVelocityI(float _velocity_i) { PID_velocity_controller.setI(_velocity_i); }
    void setVelocityD(float _velocity_d) { PID_velocity_controller.setD(_velocity_d); }
    void setVelocityRamp(float _velocity_ramp) { PID_velocity_controller.setRamp(_velocity_ramp); }
    void setAngleP(float _angle_p) { PID_angle_controller.setP(_angle_p); }
    void setAngleI(float _angle_i) { PID_angle_controller.setI(_angle_i); }
    void setAngleD(float _angle_d) { PID_angle_controller.setD(_angle_d); }
    void setAngleRamp(float _angle_ramp) { PID_angle_controller.setRamp(_angle_ramp); }
    void setFOCVoltageLPFTau(float _foc_voltage_lpf_tau) { LPF_voltage.setTau(_foc_voltage_lpf_tau); }
    void setVelocityFeedForward(float gain) { K_vff = gain; }

    // Angle Tracking
    void resetAngleTracking() { angle_tracking_count = 0; }
    float angle_tracking[ANGLE_TRACKING_LIMIT];
    int angle_tracking_count = 0;

    float estimate_Vq(float Iq);

    // void setPhaseVoltage(float _Vq, float _Vd, float _angle_el) { Vq = _Vq; Vd = _Vd; electrical_angle = _angle_el; setPhaseVoltage(); }
    const int motor_number;

private:
    Sensor& sensor;
    PWMDriver& pwm_driver;

    void setPhaseVoltage();
    float calculate_electrical_angle();

    float calibrateLoopPeriod(uint16_t num_samples);
    void setLoopPeriod();
    
    // state variables
    float angular_velocity;
    float mechanical_angle;
    float electrical_angle;
    float angular_error;
    float velocity_error;

    // FOC Targets
    float target_angle;
    float target_velocity;
    float target_q_current;
    float target_d_current = 0.0f;  // set to zero while not current sensing

    uint32_t last_current_us  = 0;
    uint32_t last_velocity_us = 0;
    uint32_t last_angle_us    = 0;
    uint32_t last_sensor_update_us = 0;


    // FOC Key Values
    float Iq = 0.0f;
    float Id = 0.0f;
    float Vq = 0.0f;
    float Vd = 0.0f;
    float Valpha;
    float Vbeta;
    float Va;
    float Vb;
    float Vc;

    // FOC utils
    float _sa;
    float _ca;
    float center;


    // PID controllers
    PIDControl PID_angle_controller{ANGLE_P, ANGLE_I, ANGLE_D, ANGLE_RAMP, VELOCITY_LOWER_LIMIT, VELOCITY_UPPER_LIMIT, ANGLE_DT_US*1e-6f}; // angular error -> target velocity
    PIDControl PID_velocity_controller{VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_RAMP, CURRENT_LOWER_LIMIT, CURRENT_UPPER_LIMIT, VELOCITY_DT_US*1e-6f};  // velocity error -> target q current

    _LowPassFilter LPF_voltage{FOC_VOLTAGE_LPF_TAU};

    float K_vff = 0.0f;   // A / (rad/s) - velocity feedforward gain
  
    bool enabled = false;
    bool initialised = false;
    ControlMode controlMode = ANGLE;

    // motor constants
    const float motor_phase_resistance;
    const float motor_K_v;
    const float motor_K_e;
    const float motor_pole_pairs;
    const float motor_sensor_direction;
    const float motor_electrical_offset;
};