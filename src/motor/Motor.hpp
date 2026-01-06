#pragma once

#include <Arduino.h>
#include <cmath>
#include "../sensor/Sensor.hpp"
#include "../pwm_driver/PWMDriver.hpp"
#include "../pid_control/PIDControl.hpp"
#include "../utils/constants.h"
#include "../utils/quick_defs.h"
#include "../utils/tweakable_params.h"
#include "../state_machine/StateMachine.hpp"

class Motor {
public:
    Motor(Sensor& _sensor, PWMDriver& _pwm_driver, int _motor_number);
    bool init();
    void deinit();
    void enable();
    void disable();
    bool loopFOC();

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
    float getMechanicalOffset() { return mechanical_offset; }
    float getElectricalOffset() { return electrical_offset; }

    void setControlMode(ControlMode _controlMode) { controlMode = _controlMode; }
    void setTargetAngle(float _target_angle) { target_angle = _target_angle; }
    void setTargetVelocity(float _target_velocity) { target_velocity = _target_velocity; }
    void setVelocityP(float _velocity_p) { PID_velocity_controller.setP(_velocity_p); }
    void setVelocityI(float _velocity_i) { PID_velocity_controller.setI(_velocity_i); }
    void setVelocityD(float _velocity_d) { PID_velocity_controller.setD(_velocity_d); }
    void setVelocityRamp(float _velocity_ramp) { PID_velocity_controller.setRamp(_velocity_ramp); }
    void setAngleP(float _angle_p) { PID_angle_controller.setP(_angle_p); }
    void setAngleI(float _angle_i) { PID_angle_controller.setI(_angle_i); }
    void setAngleD(float _angle_d) { PID_angle_controller.setD(_angle_d); }
    void setAngleRamp(float _angle_ramp) { PID_angle_controller.setRamp(_angle_ramp); }
    void setFOCVoltageLPF(float _foc_voltage_lpf) { LPF_voltage.setTau(_foc_voltage_lpf); }

    void resetAngleTracking() { angle_count = 0; }
    // Angle Tracking
    float angle_tracking[ANGLE_TRACKING_LIMIT];
    int angle_count = 0;
    const int motor_number;
    float estimate_Vq(float Iq);

    void setPhaseVoltage(float _Vq, float _Vd, float _angle_el) { Vq = _Vq; Vd = _Vd; electrical_angle = _angle_el; setPhaseVoltage(); }
private:
    Sensor& sensor;
    PWMDriver& pwm_driver;

    void updateFOC();
    void setPhaseVoltage();
    bool findMechanicalOffset();
    bool findElectricalOffset();
    float calculate_electrical_angle();

    
    // motor constants
    float electrical_offset; // hardcoded offset for loaded motor.
    float mechanical_offset;
    float phase_resistance;
    float K_v;
    float K_e;
    
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
    PIDControl PID_angle_controller{ANGLE_P, ANGLE_I, ANGLE_D, ANGLE_RAMP, VELOCITY_LOWER_LIMIT, VELOCITY_UPPER_LIMIT}; // angular error -> target velocity
    PIDControl PID_velocity_controller{VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_RAMP, CURRENT_LOWER_LIMIT, CURRENT_UPPER_LIMIT};  // velocity error -> target q current

    _LowPassFilter LPF_voltage{FOC_VOLTAGE_LPF_TAU};
  

    bool enabled = false;
    ControlMode controlMode = ANGLE;

};