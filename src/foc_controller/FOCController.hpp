#pragma once

#include <Arduino.h>
#include <cmath>
#include "../sensor/Sensor.hpp"
#include "../pwm_driver/PWMDriver.hpp"
#include "../pid_control/PIDControl.hpp"
#include "../motor/Motor.hpp"
#include "../filter/LowPassFilter.hpp"
#include "../current_sense/InlineCurrentSense.hpp"
#include "../utils/constants.h"
#include "../utils/quick_defs.h"
#include "../utils/tweakable_params.h"
#include "../state_machine/StateMachine.hpp"

// Cascaded-loop periods [us], derived from the editable rates in tweakable_params.h.
// loop() runs the whole cascade at (at most) the current-loop rate and gates the
// velocity/angle stages down to their own rates off micros().
constexpr uint32_t CURRENT_LOOP_DT_US  = 1000000UL / CURRENT_LOOP_HZ;
constexpr uint32_t VELOCITY_LOOP_DT_US = 1000000UL / VELOCITY_LOOP_HZ;
constexpr uint32_t ANGLE_LOOP_DT_US    = 1000000UL / ANGLE_LOOP_HZ;

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
    float getCommandedElectricalAngle() const {
        return normalise_angle(
            target_angle * motor_pole_pairs * motor_sensor_direction - motor_electrical_offset);
    }
    float getAngularError() { return angular_error; }
    float getVelocityError() { return velocity_error; }
    float getTargetAngle() { return target_angle; }
    float getTargetVelocity() { return target_velocity; }
    float getTargetQCurrent() { return target_q_current; }
    float getTargetDCurrent() { return target_d_current; }
    float getMeasuredQCurrent() { return Iq; }  // TODO: derive from Ia/Ib/Ic once current sensing is implemented
    float getMeasuredDCurrent() { return Id; }
    float getVq() { return Vq; }
    float getVd() { return Vd; }
    float getVa() { return Va; }  // phase voltage as sent to PWM [V]
    float getVb() { return Vb; }
    float getVc() { return Vc; }

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

    float getVelocityP() const { return PID_velocity_controller.P; }
    float getVelocityI() const { return PID_velocity_controller.I; }
    float getVelocityD() const { return PID_velocity_controller.D; }
    float getVelocityRamp() const { return PID_velocity_controller.output_ramp; }
    float getAngleP() const { return PID_angle_controller.P; }
    float getAngleI() const { return PID_angle_controller.I; }
    float getAngleD() const { return PID_angle_controller.D; }
    float getAngleRamp() const { return PID_angle_controller.output_ramp; }
    float getFOCVoltageLPFTau() const { return LPF_voltage.getTau(); }
    float getVelocityFeedForward() const { return K_vff; }

    // Inner current-loop PI gains (STM32G4 measured-current control; no-op path on
    // targets without current sensing). Exposed for live tuning in the L12 bench test.
    void setCurrentP(float _p) { PID_current_q.setP(_p); PID_current_d.setP(_p); }
    void setCurrentI(float _i) { PID_current_q.setI(_i); PID_current_d.setI(_i); }
    void setCurrentD(float _d) { PID_current_q.setD(_d); PID_current_d.setD(_d); }

    // Attach the inline current sense (STM32G4). When set, the inner loop closes a
    // measured-current PI (Vd/Vq from PI on Iq/Id error); when null, or on targets
    // without sensing, the inner loop falls back to the estimate_Vq voltage model.
    void setCurrentSense(InlineCurrentSense& cs) { current_sense = &cs; }

    // L12 bench helper: run one inner current-loop step against a commanded Iq/Id,
    // bypassing the velocity/angle cascade. Reads the live encoder, commutates off
    // it, runs the current loop and drives the phases. Returns false on sensor fault.
    bool currentLoopStep(float iq_target, float id_target = 0.0f);

    void setVq(float _Vq) { Vq = _Vq; }
    void setVd(float _Vd) { Vd = _Vd; }
    void setElectricalAngle(float _electrical_angle) { electrical_angle = _electrical_angle; }

    float estimate_Vq(float Iq);

    // void setPhaseVoltage(float _Vq, float _Vd, float _angle_el) { Vq = _Vq; Vd = _Vd; electrical_angle = _angle_el; setPhaseVoltage(); }
    const int motor_number;
    void setPhaseVoltage();

private:
    Sensor& sensor;
    PWMDriver& pwm_driver;

    // void setPhaseVoltage();
    float calculate_electrical_angle();

    // Innermost cascade stage. Compile-time selected: STM32G4 closes a measured-
    // current PI (Vd/Vq); other targets use the estimate_Vq voltage model.
    void currentLoop();

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

    // Cascade rate-gating timestamps [us].
    uint32_t last_current_us  = 0;
    uint32_t last_velocity_us = 0;
    uint32_t last_angle_us    = 0;

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
    // Inner current PI: current error [A] -> phase voltage [V] (STM32G4). Output
    // limited to the voltage rail so anti-windup clamps the integral in volts.
    PIDControl PID_current_q{CURRENT_P, CURRENT_I, CURRENT_D, CURRENT_RAMP, -MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT};
    PIDControl PID_current_d{CURRENT_P, CURRENT_I, CURRENT_D, CURRENT_RAMP, -MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT};

    // Inline current sense (STM32G4). Null on targets without sensing -> voltage model.
    InlineCurrentSense* current_sense = nullptr;

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
    float motor_electrical_offset;
};