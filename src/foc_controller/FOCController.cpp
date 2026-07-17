#include "FOCController.hpp"

FOCController::FOCController(Sensor& _sensor, PWMDriver& _pwm_driver, Motor& _motor) 
: motor_number(_motor.motor_number)
, sensor(_sensor)
, pwm_driver(_pwm_driver)
, motor_phase_resistance(_motor.phase_resistance)
, motor_K_v(_motor.K_v)
, motor_K_e(_motor.K_e)
, motor_pole_pairs(_motor.pole_pairs)
, motor_sensor_direction(_motor.sensor_direction)
, motor_electrical_offset(_motor.electrical_offset)
{};

static inline bool isMotorX(int motor_number) { return motor_number == 1; }

bool FOCController::init() {
    if (!sensor.isInitialised()) {
        faultManager.addFault(isMotorX(motor_number) ? FAULT_FOC_X_INIT : FAULT_FOC_Y_INIT);
        Debug::msg("FOC CONTROLLER INIT FAILED: SENSOR NOT INITIALISED");
        return false;
    } else if (!pwm_driver.isInitialised()) {
        faultManager.addFault(isMotorX(motor_number) ? FAULT_FOC_X_INIT : FAULT_FOC_Y_INIT);
        Debug::msg("FOC CONTROLLER INIT FAILED: PWM DRIVER NOT INITIALISED");
        return false;
    }
    
    mechanical_angle = sensor.getLatestRotorAngle();
    electrical_angle = calculate_electrical_angle();
    target_angle = mechanical_angle;
    target_velocity = 0.0f;
    target_q_current = 0.0f;
    target_d_current = 0.0f;
    last_current_us = last_velocity_us = last_angle_us = 0; // run every stage on first loop()
    initialised = true;
    return true;
}

// Cascaded FOC control, standard three-loop layout:
//   angle loop (ANGLE_LOOP_HZ)  -> target_velocity
//   velocity loop (VELOCITY_LOOP_HZ) -> target_q_current
//   current loop (CURRENT_LOOP_HZ)   -> Vd/Vq -> setPhaseVoltage()
// loop() is called as fast as the superloop allows; the whole cascade advances at
// (at most) the current-loop rate, and the outer stages are further rate-gated off
// micros(). The current loop owns the sensor refresh + commutation so the outer
// stages always run against fresh angle/velocity.
bool FOCController::loop() {
    if (!enabled) return false;
    if (!initialised) {
        faultManager.addFault(FAULT_SYS_NOT_INITIALISED);
        return false;
    }

    const uint32_t now_us = micros();
    if ((uint32_t)(now_us - last_current_us) < CURRENT_LOOP_DT_US) return true;
    last_current_us = now_us;

    if (!sensor.update()) {
        faultManager.addFault(isMotorX(motor_number) ? FAULT_ENC_X_UPDATE : FAULT_ENC_Y_UPDATE);
        return false;
    }
    mechanical_angle = sensor.getLatestRotorAngle();
    electrical_angle = calculate_electrical_angle();
    angular_velocity = sensor.getFilteredAngularVelocity();

    // --- Angle loop (outermost): angular error -> target velocity ---
    if (controlMode == ANGLE && (uint32_t)(now_us - last_angle_us) >= ANGLE_LOOP_DT_US) {
        last_angle_us = now_us;
        angular_error = findAngularError(target_angle, mechanical_angle);
        target_velocity = PID_angle_controller(angular_error);
    }

    // --- Velocity loop: velocity error -> target q current ---
    if ((uint32_t)(now_us - last_velocity_us) >= VELOCITY_LOOP_DT_US) {
        last_velocity_us = now_us;
        velocity_error = target_velocity - angular_velocity;
        const float Iq_pid = PID_velocity_controller(velocity_error);
        const float Iq_ff  = K_vff * target_velocity;
        target_q_current = _constrain(Iq_pid + Iq_ff, -MOTOR_CURRENT_LIMIT, MOTOR_CURRENT_LIMIT);
    }

    // --- Current loop (innermost): target current -> Vd/Vq ---
    currentLoop();
    setPhaseVoltage();
    return true;
}

// Innermost cascade stage — compile-time polymorphic.
//   STM32G4: measured-current PI. Reads Iq/Id from the inline current sense and
//            drives Vq from (target_q_current - Iq) and Vd from (target_d_current
//            - Id). Anti-windup + output clamp live in the PI (voltage-limited).
//   others (Teensy 4.1): no current feedback -> estimate_Vq voltage model, Vd=0.
// The STM32G4 path falls back to the voltage model if no current sense is attached,
// so it always fails safe rather than commanding off a zero current reading.
void FOCController::currentLoop() {
#if defined(STM32G4xx)
    if (current_sense != nullptr) {
        const DQCurrent_s i_dq = current_sense->getFOCCurrents(electrical_angle);
        Id = i_dq.d;
        Iq = i_dq.q;
        Vq = PID_current_q(target_q_current - Iq);
        Vd = PID_current_d(target_d_current - Id);
        return;
    }
#endif
    Vq = _constrain(LPF_voltage(estimate_Vq(target_q_current)), -MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT);
    Vd = 0.0f;

    // Defense in depth: never let a non-finite command reach the PWM. _constrain
    // passes NaN through (NaN compares false), so guard explicitly and re-arm state.
    if (!isfinite(Vq) || !isfinite(Vd)) {
        Vq = 0.0f;
        Vd = 0.0f;
        LPF_voltage.reset();
        faultManager.addFault(isMotorX(motor_number) ? FAULT_FOC_X_NAN_COMMAND : FAULT_FOC_Y_NAN_COMMAND);
    }
}

// Bench helper (L12): drive one inner current-loop step at a commanded Iq/Id,
// bypassing the velocity/angle cascade. Not rate-gated — the PI derives its own dt
// from micros(), so it can run at full superloop speed for the tightest loop.
bool FOCController::currentLoopStep(float iq_target, float id_target) {
    if (!sensor.update()) {
        faultManager.addFault(isMotorX(motor_number) ? FAULT_ENC_X_UPDATE : FAULT_ENC_Y_UPDATE);
        return false;
    }
    mechanical_angle = sensor.getLatestRotorAngle();
    electrical_angle = calculate_electrical_angle();
    angular_velocity = sensor.getFilteredAngularVelocity();
    target_q_current = _constrain(iq_target, -MOTOR_CURRENT_LIMIT, MOTOR_CURRENT_LIMIT);
    target_d_current = _constrain(id_target, -MOTOR_CURRENT_LIMIT, MOTOR_CURRENT_LIMIT);
    currentLoop();
    setPhaseVoltage();
    return true;
}



void FOCController::setPhaseVoltage() {
    // Sinusoidal PWM modulation
    // Inverse Park + Clarke transformation
    fast_sincos(electrical_angle, &_sa, &_ca);

    // Inverse park transform
    Valpha =  _ca * Vd - _sa * Vq;  // -sin(angle) * Uq;
    Vbeta =  _sa * Vd + _ca * Vq;    //  cos(angle) * Uq;

    // Clarke transform
    Va = Valpha;
    Vb = -0.5f * Valpha + _SQRT3_2 * Vbeta;
    Vc = -0.5f * Valpha - _SQRT3_2 * Vbeta;

    center = pwm_driver.getVoltageLimit()/2;

    // discussed here: https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
    // Midpoint Clamp
    float Vmin = min(Va, min(Vb, Vc));
    float Vmax = max(Va, max(Vb, Vc));
    center -= (Vmax+Vmin) / 2;
      
    Va += center;
    Vb += center;
    Vc += center;

    pwm_driver.setPwm(Va, Vb, Vc);
}


float FOCController::calculate_electrical_angle() {
    return normalise_angle(mechanical_angle * motor_pole_pairs * motor_sensor_direction - motor_electrical_offset);
}


float FOCController::estimate_Vq(float _Iq) {
    Vq = _Iq * motor_phase_resistance + (angular_velocity / (motor_K_v * _SQRT3) / _RPM_TO_RADS);
    return Vq;
}


void FOCController::enable() {
    if (enabled) return;
    if (!initialised) {
        faultManager.addFault(FAULT_SYS_NOT_INITIALISED);
        return;
    }
    pwm_driver.enable();
    PID_angle_controller.reset();
    PID_velocity_controller.reset();
    PID_current_q.reset();
    PID_current_d.reset();
    LPF_voltage.reset();
    last_current_us = last_velocity_us = last_angle_us = 0; // run every stage on first loop()
    enabled = true;
    Debug::msg("FOC CONTROLLER ENABLED");
}

void FOCController::disable() {
    if (!enabled) return;
    PID_angle_controller.reset();
    PID_velocity_controller.reset();
    PID_current_q.reset();
    PID_current_d.reset();
    pwm_driver.disable();
    enabled = false;
    Debug::msg("FOC CONTROLLER DISABLED");
}


void FOCController::setControlMode(ControlMode _controlMode) {
    // Reset PID controllers when switching modes to prevent stale state
    if (controlMode != _controlMode) {
        if (_controlMode == VELOCITY) {
            // Switching to velocity mode: reset angle PID since it won't be used
            PID_angle_controller.reset();
        } else if (_controlMode == ANGLE) {
            // Switching to angle mode: reset velocity PID since it will get new values from angle PID
            PID_velocity_controller.reset();
        }
    }
    controlMode = _controlMode;
}

void FOCController::setTargetAngle(float _target_world_angle) { 
    // float target_local_angle = worldToLocal(motor_number, _target_world_angle); 
    // target_angle = target_local_angle; 
    target_angle =  _target_world_angle;
}


void FOCController::deinit() {
    disable();
    sensor.deinit();
    pwm_driver.deinit();
}


// ---------- Calibration ----------
void FOCController::calibrate() {
    if (!initialised) {
        faultManager.addFault(FAULT_SYS_NOT_INITIALISED);
        return;
    }

    // waiting for something to calibrate...
}
