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

bool FOCController::init() {
    Serial.println("Initialising FOC controller for motor " + String(motor_number) + "...");

    if (!sensor.init()) {
        Serial.println("Failed to initialize sensor");
        return false;
    }
    if (!pwm_driver.init()) {
        Serial.println("Failed to initialize pwm driver");
        return false;
    }

    if (!sensor.update()) {
        Serial.println("Failed to update sensor");
        return false;
    }

    mechanical_angle = sensor.getLatestRotorAngle();
    electrical_angle = calculate_electrical_angle();
    target_angle = mechanical_angle;
    target_velocity = 0.0f;
    target_q_current = 0.0f;
    target_d_current = 0.0f;
    Serial.println("FOC controller for motor " + String(motor_number) + " initialised. Starting mechanical angle: " + String(mechanical_angle, 3) + " rad");
    initialised = true;
    return true;
}

bool FOCController::loop() {
    if (!enabled) return false;
    if (!initialised) {
        Serial.println("Can't loop FOC controller - not initialised");
        return false;
    }

    if (!sensor.update()) {
    Serial.println("Failed to update sensor");
    return false;
    }
    mechanical_angle = sensor.getLatestRotorAngle();
    electrical_angle = calculate_electrical_angle();
    angular_velocity = sensor.getFilteredAngularVelocity();

    if (angle_tracking_count < ANGLE_TRACKING_LIMIT) {
        angle_tracking[angle_tracking_count++] = mechanical_angle;
    }

    if (controlMode == ANGLE) {
        // angular_error = target_angle - mechanical_angle;
        angular_error = findAngularError(target_angle, mechanical_angle);
        target_velocity = PID_angle_controller(angular_error);
    }

    velocity_error = target_velocity - angular_velocity;

    float Iq_pid = PID_velocity_controller(velocity_error);
    float Iq_ff  = K_vff * target_velocity;

    target_q_current = Iq_pid + Iq_ff;
    target_q_current = _constrain(
        target_q_current,
        -MOTOR_CURRENT_LIMIT,
        MOTOR_CURRENT_LIMIT
    );

    
    Vq = estimate_Vq(target_q_current);
    Vq = LPF_voltage(Vq);
    Vq = _constrain(Vq, -MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT);
    setPhaseVoltage(); // Vd = 0.0f
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

    // Serial.println("Vq: " + String(Vq) + " Vd: " + String(Vd) + "  |   Va: " + String(Va) + " Vb: " + String(Vb) + " Vc: " + String(Vc));
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
    pwm_driver.enable();
    sensor.enable();
    PID_angle_controller.reset();
    PID_velocity_controller.reset();
    enabled = true;
}

void FOCController::disable() {
    if (!enabled) return;
    PID_angle_controller.reset();
    PID_velocity_controller.reset();
    pwm_driver.disable();
    sensor.disable();
    enabled = false;
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
        Serial.println("Can't calibrate FOC controller - not initialised");
        return;
    }

    // waiting for something to calibrate...
}