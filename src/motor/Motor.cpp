#include "Motor.hpp"

Motor::Motor(Sensor& _sensor, PWMDriver& _pwm_driver) : sensor(_sensor), pwm_driver(_pwm_driver) {};


bool Motor::init() {
    Serial.println("Initialising motor...");

    if (!sensor.init()) {
        Serial.println("Failed to initialize sensor");
        return false;
    }
    if (!pwm_driver.init()) {
        Serial.println("Failed to initialize pwm driver");
        return false;
    }

    if (!find_sensor_offset()) {  // this won't work if the motor is loaded. For now we hardcode loaded motor offsets (4.03rad)
        Serial.println("Failed to find sensor offset");
        return false;
    }
    if (!sensor._update()) {
        Serial.println("Failed to update sensor");
        return false;
    }
    mechanical_angle = sensor.getLatestRotorAngle();
    electrical_angle = calculate_electrical_angle();
    target_angle = mechanical_angle;
    target_velocity = 0.0f;
    target_q_current = 0.0f;
    target_d_current = 0.0f;
    phase_resistance = PHASE_RESISTANCE;
    K_v = K_V;
    K_e = K_E;
    Serial.println("Motor initialised. Starting mechanical angle: " + String(mechanical_angle) + " rad");
    return true;
}


bool Motor::loopFOC() {  // Things to add here: get sensor timestamp, get distance and timestamp from Garmin
    if (!sensor._update()) {
        Serial.println("Failed to update sensor");
        return false;
    }
    mechanical_angle = sensor.getLatestRotorAngle();
    electrical_angle = calculate_electrical_angle();
    angular_velocity = sensor.getAngularVelocity();

    updateFOC();

    setPhaseVoltage();
    return true;
}


void Motor::updateFOC() {
    if (controlMode == ANGLE) {
        angular_error = target_angle - mechanical_angle; // TODO: constrain the target angle
        target_velocity = PID_angle_controller(angular_error);
    }
  
    velocity_error = target_velocity - angular_velocity;
    target_q_current = PID_velocity_controller(velocity_error);
    Vq = estimate_Vq(target_q_current);
    Vq = LPF_voltage(Vq);
    Vq = _constrain(Vq, -MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT);
    Vd = 0.0f;
}


void Motor::setPhaseVoltage() {

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


bool Motor::find_sensor_offset() {
    Serial.println("Finding sensor offset...");
    pwm_driver.setPwm(ALIGNMENT_VOLTAGE, 0.0f, 0.0f); // Vq = ALIGNMENT_VOLTAGE, Vd = 0, angle_el = 0
    delay(1000);
    int sample_num = 0;
    float sample = 0.0f;
    unsigned long sample_time = millis();
    while (millis() - sample_time < 2000) {
        if (!sensor._update()) {
            return false;
        }
        sample += sensor.getLatestAbsoluteRotorAngle();
        sample_num ++;
    }
    sensor_offset = normalise_angle((sample * POLE_PAIRS * SENSOR_DIRECTION) / (float)sample_num);
    Serial.println("Sensor offset found: " + String(sensor_offset) + " rad");
    return true;
}


float Motor::calculate_electrical_angle() {
    return normalise_angle(mechanical_angle * POLE_PAIRS * SENSOR_DIRECTION - sensor_offset);
}


float Motor::estimate_Vq(float _Iq) {
    Vq = _Iq * phase_resistance + (angular_velocity / (K_v * _SQRT3) / _RPM_TO_RADS);
    return Vq;
}


// normalizing radian angle to [0,2PI]
float Motor::normalise_angle(float angle){
    float a = fmod(angle, _2PI);
    return a >= 0 ? a : (a + _2PI);
  }


void Motor::enable() {
    if (enabled) return;
    pwm_driver.enable();
    sensor.enable();
    enabled = true;
}

void Motor::disable() {
    if (!enabled) return;
    PID_angle_controller.reset();
    PID_velocity_controller.reset();
    pwm_driver.disable();
    sensor.disable();
    enabled = false;
}