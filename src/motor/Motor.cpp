#include "Motor.hpp"

Motor::Motor(Sensor& _sensor, PWMDriver& _pwm_driver, int _motor_number) 
: motor_number(_motor_number)
, sensor(_sensor)
, pwm_driver(_pwm_driver)
{};

bool Motor::init() {
    Serial.println("Initialising motor " + String(motor_number) + "...");

    if (!sensor.init()) {
        Serial.println("Failed to initialize sensor");
        return false;
    }
    if (!pwm_driver.init()) {
        Serial.println("Failed to initialize pwm driver");
        return false;
    }

    Serial.print("Using hardcoded sensor offset: ");
    if (motor_number == 1) {
        Serial.println(String(SENSOR_OFFSET_1) + " angle units (motor 1)");
    } else if (motor_number == 2) {
        Serial.println(String(SENSOR_OFFSET_2) + " angle units (motor 2)");
    }
    electrical_offset = motor_number == 1 ? SENSOR_OFFSET_1 : SENSOR_OFFSET_2;

    // if (!findElectricalOffset()) {
    //     Serial.println("Failed to find electrical offset");
    //     return false;
    // }

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
    phase_resistance = PHASE_RESISTANCE;
    K_v = K_V;
    K_e = K_E;
    Serial.println("Motor " + String(motor_number) + " initialised. Starting mechanical angle: " + String(mechanical_angle, 3) + " rad");
    return true;
}

bool Motor::loopFOC() {
    if (!enabled) return false;
    if (!sensor.update()) {
        Serial.println("Failed to update sensor");
        return false;
    }
    mechanical_angle = sensor.getLatestRotorAngle();
    electrical_angle = calculate_electrical_angle();
    angular_velocity = sensor.getAngularVelocity();

    if ( angle_count < ANGLE_TRACKING_LIMIT ) {
        angle_tracking[angle_count] = mechanical_angle;
        angle_count++;
    }

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
    target_q_current = _constrain(target_q_current, -MOTOR_CURRENT_LIMIT, MOTOR_CURRENT_LIMIT);
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

    // Serial.println("Vq: " + String(Vq) + " Vd: " + String(Vd) + "  |   Va: " + String(Va) + " Vb: " + String(Vb) + " Vc: " + String(Vc));
    pwm_driver.setPwm(Va, Vb, Vc);
}


// bool Motor::findMechanicalOffset() {
//     Serial.println("Finding mechanical offset...");
//     unsigned long start_time = millis();
//     int sample_num = 0;
//     int64_t sample = 0;
//     while (millis() - start_time < 2000) {
//         if (!sensor.update()) {
//             return false;
//         }
//         sample += sensor.getLatestRotorAngle();
//         sample_num ++;
//     }
//     mechanical_offset = sample / sample_num;
//     Serial.println("Mechanical offset found: " + String(mechanical_offset) + " units");
//     return true;
// }


// bool Motor::findElectricalOffset() {
//     Serial.println("Finding electrical offset...");

//     float Id = MOTOR_CURRENT_LIMIT * 0.5f;
//     float Vd = Id * PHASE_RESISTANCE;
//     Serial.println("Setting alignment voltage to: " + String(Vd) + " V");
//     setPhaseVoltage(0.0f, Vd, 0.0f); // Vq = 0, Vd = ALIGNMENT_VOLTAGE, angle_el = 0
//     delay(1000);

//     float angle = 0.0f;
//     float sample_offset = 0.0f;
//     int count = 0;
//     unsigned long start_time = millis();
//     while (millis() - start_time < 5000) {
//         sensor.update();
//         angle = sensor.getLatestRotorAngle();
//         sample_offset += angle;
//         count++;
//     }
//     electrical_offset = normalise_angle((sample_offset * POLE_PAIRS * SENSOR_DIRECTION) / (float)count);
//     Serial.println("Electrical offset found: " + String(electrical_offset) + " units");
//     return true;
// }


float Motor::calculate_electrical_angle() {
    return normalise_angle(mechanical_angle * POLE_PAIRS * SENSOR_DIRECTION - electrical_offset);
}


float Motor::estimate_Vq(float _Iq) {
    Vq = _Iq * phase_resistance + (angular_velocity / (K_v * _SQRT3) / _RPM_TO_RADS);
    return Vq;
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


void setTargetAngle(float _target_world_angle) { 
    float target_local_angle = worldToLocal(_target_world_angle); 
    target_angle = target_local_angle; 
}

float worldToLocal(float world_angle) {
    return motor_number == 1 ? world_angle - WORLD_OFFSET_X : world_angle - WORLD_OFFSET_Y;
}