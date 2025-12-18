#include "Sensor.hpp"

bool Sensor::_update() {
    
    timestamp1 = micros();
    float raw_sensor_angle = getSensorAngle();  // takes 5us on teensy 4.1
    if (raw_sensor_angle < 0) {
        return false;
    }
 
    float angle_change = raw_sensor_angle - absolute_rotor_angle;

    // Handle wrapping
    if (abs(angle_change) > _3PI_2) cumulated_rotations += (angle_change > 0) ? -1 : 1;
    absolute_rotor_angle = raw_sensor_angle;

    rotor_angle1 = absolute_rotor_angle + cumulated_rotations * _2PI;
    // rotor_angle1 = sensor_denoising_filter(rotor_angle1);
    // this will obviously break if the rotor turns at more than 270 degrees per update.
    // As I expect to be updating at >10KHz, this won't be a problem.

    time_delta = timestamp1 - timestamp0;
    if (time_delta > VELOCITY_MIN_TIME_INCREMENT) {
        angular_velocity = (rotor_angle1 - rotor_angle0) / ((float)time_delta) * 1e6f;
        angular_velocity = angular_velocity_LPF(angular_velocity);
        rotor_angle0 = rotor_angle1;
        timestamp0 = timestamp1;
    }

    return true;
}


float Sensor::getLatestAbsoluteRotorAngle() {
    return absolute_rotor_angle;
}

float Sensor::getLatestRotorAngle() {
    return rotor_angle1;
}

float Sensor::getAngularVelocity() {
    return angular_velocity;
}

unsigned long Sensor::getLatestTimestamp() {
    return timestamp1;
}

void Sensor::enable() {
    if (enabled) return;
    enabled = true;
}

void Sensor::disable() {
    if (!enabled) return;
    enabled = false;
}