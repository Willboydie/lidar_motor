#include "Sensor.hpp"

bool Sensor::update() {
    if (!initialised) {
        Serial.println("Sensor not initialised");
        return false;
    }
    timestamp1 = micros();
    uint16_t raw_sensor_angle = getRawCount();  // takes 5us on teensy 4.1
 
    if (!first_measurement) {
        int32_t angle_change = raw_sensor_angle - absolute_rotor_angle;
        // Handle wrapping
        if (abs(angle_change) > (3 * CPR / 4)) cumulated_rotations += (angle_change > 0) ? -1 : 1;
    } else {
        first_measurement = false;
        cumulated_rotations = 0;
    }

    absolute_rotor_angle = raw_sensor_angle;

    rotor_angle1 = absolute_rotor_angle + cumulated_rotations * CPR; // multiply by (1 << 14) to get the raw integer value
    // this will obviously break if the rotor turns at more than 270 degrees per update.
    // As I expect to be updating at >10KHz, this won't be a problem.

    unsigned long time_delta = timestamp1 - timestamp0;
    if (time_delta > VELOCITY_MIN_TIME_INCREMENT) {
        angular_velocity = (rotor_angle1 - rotor_angle0) * 1e6 / time_delta;
        angular_velocity = angular_velocity_LPF(angular_velocity);
        rotor_angle0 = rotor_angle1;
        timestamp0 = timestamp1;
    }

    return true;
}



float Sensor::getLatestAbsoluteRotorAngle() {
    return toRadians(absolute_rotor_angle);
}

float Sensor::getLatestRotorAngle() {
    return toRadians(rotor_angle1);
}

float Sensor::getAngularVelocity() {
    return toRadians(angular_velocity);
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