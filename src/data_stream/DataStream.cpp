#include "DataStream.hpp"

DataStream::DataStream(Motor& _motor1, Motor& _motor2, LDM& _ldm) : motor1(_motor1), motor2(_motor2), ldm(_ldm) {};


// This function estimates the angle of the motor associated with the latest distance measurement.
// It does so by finding the median angle over the period the distance is being measured, defining a window around it and taking the average.
float DataStream::calculateAssociatedAngle(Motor& motor) {
    if ( motor.angle_count >= ANGLE_TRACKING_LIMIT ) return 0.0f;
    int associated_index = motor.angle_count / 2;
    int window_size = motor.angle_count < ANGLE_TRACKING_WINDOW_SIZE ? motor.angle_count : ANGLE_TRACKING_WINDOW_SIZE;
    associated_index = associated_index < ANGLE_TRACKING_LIMIT - window_size ? associated_index : ANGLE_TRACKING_LIMIT - window_size;
    float associated_angle = 0.0f;
    for ( int i = 0; i < window_size; i++ ) {
        associated_angle += motor.angle_tracking[associated_index - window_size/2 + i];
    }
    associated_angle /= window_size;
    motor.resetAngleTracking();
    return associated_angle;
}

void DataStream::sendData() {
    associated_angle1 = calculateAssociatedAngle(motor1);
    associated_angle2 = calculateAssociatedAngle(motor2);
    // Serial.println("Distance: " + String(ldm.getLatestDistance()) + "  |  Associated Angle: " + String(associated_angle, 3) + "  |  Index: " + String(associated_index));
}