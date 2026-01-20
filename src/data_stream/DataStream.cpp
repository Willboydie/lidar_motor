#include "DataStream.hpp"

#define SEND_DATA_CMD 0b01010101

DataStream::DataStream(FOCController& _focController1, FOCController& _focController2, LDM& _ldm, ScanPath& _scanPath) 
: focController1(_focController1)
, focController2(_focController2)
, ldm(_ldm)
, scanPath(_scanPath) 
{};

// This function estimates the angle of the motor associated with the latest distance measurement.
// It does so by finding the median angle over the period the distance is being measured, defining a window around it and taking the average.
float DataStream::calculateAssociatedAngle(FOCController& focController) {
    if ( focController.angle_tracking_count >= ANGLE_TRACKING_LIMIT ) {
        focController.resetAngleTracking();
        return 0.0f;
    }
    int associated_index = focController.angle_tracking_count / 2;
    int window_size = focController.angle_tracking_count < ANGLE_TRACKING_WINDOW_SIZE ? focController.angle_tracking_count : ANGLE_TRACKING_WINDOW_SIZE;
    associated_index = associated_index < ANGLE_TRACKING_LIMIT - window_size ? associated_index : ANGLE_TRACKING_LIMIT - window_size;
    float associated_angle = 0.0f;
    for ( int i = 0; i < window_size; i++ ) {
        associated_angle += focController.angle_tracking[associated_index - window_size/2 + i];
    }
    associated_angle /= window_size;
    focController.resetAngleTracking();
    return associated_angle;
}

void DataStream::sendData() {
    float associated_angle1 = worldToLocal(1, calculateAssociatedAngle(focController1));
    float associated_angle2 = worldToLocal(2, calculateAssociatedAngle(focController2));
    float target_theta = scanPath.theta;
    float target_phi = scanPath.phi;

    DataPacket packet = {
        SEND_DATA_CMD,
        ldm.getLatestDistance(),
        associated_angle1,
        associated_angle2,
        target_theta,
        target_phi
    };
    
    Serial.write((uint8_t*)&packet, sizeof(packet));
}

