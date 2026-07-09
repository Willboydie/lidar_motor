#include "AngleTracker.hpp"

AngleTracker::AngleTracker(Sensor& _sensor) : sensor(_sensor) {}

void AngleTracker::update() {
    
    if ( count < limit ) {
        angles[count] = sensor.getLatestRotorAngle();
    }
    count++;
}

void AngleTracker::reset() {
    count = 0;
}

// This function estimates the angle of the motor associated with the latest distance measurement.
// It does so by finding the median angle over the period the distance is being measured, defining a window around it and taking the average.
void AngleTracker::calculateMedianAverage() {
    if ( count > limit ) {
        median_average = 0.0f;
        reset();
    }

    int median_index = count / 2;
    int window_size = count < window_size ? count : window_size;
    median_index = median_index < limit - window_size ? median_index : limit - window_size;
    float median_average_sum = 0.0f;
    for ( int i = 0; i < window_size; i++ ) {
        median_average_sum += angles[median_index - window_size/2 + i];
    }
    median_average = median_average_sum / window_size;
    reset();
}