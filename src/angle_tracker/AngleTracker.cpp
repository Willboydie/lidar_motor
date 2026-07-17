#include "AngleTracker.hpp"

AngleTracker::AngleTracker(Sensor& _sensor, FaultManager& _fault_manager, FaultCode _overflow_fault)
: sensor(_sensor)
, fault_manager(_fault_manager)
, overflow_fault(_overflow_fault)
{}

void AngleTracker::update() {
    if (count < ANGLE_TRACKING_LIMIT) {
        angles[count] = sensor.getLatestRotorAngle();
    }
    count++;
}

void AngleTracker::reset() {
    count = 0;
}

// Estimates the motor angle associated with the latest distance measurement by
// finding the median sample index, averaging a window around it, then resetting.
void AngleTracker::calculateMedianAverage() {
    if (count >= ANGLE_TRACKING_LIMIT) {
        median_average = 0.0f;
        fault_manager.addFault(overflow_fault);
        reset();
        return;
    }

    int median_index = count / 2;
    int effective_window_size = count < ANGLE_TRACKING_WINDOW_SIZE ? count : ANGLE_TRACKING_WINDOW_SIZE;
    median_index = median_index < ANGLE_TRACKING_LIMIT - effective_window_size
        ? median_index
        : ANGLE_TRACKING_LIMIT - effective_window_size;
    float median_average_sum = 0.0f;
    for (int i = 0; i < effective_window_size; i++) {
        median_average_sum += angles[median_index - effective_window_size / 2 + i];
    }
    median_average = median_average_sum / effective_window_size;
    reset();
}
