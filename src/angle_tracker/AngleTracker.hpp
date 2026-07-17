#pragma once

#include "../sensor/Sensor.hpp"
#include "../utils/constants.h"
#include "../fault_manager/FaultManager.hpp"

class AngleTracker {

public:
    AngleTracker(Sensor& _sensor, FaultManager& _fault_manager, FaultCode _overflow_fault);
    ~AngleTracker() = default;

    void update();
    void calculateMedianAverage();
    float getAssociatedAngle() const { return median_average; }

private:
    void reset();

    Sensor& sensor;
    FaultManager& fault_manager;
    FaultCode overflow_fault;

    float median_average = 0.0f;
    int count = 0;
    float angles[ANGLE_TRACKING_LIMIT];
};
