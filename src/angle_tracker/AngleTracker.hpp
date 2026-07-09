#pragma once

#include "../sensor/Sensor.hpp"
#include "../utils/tweakable_params.h"
#include "../utils/quick_defs.h"

class LidarEngine {

public:
    AngleTracker(Sensor& _sensor);
    ~AngleTracker() = default;

    void update_angle_tracking();
    float calculate_associated_angle();

private:
    Sensor& sensor;

    float median_average = 0.0f;

    float count = 0;
    constexpr float window_size = ANGLE_TRACKING_WINDOW_SIZE;
    constexpr float limit = ANGLE_TRACKING_LIMIT;
    float angles[limit];

};