#include "Sensor.hpp"

// constexpr int32_t _3adapted_cpr_4 = (3 * ADAPTED_CPR) / 4;
// constexpr int32_t adapted_cpr = ADAPTED_CPR;
// constexpr unsigned int N = VELOCITY_ESTIMATE_N;

constexpr float _3pi_over_4 = 3.0f * _PI_4;
constexpr float _2pi = _2PI;
constexpr float dt = static_cast<float>(SENSOR_UPDATE_DT_US) * 1e-6f;

constexpr uint32_t velocity_estimation_dt_us = VELOCITY_ESTIMATION_DT_US;

Sensor::Sensor() {}

bool Sensor::update() {
    if (!initialised) {
        Serial.println("Sensor not initialised");
        return false;
    }
    
    raw_sensor_angle = getSensorAngle();
    // updateDiagnostics();
    // if (cof == 1) {
    //     raw_sensor_angle = previous_sensor_angle;
    //     return true;
    // }
    uint32_t t1 = micros();
    // rotor_angle = angleLPF(rotor_angle);

    // velocity estimation
    if (t0 > t1) {
        t0 = micros();
        return true;
    }
    uint32_t dt_us = t1 - t0;
    if (dt_us >= velocity_estimation_dt_us) {
        float dt = static_cast<float>(dt_us) * 1e-6f;
        float change = findAngularError(raw_sensor_angle, previous_raw_angle);
        raw_angular_velocity = change / dt;
        filtered_angular_velocity = velocityLPF(raw_angular_velocity);
        previous_raw_angle = raw_sensor_angle;
        t0 = t1;
    }

    return true;
}


void Sensor::enable() {
    if (enabled) return;
    enabled = true;
    t0 = micros();
    previous_raw_angle = getSensorAngle();
    raw_angular_velocity = 0.0f;
    filtered_angular_velocity = velocityLPF(0.0f);
}

void Sensor::disable() {
    if (!enabled) return;
    enabled = false;
}

void Sensor::clearDebugData() {
    memset(debug_data, 0, sizeof(debug_data));
    debug_idx = 0;
}