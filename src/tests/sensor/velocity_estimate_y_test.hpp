#pragma once

#include <Arduino.h>
#include "../../sensor/AS5048.hpp"
#include "../wiring/wiring_common.hpp"
#include "sensor_l4_common.hpp"

// L4 — Motor Y: velocity estimate + LPF (hand rotation + wrap probe).
inline void velocity_estimate_y_test() {
    Serial.print("TEST velocity_estimate axis=Y target=");
    Serial.println(wiring_target_name());
    Serial.print("CS=");
    Serial.println(SENSOR_SPI_CS_Y);

    sensor_print_l4_params();

    AS5048 sensor(SENSOR_SPI_CS_Y);
    if (!sensor.init()) {
        Serial.println("RESULT velocity_estimate_y FAIL init=0");
        while (true) {
            delay(1000);
        }
    }
    sensor.enable();
    delay(200);

    Serial.println("REST — keep motor Y rotor still (~2 s).");
    const VelocityStreamStats rest = sensor_stream_velocity_phase(sensor, 2000, L4_WRAP_SPIKE_THRESH_RAD_S);
    sensor_print_velocity_stats("rest", rest);

    Serial.println("CCW — turn motor Y rotor steadily counter-clockwise (~12 s).");
    Serial.println("Expect positive mean_raw_rad_s (SENSOR_DIRECTION=+1, CCW).");
    const VelocityStreamStats ccw = sensor_stream_velocity_phase(sensor, L4_ROTATE_PHASE_MS, L4_WRAP_SPIKE_THRESH_RAD_S);
    sensor_print_velocity_stats("ccw", ccw);
    wiring_wait_for_confirm("CCW rotation: raw_vel sign positive, filt_vel smooth");

    Serial.println("CW — turn motor Y rotor steadily clockwise (~12 s).");
    Serial.println("Expect negative mean_raw_rad_s.");
    const VelocityStreamStats cw = sensor_stream_velocity_phase(sensor, L4_ROTATE_PHASE_MS, L4_WRAP_SPIKE_THRESH_RAD_S);
    sensor_print_velocity_stats("cw", cw);
    wiring_wait_for_confirm("CW rotation: raw_vel sign negative, filt_vel smooth");

    Serial.println("WRAP — spin motor Y quickly through the 0/2pi boundary several times (~12 s).");
    Serial.println("Watch for spurious raw_vel spikes at the wrap (spike_count should stay 0).");
    const VelocityStreamStats wrap = sensor_stream_velocity_phase(sensor, L4_WRAP_PHASE_MS, L4_WRAP_SPIKE_THRESH_RAD_S);
    sensor_print_velocity_stats("wrap", wrap);
    wiring_wait_for_confirm("wrap: no raw_vel spikes at 0/2pi boundary on plot");

    sensor.disable();
    sensor.deinit();

    Serial.print("RESULT velocity_estimate_y PASS max_abs_raw_rad_s=");
    Serial.print(wrap.max_abs_raw, 4);
    Serial.print(" wrap_spikes=");
    Serial.print(wrap.spike_count);
    Serial.print(" ccw_mean_raw=");
    Serial.print((ccw.n > 0) ? (ccw.sum_raw / static_cast<float>(ccw.n)) : 0.0f, 4);
    Serial.print(" cw_mean_raw=");
    Serial.println((cw.n > 0) ? (cw.sum_raw / static_cast<float>(cw.n)) : 0.0f, 4);
}
