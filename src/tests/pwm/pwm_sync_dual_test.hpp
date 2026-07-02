#pragma once

#include <Arduino.h>
#include "pwm_sync_common.hpp"

// L6 — Dual: both motors up at once; assert disjoint timer resources, no beat.
inline void pwm_sync_dual_test() {
    Serial.print("TEST pwm_sync axis=dual target=");
    Serial.println(wiring_target_name());
    Serial.print("PINS X: N1=");
    Serial.print(N1_X);
    Serial.print(" N2=");
    Serial.print(N2_X);
    Serial.print(" N3=");
    Serial.println(N3_X);
    Serial.print("PINS Y: N1=");
    Serial.print(N1_Y);
    Serial.print(" N2=");
    Serial.print(N2_Y);
    Serial.print(" N3=");
    Serial.println(N3_Y);

    const int pinsX[3] = {N1_X, N2_X, N3_X};
    const int pinsY[3] = {N1_Y, N2_Y, N3_Y};

#if defined(ARDUINO_TEENSY41)
    PWMDriverTeensy41::clearRegisteredSubmodules();
#endif

    PlatformPWMDriver driverX(N1_X, N2_X, N3_X, EN_X);
    PlatformPWMDriver driverY(N1_Y, N2_Y, N3_Y, EN_Y);

    if (!driverX.init()) {
        Serial.println("RESULT pwm_sync_dual FAIL driverX_init=0");
        while (true) {
            delay(1000);
        }
    }
    if (!driverY.init()) {
        Serial.println("RESULT pwm_sync_dual FAIL driverY_init=0");
        while (true) {
            delay(1000);
        }
    }

#if defined(ARDUINO_TEENSY41)
    PWMDriverTeensy41::alignAllRegisteredSubmodules();
    Serial.println("SYNC dual teensy_submodule_align=done");
#endif

    PwmSyncInfo infoX;
    PwmSyncInfo infoY;
    pwm_sync_resolve(driverX, pinsX, infoX);
    pwm_sync_resolve(driverY, pinsY, infoY);
    pwm_sync_print_axis("X", pinsX, infoX);
    pwm_sync_print_axis("Y", pinsY, infoY);

    const int shared = pwm_sync_count_shared(infoX, infoY);
    const int total_distinct = pwm_sync_total_distinct(infoX, infoY);
    const int channel_conflicts = pwm_sync_count_channel_conflicts(infoX, infoY);
    // A shared counter with distinct channels is acceptable (independent duty,
    // guaranteed phase-lock). The hard fault is two phases on the same channel.
    const bool independence_ok = (channel_conflicts == 0);
    Serial.print("SYNC dual total_distinct_timers=");
    Serial.print(total_distinct);
    Serial.print(" shared_counters=");
    Serial.print(shared);
    Serial.print(" channel_conflicts=");
    Serial.println(channel_conflicts);
    Serial.print("ASSESS dual timer_independence=");
    Serial.println(independence_ok ? "OK" : "FAIL");

    driverX.enable();
    driverY.enable();
    pwm_sync_hold_dual(driverX, driverY);

    driverX.disable();
    driverY.disable();
    driverX.deinit();
    driverY.deinit();
    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    Serial.print("RESULT pwm_sync_dual ");
    Serial.print(independence_ok ? "PASS" : "FAIL");
    Serial.print(" channel_conflicts=");
    Serial.print(channel_conflicts);
    Serial.print(" shared_counters=");
    Serial.print(shared);
    Serial.print(" total_distinct_timers=");
    Serial.println(total_distinct);
}
