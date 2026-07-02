#pragma once

#include <Arduino.h>
#include "pwm_common.hpp"

#if defined(STM32G4xx)
#include "../../pwm_driver/stm32g4/stm32_timerutils.h"
#endif

// L6 — PWM synchronisation introspection.
//
// Resolves, per phase pin, which hardware timer resource the PWMDriver actually
// bound the pin to, so the operator can confirm (a) the three phases of one
// motor share one frequency/master and are edge-aligned, and (b) the two motors
// use *disjoint* timer resources (no cross-axis contention).
//
// The introspection is target-specific:
//  - STM32G4: SimpleFOC allocates a TIM per channel; timer_id = TIM number,
//    arr/psc are the period registers, master = stm32_alignTimers() result.
//  - Teensy 4.1: FlexPWM submodules synced over XBAR; timer_id = module*10 +
//    submodule, master = the internally-synced submodule (phase C).

struct PwmSyncInfo {
    int timer_id[3];     // STM32: TIM number; Teensy: module*10 + submodule
    int channel[3];      // STM32: 1..4 (TIM channel); Teensy: 0=X,1=A,2=B
    uint32_t arr[3];     // STM32 period reg (0 on Teensy)
    uint32_t psc[3];     // STM32 prescaler   (0 on Teensy)
    int distinct_count;  // distinct timer resources this motor occupies
    int master_id;       // alignment master (timer_id) or -1 if none
    bool master_found;
    bool valid;          // false if the driver could not bind a pin
};

#if defined(STM32G4xx)

inline void pwm_sync_resolve(PlatformPWMDriver& driver, const int pins[3], PwmSyncInfo& info) {
    (void)pins;
    info.valid = true;
    STM32DriverParams* p = driver.getDriverParams();
    if (p == nullptr) {
        info.valid = false;
        for (int i = 0; i < 3; ++i) {
            info.timer_id[i] = -1;
            info.channel[i] = -1;
            info.arr[i] = 0;
            info.psc[i] = 0;
        }
        info.distinct_count = 0;
        info.master_found = false;
        info.master_id = -1;
        return;
    }
    for (int i = 0; i < 3; ++i) {
        TIM_HandleTypeDef* h = p->timers_handle[i];
        if (h == nullptr) {
            info.timer_id[i] = -1;
            info.channel[i] = -1;
            info.arr[i] = 0;
            info.psc[i] = 0;
            info.valid = false;
            continue;
        }
        info.timer_id[i] = stm32_getTimerNumber(h->Instance);
        info.channel[i] = static_cast<int>(p->channels[i] >> 2) + 1;  // TIM_CHANNEL_x -> 1..4
        info.arr[i] = h->Instance->ARR;
        info.psc[i] = h->Instance->PSC;
    }
    info.distinct_count = p->num_timers;
    info.master_found = (p->master_timer != nullptr);
    info.master_id = info.master_found ? stm32_getTimerNumber(p->master_timer->Instance) : -1;
}

#elif defined(ARDUINO_TEENSY41)

inline void pwm_sync_resolve(PlatformPWMDriver& driver, const int pins[3], PwmSyncInfo& info) {
    (void)driver;
    info.valid = true;
    for (int i = 0; i < 3; ++i) {
        const int pin = pins[i];
        const pwm_pin_info_struct& pi = pwm_pin_info[pin];
        if (pi.type != 1) {  // not a FlexPWM pin -> driver cannot centre-align it
            info.timer_id[i] = -1;
            info.channel[i] = -1;
            info.valid = false;
        } else {
            const int module = ((pi.module >> 4) & 3) + 1;
            const int submodule = pi.module & 0x3;
            info.timer_id[i] = module * 10 + submodule;
            info.channel[i] = pi.channel;
            if (pi.channel == 0) {  // X channel: driver rejects (needs A/B for centre-aligned)
                info.valid = false;
            }
        }
        info.arr[i] = 0;
        info.psc[i] = 0;
    }
    int distinct = 0;
    for (int i = 0; i < 3; ++i) {
        bool seen = false;
        for (int j = 0; j < i; ++j) {
            if (info.timer_id[j] == info.timer_id[i]) seen = true;
        }
        if (!seen) ++distinct;
    }
    info.distinct_count = distinct;
    // Phase C submodule is configured as the internally-synced XBAR master.
    info.master_found = info.valid;
    info.master_id = info.valid ? info.timer_id[2] : -1;
}

#endif

inline bool pwm_sync_arr_aligned(const PwmSyncInfo& info) {
    for (int i = 1; i < 3; ++i) {
        if (info.arr[i] != info.arr[0] || info.psc[i] != info.psc[0]) return false;
    }
    return true;
}

inline void pwm_sync_print_axis(const char* axis, const int pins[3], const PwmSyncInfo& info) {
    const char* names[3] = {"N1", "N2", "N3"};
    for (int i = 0; i < 3; ++i) {
        Serial.print("SYNC axis=");
        Serial.print(axis);
        Serial.print(" pin=");
        Serial.print(names[i]);
        Serial.print(" gpio=");
        Serial.print(pins[i]);
        Serial.print(" timer=");
        Serial.print(info.timer_id[i]);
        Serial.print(" chan=");
        Serial.print(info.channel[i]);
#if defined(STM32G4xx)
        Serial.print(" arr=");
        Serial.print(info.arr[i]);
        Serial.print(" psc=");
        Serial.print(info.psc[i]);
#endif
        Serial.println();
    }

    Serial.print("SYNC axis=");
    Serial.print(axis);
    Serial.print(" distinct_timers=");
    Serial.print(info.distinct_count);
    Serial.print(" master=");
    if (info.master_found) {
        Serial.print(info.master_id);
    } else {
        Serial.print("none");
    }
#if defined(STM32G4xx)
    Serial.print(" arr_aligned=");
    Serial.print(pwm_sync_arr_aligned(info) ? "yes" : "no");
#endif
    Serial.println();

    // Auto-assessment against the L6 per-motor exit bar: the three phases must
    // resolve to a single frequency/master. A single shared timer is inherently
    // aligned (no master needed); multiple timers require an alignment master.
    const bool freq_ok =
#if defined(STM32G4xx)
        pwm_sync_arr_aligned(info);
#else
        true;  // Teensy submodules driven from one frequency setting
#endif
    const bool sync_ok = info.valid && freq_ok &&
                         (info.distinct_count == 1 || info.master_found);
    Serial.print("ASSESS axis=");
    Serial.print(axis);
    Serial.print(" per_motor_sync=");
    Serial.println(sync_ok ? "OK" : "CHECK");
}

inline int pwm_sync_count_shared(const PwmSyncInfo& a, const PwmSyncInfo& b) {
    int shared = 0;
    int counted[3] = {-1, -1, -1};
    int n = 0;
    for (int i = 0; i < 3; ++i) {
        if (a.timer_id[i] < 0) continue;
        bool in_b = false;
        for (int j = 0; j < 3; ++j) {
            if (b.timer_id[j] == a.timer_id[i]) in_b = true;
        }
        if (!in_b) continue;
        bool already = false;
        for (int k = 0; k < n; ++k) {
            if (counted[k] == a.timer_id[i]) already = true;
        }
        if (!already) {
            counted[n++] = a.timer_id[i];
            ++shared;
        }
    }
    return shared;
}

// A genuine cross-axis fault: two phases driving the *same channel* of the same
// timer/submodule (they would fight over one duty register). Two phases on the
// same counter but different channels (e.g. Teensy FlexPWM2 sm2 chA + chB) is
// benign — independent duties, inherently phase-locked.
inline int pwm_sync_count_channel_conflicts(const PwmSyncInfo& a, const PwmSyncInfo& b) {
    int conflicts = 0;
    for (int i = 0; i < 3; ++i) {
        if (a.timer_id[i] < 0) continue;
        for (int j = 0; j < 3; ++j) {
            if (b.timer_id[j] == a.timer_id[i] && b.channel[j] == a.channel[i]) {
                ++conflicts;
            }
        }
    }
    return conflicts;
}

inline int pwm_sync_total_distinct(const PwmSyncInfo& a, const PwmSyncInfo& b) {
    int ids[6];
    int n = 0;
    const PwmSyncInfo* infos[2] = {&a, &b};
    for (int s = 0; s < 2; ++s) {
        for (int i = 0; i < 3; ++i) {
            const int id = infos[s]->timer_id[i];
            if (id < 0) continue;
            bool seen = false;
            for (int k = 0; k < n; ++k) {
                if (ids[k] == id) seen = true;
            }
            if (!seen) ids[n++] = id;
        }
    }
    return n;
}

inline void pwm_sync_bench_prompt() {
    Serial.println();
    Serial.println("SAFETY — L6 PWM synchronisation:");
    Serial.println("  VM ON (12 V). Bench supply current limit ~1.0 A.");
    Serial.println("  Multi-channel scope: probe the three phases of a motor together");
    Serial.println("  for edge alignment; for the dual step probe one phase per axis.");
    Serial.println("  Other axis driver EN must stay LOW during per-axis steps.");
    Serial.println("  E-stop: cut VM or pull EN low if anything looks wrong.");
    pwm_print_spec();
    wiring_wait_for_confirm("VM on, multi-channel scope ready, supply current-limited");
}

inline void pwm_sync_hold(PlatformPWMDriver& driver, const char* axis) {
    pwm_set_duties(driver, L5_MID_DUTY, L5_MID_DUTY, L5_MID_DUTY);
    Serial.print("SYNC axis=");
    Serial.print(axis);
    Serial.print(" mid_duty=");
    Serial.print(L5_MID_DUTY, 2);
    Serial.print(" hold_ms=");
    Serial.println(L5_CENTER_HOLD_MS);
    Serial.println("SCOPE — probe N1/N2/N3 together: rising edges coincide, no phase drift.");
    delay(L5_CENTER_HOLD_MS);
    wiring_wait_for_confirm("scope: three phases frequency- and edge-aligned");
}

inline void pwm_sync_hold_dual(PlatformPWMDriver& driverX, PlatformPWMDriver& driverY) {
    pwm_set_duties(driverX, L5_MID_DUTY, L5_MID_DUTY, L5_MID_DUTY);
    pwm_set_duties(driverY, L5_MID_DUTY, L5_MID_DUTY, L5_MID_DUTY);
    Serial.print("SYNC axis=dual mid_duty=");
    Serial.print(L5_MID_DUTY, 2);
    Serial.print(" hold_ms=");
    Serial.println(L5_CENTER_HOLD_MS);
    Serial.println("SCOPE — probe one phase from X and one from Y: stable, no beat/cross-talk.");
    delay(L5_CENTER_HOLD_MS);
    wiring_wait_for_confirm("scope: both axes stable, no beat/interference");
}
