#pragma once

#include <Arduino.h>
#include "../../utils/constants.h"
#include "../../motor/GM2804.hpp"
#include "../../pwm_driver/PlatformPWMDriver.hpp"
#include "../wiring/wiring_common.hpp"

// L2 — one phase leg high, others at 0 V. At 12 V VPS and 9 Ω line-line this is ~0.33 A.
static constexpr float L2_ENERGIZE_VOLTAGE = ALIGNMENT_VOLTAGE;
static constexpr uint32_t L2_PAIR_DWELL_MS = 1000;
static constexpr float L2_RESISTANCE_NOMINAL_OHM = GM2804_PHASE_RESISTANCE;
static constexpr float L2_RESISTANCE_TOLERANCE = 0.15f;

struct PhasePairStep {
    const char* name;
    float va;
    float vb;
    float vc;
};

inline void windings_serial_begin() {
    wiring_serial_begin();
}

inline void windings_print_resistance_spec() {
    const float lo = L2_RESISTANCE_NOMINAL_OHM * (1.0f - L2_RESISTANCE_TOLERANCE);
    const float hi = L2_RESISTANCE_NOMINAL_OHM * (1.0f + L2_RESISTANCE_TOLERANCE);
    Serial.print("SPEC phase_resistance_nominal=");
    Serial.print(L2_RESISTANCE_NOMINAL_OHM, 1);
    Serial.print(" ohm acceptable=");
    Serial.print(lo, 2);
    Serial.print("..");
    Serial.print(hi, 2);
    Serial.println(" ohm (line-line, motor unplugged)");
}

inline void windings_ohmmeter_prompt(const char* axis) {
    Serial.println();
    Serial.println("OHMMETER — motor leads disconnected from DRV8313:");
    Serial.println("  Measure A-B, B-C, C-A line-line resistance.");
    Serial.println("  Confirm no winding short to motor case.");
    windings_print_resistance_spec();
    wiring_wait_for_confirm("ohmmeter: three balanced resistances, no case short");
    Serial.print("OHMMETER axis=");
    Serial.print(axis);
    Serial.println(" operator_confirmed");
}

inline void windings_supply_prompt() {
    Serial.println();
    Serial.println("SAFETY — before energising:");
    Serial.println("  Reconnect motor to driver.");
    Serial.println("  VM ON (12 V). Bench supply current limit <= 0.4 A.");
    Serial.println("  Rotor free to move; other axis driver EN must stay LOW.");
    wiring_wait_for_confirm("motor connected, VM on, supply limited to <= 0.4 A");
}

inline void windings_energize_pair(
    PlatformPWMDriver& driver,
    const char* axis,
    const PhasePairStep& pair) {

    driver.setPwm(pair.va, pair.vb, pair.vc);
    Serial.print("PAIR axis=");
    Serial.print(axis);
    Serial.print(" name=");
    Serial.print(pair.name);
    Serial.print(" Va=");
    Serial.print(pair.va, 2);
    Serial.print(" Vb=");
    Serial.print(pair.vb, 2);
    Serial.print(" Vc=");
    Serial.print(pair.vc, 2);
    Serial.print(" V dwell_ms=");
    Serial.println(L2_PAIR_DWELL_MS);
    delay(L2_PAIR_DWELL_MS);

    Serial.println("Feel rotor: distinct stable holding detent expected.");
    Serial.println("Watch supply current stays <= 0.4 A.");
    wiring_wait_for_confirm("stable detent felt on this pair");
}
