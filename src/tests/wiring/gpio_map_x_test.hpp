#pragma once

#include <Arduino.h>
#include "../../utils/constants.h"
#include "wiring_common.hpp"

// L1 — Motor X: GPIO pin map + EN de-energise check.
// VM must be OFF during the pin-probe sequence.
inline void gpio_map_x_test() {
    Serial.print("TEST gpio_map axis=X target=");
    Serial.println(wiring_target_name());
    Serial.print("PINS N1=");
    Serial.print(N1_X);
    Serial.print(" N2=");
    Serial.print(N2_X);
    Serial.print(" N3=");
    Serial.print(N3_X);
    Serial.print(" EN=");
    Serial.println(EN_X);

    Serial.println("SAFETY VM_OFF required for pin-probe — disconnect 12 V motor supply now.");
    wiring_wait_for_confirm("VM is OFF (12 V motor supply disconnected)");

    Serial.println("GPIO map: probe each HIGH net with a multimeter or LED.");
    wiring_gpio_map_sequence("X", N1_X, N2_X, N3_X, EN_X);

    Serial.println();
    Serial.println("MANUAL rail check (multimeter):");
    Serial.println("  3.3 V logic rail present");
    Serial.println("  DRV8313 VCC present");
    Serial.println("  12 V VM — leave OFF until next step");
    wiring_wait_for_confirm("logic rails measured OK");

    Serial.println();
    Serial.println("EN de-energise check:");
    Serial.println("  All MCU pins are LOW; EN is LOW.");
    Serial.println("  Turn VM ON (12 V). Phase outputs should be ~0 V.");
    Serial.println("  Spin motor X rotor by hand — should be free (no holding torque).");
    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
    wiring_wait_for_confirm("motor X free-spin with EN low and VM on");

    Serial.println("RESULT gpio_map_x PASS");
}
