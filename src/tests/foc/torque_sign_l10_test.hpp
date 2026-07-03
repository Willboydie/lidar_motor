#pragma once

#include <Arduino.h>
#include "torque_sign_x_test.hpp"
#include "torque_sign_y_test.hpp"

// L10 — Closed-loop commutation sign / torque test (motor X then Y).
// Invoked from setup(); main loop() stays empty.
//
// Each axis commutates off the LIVE encoder at a fixed Vq (PIDs bypassed) and is
// operator-driven — starts IDLE, then 'g' spins continuously (rotation is EXPECTED
// on a free rotor, not a fault) and 'p' fires a bounded torque pulse. Operator
// confirms per axis: continuous +Vq spins one consistent direction, 's' reverses
// it, a +Vq pulse kicks one way and the flipped pulse the other, and PHASE current
// stays under MOTOR_CURRENT_LIMIT (bench-supply bus current reads lower). '+'/'-'
// dial Vq; ' '/'x' = manual e-stop; 'q' advances to the next axis.
inline void torque_sign_l10_test() {
    wiring_serial_begin();

    Serial.print("TEST torque_sign_l10 target=");
    Serial.println(wiring_target_name());
    Serial.println("L10 — commutate off the real encoder at fixed Vq; prove torque sign + hold.");
    Serial.println("Depends on L9 offsets/direction stored in constants.h.");

    Serial.println();
    Serial.println("=== motor X: closed-loop torque sign ===");
    torque_sign_x_test();

    Serial.println();
    Serial.println("--- motor X complete; starting motor Y ---");
    Serial.println();

    Serial.println("=== motor Y: closed-loop torque sign ===");
    torque_sign_y_test();

    Serial.println();
    Serial.print("RESULT torque_sign_l10 DONE target=");
    Serial.println(wiring_target_name());
    Serial.println("L10 complete — all drivers disabled. Confirm torque sign/hold per axis before L11.");

    wiring_pins_all_low(N1_X, N2_X, N3_X, EN_X);
    wiring_pins_all_low(N1_Y, N2_Y, N3_Y, EN_Y);

    while (true) {
        delay(1000);
    }
}
