# Motor Findings & Electrical-Offset Calibration

Findings from debugging the FOC commutation on the two gimbal motors (X and Y:
GM2804-class, **7 pole pairs**, AS5048A magnetic encoder, `SENSOR_DIRECTION = 1`),
and the procedure to calibrate their electrical offsets reliably.

## TL;DR

- The original symptom — *"the estimated magnetic field direction disagrees with the
  true field"* — was **not** a code bug in the electrical-angle math. It was caused by
  **stale/incorrect `SENSOR_OFFSET_X/Y` values** in `constants.h` (off by ~2.66 rad).
- These motors are **friction-dominated when unloaded**: static friction (stiction)
  exceeds the open-loop torque available at safe drive levels, so the rotor **cannot be
  held in synchronous lock** by a rotating field. Any naive open-loop calibration is
  corrupted by this.
- The alignment of an unloaded rotor under a static d-axis field is not exactly at the
  commanded angle; it carries a **deterministic 2nd-harmonic error** (period 180°
  electrical, ±0.5 rad) from rotor saliency/cogging, plus a friction dead-band.
- The robust calibration is a **stiction-bracket + circular-mean** method, implemented
  in `src/tests/sensor/electrical_offset_autocal_test.hpp`. It is autonomous (no human
  hand) and repeatable to within the irreducible saliency band.

## The bug we were chasing

The FOC controller computes the rotor electrical angle as:

```cpp
// src/foc_controller/FOCController.cpp
float FOCController::calculate_electrical_angle() {
    return normalise_angle(mechanical_angle * motor_pole_pairs * motor_sensor_direction
                           - motor_electrical_offset);
}
```

This formula is correct. The disagreement between estimated and true field was entirely
explained by `motor_electrical_offset` (`SENSOR_OFFSET_X/Y`) being wrong:

| Motor | Old stored | Measured (autocal) | Error |
|-------|-----------:|-------------------:|------:|
| X     | 1.4240 rad | 4.0852 rad         | +2.66 rad |
| Y     | 6.1619 rad | 2.5624 rad         | +2.68 rad |

After updating the offsets, a re-run of the autocal reported `change vs stored offset`
of **−0.0013 rad (X)** and **−0.0342 rad (Y)** — i.e. the stored values now match the
physically-measured alignment, confirming the fix.

## What we learned about these motors

### 1. Stiction exceeds open-loop torque (unloaded)

Driving a continuously rotating field open-loop does **not** spin the rotor
synchronously. The breakaway friction dead-band is **wider than π electrical**, so the
field slips past the rotor and ratchets it in one fixed mechanical direction *regardless
of the commanded direction*. Consequence: methods that rely on tracking a moving field
(e.g. "point of highest velocity = electrical zero") fail — both motors just creep one
way.

### 2. The alignment error is a deterministic 2nd harmonic

When a static d-axis field is applied at electrical angle `theta`, the rotor settles at
`theta + e(theta)`, where `e(theta)` is **repeatable** and dominated by a **2nd
electrical harmonic** (period 180°). Measured implied-offset vs electrical phase
(MOTOR X shown; MOTOR Y identical in shape):

| phase (elec) | 0° | 60° | 120° | 180° | 240° | 300° |
|---|---|---|---|---|---|---|
| implied offset (rad) | 4.55 | 3.50 | 4.19 | 4.55 | 3.50 | 4.19 |

The values repeat every 180°, the signature of reluctance/cogging saliency
(torque ∝ sin(2·load_angle)). This ±0.5 rad wobble is **inherent** and cannot be removed
by any single scalar offset — but it has zero mean over a full electrical period, so it
**averages out**.

### 3. Friction adds a symmetric dead-band

Because the rotor sticks before reaching true alignment, a single-sided approach is
biased by the friction dead-band. Approaching the same target from **both** sides and
taking the midpoint cancels this (symmetric) bias.

## The calibration method (stiction-bracket)

Implemented in `src/tests/sensor/electrical_offset_autocal_test.hpp`. For each target
electrical angle `theta`:

1. **Approach from below** — pull the rotor under `theta`, creep up slowly, let it settle.
   It sticks just *below* true alignment → `mech_lo`.
2. **Approach from above** — pull the rotor over `theta`, creep down slowly, let it settle.
   It sticks just *above* true alignment → `mech_hi`.
3. **Midpoint** `mech_mid = circular_mean(mech_lo, mech_hi)` cancels the symmetric stiction
   → true aligned mechanical angle at `theta`.
4. **Implied offset** `= wrap(mech_mid · pole_pairs · sensor_direction − theta)`.

Targets are spread across a **full mechanical revolution** (several per pole pair) and the
implied offsets are **circular-averaged**. Even spacing makes the average robust to the
1st and 2nd harmonics of `e(theta)` and to rotor/magnet eccentricity.

### Key parameters (tuned from runtime evidence)

| Parameter | Value | Why |
|---|---|---|
| `TARGETS_PER_POLE` | 6 | Even phases (0/60/.../300°) average out harmonics 1–5 |
| `CAL_VOLTAGE` | 6.0 V | Enough torque margin; clamped to the current ceiling |
| `BRACKET_OFFSET` | 4.0 elec rad | **Must exceed the stiction dead-band** to break the rotor free on *both* sides. Smaller nudges (e.g. 2.5) leave one side stuck → biased midpoint and inconsistent results |
| `HOLD_SETTLE_MS` | 900 ms | Let the rotor fully settle before capturing |
| current ceiling | `MOTOR_CURRENT_LIMIT` (0.8 A) | Drive voltage is clamped so coil current never exceeds this |

> Lesson learned: reducing `BRACKET_OFFSET` below the dead-band made consistency *worse*
> (`circ_std` 0.44 → 1.67), because one approach failed to break free and the bracket
> midpoint jumped ~π at those phases. A *larger* nudge is safer.

### Reading the result

Each run prints, per target, a parseable `TGT` line and a final per-motor summary:

```
>>> electrical offset = 4.0839 rad
consistency: circ_std=0.4399 rad (R=0.9078; lower=better)
mean stiction bracket=1.0473 rad mech (electrical ~7.331 rad)
change vs stored offset = -0.0013 rad
```

- **electrical offset** — the value to store in `constants.h`.
- **circ_std ≈ 0.44 rad** — this is *not* measurement noise; it is the real ±0.5 rad
  2nd-harmonic saliency band. It will not shrink with more samples; the **mean** is the
  correct calibration.
- **change vs stored offset** — after writing the new value and re-running, this should
  collapse to ≈ 0, confirming the stored offset matches physical reality.

## How to (re)calibrate

1. **Mechanically free both motors** — unloaded and able to wiggle ~1 turn. Calibration
   must be done unloaded.
2. **Wire the test in** — have `_stm32g4_main_stub.cpp` call `electrical_offset_autocal_test()`
   (include `tests/sensor/electrical_offset_autocal_test.hpp`).
3. **Build, upload, monitor:**
   ```bash
   pio run -e stm32g4 --target upload -t monitor
   ```
   Runs ~4 minutes (42 targets/motor × 2 motors). The motors twitch back and forth; they
   do **not** spin continuously.
4. **Record the offsets** from the `CALIBRATION SUMMARY` and write them into
   `src/utils/constants.h`:
   ```c
   #define SENSOR_OFFSET_X  4.0852f
   #define SENSOR_OFFSET_Y  2.5624f
   ```
5. **Verify** — rebuild, re-run the autocal, and confirm `change vs stored offset ≈ 0`
   for both motors.
6. **Restore** `_stm32g4_main_stub.cpp` to normal operation.

## Current calibrated values

| Motor | `SENSOR_OFFSET` (electrical rad) |
|-------|---------------------------------:|
| X     | 4.0852 |
| Y     | 2.5624 |

(`pole_pairs = 7`, `SENSOR_DIRECTION = 1` / CCW, AS5048A encoder, drive 6 V @ ≤ 0.8 A.)

## Notes & caveats

- The offsets are mounting-specific. **Re-run calibration after any change** to the
  encoder mount, magnet, or motor wiring.
- The ±0.5 rad 2nd-harmonic saliency error is inherent to these motors with a single
  scalar offset. If tighter accuracy is ever required, it would need a harmonic
  (look-up-table) compensation of `e(theta)`, not a better scalar offset.
- Methods that were tried and **rejected** by runtime evidence: continuous open-loop spin
  with phase regression (rotor ratchets one way, never locks), and quasi-static single-
  sided stepping (biased by stiction). Use the bracket method.
