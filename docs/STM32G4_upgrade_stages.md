# STM32G4 Upgrade — Staged Implementation Plan

Companion to [STM32G4_upgrade_plan.md](STM32G4_upgrade_plan.md). That document fixes the architecture; this one slices the work into small, independently testable stages and defines exactly what must pass on the bench before each stage is considered done.

---

## Ground rules

1. **One stage per branch / PR.** Each stage compiles, uploads, and passes its bench tests in isolation. A stage that needs a later stage to be useful still passes its own tests on its own.
2. **Compile is necessary, not sufficient.** Every stage requires both:
   - `pio run -e teensy41` clean (no regressions on the existing hardware)
   - `pio run -e stm32g4` clean
   - the stage's own bench test(s) passing on the STM32G4 Nucleo on the bench, with scope / serial captures recorded.
3. **One bench-test sketch per stage.** Tests live in `src/tests/<area>/<stage_name>_test.hpp` and are invoked from `setup()` in `main.cpp`, in the same style as the existing tests (e.g. [sensor_test.hpp](src/tests/sensor/sensor_test.hpp), [driver_test.hpp](src/tests/driver/driver_test.hpp)). The body of `loop()` stays empty during a stage test so that nothing else fights for time.
4. **No skipping.** Stages 7–11 in particular form a calibration chain — measure first, then close one loop at a time. Skipping ahead because "the code compiles" is the failure mode this document exists to prevent.
5. **One foot on the ground.** The Teensy build stays functional throughout. If a stage requires touching shared code (e.g. `FOCController.cpp`), the Teensy test for the corresponding subsystem (`sensor_test`, `sine_path_test`) is re-run at the end of that stage before promotion.
6. **Capture everything.** Each stage's PR description has: serial transcripts of the test, scope screenshots where the test calls for them, and a one-line pass/fail per criterion. Bench evidence is the gate, not the diff.

---

## Stage map

| # | Stage | Touches | Bench rig |
| --- | --- | --- | --- |
| 0 | STM32G4 toolchain sanity | platformio.ini, stub main | Nucleo only, USB |
| 1 | AS5048 over SPI on STM32G4 | sensor/*, constants.h | Nucleo + shield + AS5048 + GM2804 (hand-spun) |
| 2 | Garmin V3HP over I2C on STM32G4 | ldm/* | Nucleo + Garmin + known target |
| 3 | Edge-aligned 3PWM, one motor | `PWMDriverSTM32G4` (basic) | Nucleo + shield + scope + DRV8313 only (no motor coil) |
| 4 | Centre-aligned + TRGO + master timer | `PWMDriverSTM32G4`, `stm32_mcu.cpp` | as above |
| 5 | Zero-output `stopPwmHardware` | `PWMDriverSTM32G4` | as above + motor coil |
| 6 | Open-loop FOC with new driver | `FOCController` wiring | full motor rig, no current sense |
| 7 | ADC raw, software start | `InlineCurrentSenseSTM32G4` skeleton | full rig + bench supply on one phase |
| 8 | ADC hardware-triggered + DMA | `InlineCurrentSenseSTM32G4::init` | full rig |
| 9 | `getPhaseCurrents` + offset cal, open-loop logging | full sense class | full rig |
| 10 | Closed-loop Iq | `FOCController::loop` | full rig + dyno fixture |
| 11 | Closed-loop Id | `FOCController::loop` | full rig + dyno fixture |
| 12 | Both motors, full FOC | `main.cpp` | full rig, both motors |
| 13 | Full system integration | `main.cpp`, scan path, LDM | full rig + Garmin + target |
| 14 | SimpleFOC dead-code removal | sweep across `src/` | full rig (regression run) |

Stages 1, 2 and 3 are independent and can run in parallel across different people if the bench has the spare hardware. 4 → 14 are strictly sequential.

---

## Stage 0 — STM32G4 toolchain sanity

**Goal.** Prove `env:stm32g4` builds, uploads, and runs Arduino-framework code on the Nucleo before we trust any other STM32G4 result.

**Work.**
- Add the `[env:stm32g4]` block to [platformio.ini](platformio.ini): `platform = ststm32`, the correct G4 Nucleo `board = `, `framework = arduino`, `upload_port`, `monitor_speed = 115200`.
- Keep `_stm32g4_main_stub.cpp` (or move it into `src/main.cpp` behind `#if defined(STM32G4xx)`) but replace its `setup()`/`loop()` body with a heartbeat: toggle the on-board LED at 1 Hz and `Serial.println("hb ", millis())`.
- Confirm `build_src_filter` excludes Teensy-only test files for the G4 env and excludes G4-only files for the Teensy env.

**New test sketch.** `src/tests/system/stm32g4_heartbeat_test.hpp`.

**Compile gate.** `pio run -e stm32g4` and `pio run -e teensy41` both succeed.

**Bench tests.**
1. Upload to Nucleo over USB. LED toggles at 1 Hz visible to the eye.
2. `pio device monitor -e stm32g4` shows `hb 1000`, `hb 2000`, ... contiguous, no resets.
3. Press the user button (if wired) — optional: log a transition.

**Pass criteria.** 60 s of heartbeat with no missed lines and no reset; teensy build still produces an identical binary hash to pre-change.

**Failure exit.** If serial silently dies, suspect USB CDC vs ST-Link drivers — fix at this stage, not later.

---

## Stage 1 — AS5048 over SPI on STM32G4

**Goal.** Port the angle sensor so `sensor.update()`, `getLatestRotorAngle()`, `getAngularVelocity()`, `getCumulatedRotations()` work on the G4 with the same semantics as on the Teensy.

**Work.**
- Audit [AS5048.cpp](src/sensor/AS5048.cpp) / [Sensor.cpp](src/sensor/Sensor.cpp) for any Teensy-specific calls (`SPI.setMISO`, `SPI.setMOSI`, `SPI.setSCK`, Teensy DMA, etc.).
- If [constants.h](src/utils/constants.h) has comments referring to Teensy pins for SPI MISO/MOSI/SCK, add the equivalent G4 pin notes (MISO `PB14`, MOSI `PB15`, SCK `PB13` per existing comment).
- Wrap any platform-only API behind `#if defined(STM32G4xx)` / `#elif defined(TEENSY41)`.

**New test sketch.** `src/tests/sensor/stm32g4_sensor_test.hpp` — a slimmed copy of [sensor_test.hpp](src/tests/sensor/sensor_test.hpp) that omits the `PWMDriver` / `FOCController` setup (we don't have a working STM32G4 PWM driver yet).

**Compile gate.** Both envs clean.

**Bench tests.**
1. Power-on with the GM2804 free to spin by hand. Serial dumps raw angle + filtered angle + velocity + cumulated rotations at 10 Hz for 30 s.
2. Slowly rotate one full turn CCW (positive direction per `SENSOR_DIRECTION`): `cumulated_rotations` increments by exactly 1.
3. Hold still: `velocity` settles to `< 0.05 rad/s` (LPF dependent).
4. Spin moderately fast by hand: raw angle wraps without producing velocity spikes > 100 rad/s (matches expected behaviour from existing [velocity_spike_test.hpp](src/tests/sensor/velocity_spike_test.hpp)).
5. Run the same test on Teensy with the existing rig. The raw-angle output for the same hand-rotation pattern must match within `±2` LSB of the 14-bit reading.

**Pass criteria.** All five checks pass on the G4. Teensy regression test still passes.

---

## Stage 2 — Garmin V3HP over I2C on STM32G4

**Goal.** Port [GarminV3HP.cpp](src/ldm/GarminV3HP.cpp) / [LDM.cpp](src/ldm/LDM.cpp) so distance measurements work on the G4 with the existing trigger / mode-pin flow.

**Work.**
- Audit [GarminV3HP.cpp](src/ldm/GarminV3HP.cpp) for any Teensy-specific `Wire` setup. Stm32duino's `Wire` defaults to a specific pin pair on Nucleo G4 — confirm `GARMIN_SCL = PB8`, `GARMIN_SDA = PB9` are actually the default I2C1 pins, or call `Wire.setSCL(PB8)` / `Wire.setSDA(PB9)` explicitly.
- Confirm `GARMIN_MODE_PIN = PC9` supports the level / EXTI mode the driver uses.
- Adjust [platformio.ini](platformio.ini) `build_src_filter` so [garmin_hardware_test.cpp](src/tests/ldm/garmin_hardware_test.cpp) is included for the G4 env (it is currently excluded for Teensy).

**New test sketch.** `src/tests/ldm/stm32g4_ldm_test.hpp` — copy of [ldm_test.hpp](src/tests/ldm/ldm_test.hpp) without the motor stack.

**Compile gate.** Both envs clean.

**Bench tests.**
1. Aim Garmin at a static target at 1.00 m. Serial dumps distance at the data-ready interrupt rate for 30 s. Mean reading is `1.00 m ± 0.03 m`; rate is 300–400 Hz (matches the comment in [main.cpp](src/main.cpp)).
2. Move target to 5.00 m. Repeat. Mean within `5.00 m ± 0.10 m`.
3. Block the laser entirely. Driver reports no-return / bad-read code (per `LDM` implementation) and does not crash or stall serial.
4. 5 min soak at 1.00 m. No I2C lockups (no NACK storms in serial); rate stable.

**Pass criteria.** All four checks pass.

---

## Stage 3 — Edge-aligned 3PWM, one motor

**Goal.** Prove the **structure** of `PWMDriverSTM32G4` (allocation, params lifecycle, write path) without yet touching counter mode or TRGO. Hardware here is the SimpleFOC Shield 3.2 + DRV8313 only — **motor coils disconnected**.

**Work.**
- Fill in [PWMDriverSTM32G4.hpp](src/pwm_driver/stm32g4/PWMDriverSTM32G4.hpp) and [.cpp](src/pwm_driver/stm32g4/PWMDriverSTM32G4.cpp). `configure3PWM` delegates to the existing `_configure3PWM` in [stm32_mcu.cpp](src/pwm_driver/stm32g4/stm32_mcu.cpp) **unchanged** (still edge-aligned). `write3PWM` delegates to `_writeDutyCycle3PWM`. `stopPwmHardware` / `cleanupParams` are temporary stubs that simply call `_writeDutyCycle3PWM(0,0,0,p)` and `{}` respectively — we land the final semantics in Stage 5 once the rest works.
- Do **not** delete the 1/2/4/6-PWM helpers yet; cleanup is Stage 14.

**New test sketch.** `src/tests/driver/stm32g4_driver_test.hpp` — copy of [driver_test.hpp](src/tests/driver/driver_test.hpp) for the X motor only, using `PWMDriverSTM32G4`. `setPwm(3.0, 1.5, 0.0)` so the three phases differ.

**Compile gate.** Both envs clean.

**Bench tests** (scope on `N1_X`, `N2_X`, `N3_X` at the shield input):
1. After `driver.init()`, `EN_X` is high.
2. PWM frequency on each pin is `25.0 kHz ± 0.5 %`.
3. Duty cycles match `3.0/12, 1.5/12, 0.0/12` within `±1 %`.
4. After `driver.disable()`, `EN_X` goes low and all three outputs settle low within one PWM period.
5. After `driver.enable()` again, outputs come back with no observable glitch.

**Pass criteria.** All five scope checks pass.

**Failure exit.** If `_configure3PWM` returns `SIMPLEFOC_DRIVER_INIT_FAILED`, run the PinMap audit from [STM32G4_upgrade_plan.md §2.2](STM32G4_upgrade_plan.md). Do not patch around it.

---

## Stage 4 — Centre-aligned + master timer + TRGO

**Goal.** Add centre alignment and master-timer TRGO emission to `_configure3PWM`. Still no ADC — we just verify the timer peripheral does what we'll later trigger off.

**Work.**
- In [stm32_mcu.cpp](src/pwm_driver/stm32g4/stm32_mcu.cpp) (or a centre-aligned variant of `stm32_initPinPWM`): set `TIM_COUNTERMODE_CENTERALIGNED2`.
- After `stm32_alignTimers(timers, 3)` returns the master, configure `LL_TIM_SetTriggerOutput(master, LL_TIM_TRGO_UPDATE)`. If the master is TIM1 or TIM8, set `RCR = 1` so update fires once per PWM period rather than twice.
- Surface the chosen master via a `getMasterTimerInstance()` debug accessor on `PWMDriverSTM32G4` for the test sketch.

**New test sketch.** `src/tests/driver/stm32g4_center_aligned_test.hpp` — same drive pattern as Stage 3, but also poke a GPIO high on each TRGO event via a one-line ISR (or route TRGO to `MCO` / a CH4 output if your master supports it) so we can see TRGO on the scope.

**Compile gate.** Both envs clean.

**Bench tests** (scope):
1. **Centre alignment.** The three phase pulses are symmetric about the same point in the PWM period — rising edges no longer coincide; the *centres* do.
2. **Frequency.** 25 kHz ± 0.5 % held.
3. **TRGO timing.** TRGO toggle is exactly one event per PWM period, located at the count-zero crossover (mid-low). Sanity-print the master timer instance over serial — it should be one of `TIM1`/`TIM8` for the X motor pin set, or document otherwise.
4. **No glitch on duty change.** Step duty from `25 %` to `75 %` via serial command. Centred pulses widen symmetrically; no edge-aligned artefact.

**Pass criteria.** All four scope checks pass.

---

## Stage 5 — Zero-output `stopPwmHardware`

**Goal.** Lock in the architecture decision: `stopPwmHardware` is "set all duties to zero, do nothing else", `cleanupParams` is a no-op. Reconnect the motor coils for this stage so we see the effect on a live load.

**Work.**
- Replace the Stage 3 placeholder bodies with the final versions from [STM32G4_upgrade_plan.md §2.4](STM32G4_upgrade_plan.md) and [§2.5](STM32G4_upgrade_plan.md).
- Add a small `disable()`/`enable()` cycle to `stm32g4_driver_test.hpp` (or copy it into `src/tests/driver/stm32g4_safe_stop_test.hpp`).

**Compile gate.** Both envs clean.

**Bench tests** (motor connected, low duty):
1. `driver.setPwm(3.0, 0.0, 0.0)` and wait — coil pulls current, motor cogs slightly.
2. `driver.disable()` → outputs go to zero, current through phase A drops to zero within one PWM period, `EN_X` goes low. **No high-side flicker, no audible click beyond ordinary cog release.**
3. `driver.enable()` → outputs come back without re-init delay; same duty as before.
4. Repeat 10× in a tight loop. No drift, no `motorsUsed[]` leak (instrument `stm32_getNumMotorsUsed()` over serial — it stays at 1).
5. Run [driver_test.hpp](src/tests/driver/driver_test.hpp) on Teensy unchanged — no regression.

**Pass criteria.** All five checks pass.

---

## Stage 6 — Open-loop FOC with the new driver

**Goal.** Land the new `PWMDriverSTM32G4` into `FOCController` and run the existing voltage-model FOC on the G4. **No current sensing yet.** This isolates "the FOC code ports cleanly" from "the current loop works".

**Work.**
- Replace the `PWMDriverTeensy41` instantiations in `main.cpp` with `PWMDriverSTM32G4` under `#if defined(STM32G4xx)`. Keep both branches.
- Verify [FOCController.cpp](src/foc_controller/FOCController.cpp) still uses only `estimate_Vq` (no current sense path touched yet).
- Wire one motor (X) for this stage; motor Y comes in Stage 12.

**New test sketch.** Reuse [sine_path_test.hpp](src/tests/scan_path/sine_path_test.hpp) but limit it to motor X. Add an STM32G4 instantiation block.

**Compile gate.** Both envs clean.

**Bench tests.**
1. **Calibrate.** `focController1.calibrate()` runs and reports a stable electrical offset within `±0.05 rad` of `SENSOR_OFFSET_1` (or whatever the current good value is for this motor).
2. **Hold.** `setTargetAngle(0)` and let it settle. Steady-state angle within `±0.005 rad` of target after 0.5 s.
3. **Step.** Step target from `0` to `+π/4`. Settling time `< 200 ms`, no overshoot beyond `5 %` of the step.
4. **Sine track.** Run the sine-scan-path-test at the existing amplitude/frequency. Tracking RMS error matches the Teensy baseline within `±20 %`.
5. **No FOC loop overrun.** Log `loop()` interval; matches `SCAN_PATH_UPDATE_PERIOD_US` constraint with margin.

**Pass criteria.** All five checks pass. Teensy sine-path test still passes unchanged.

**Note.** This stage exposes any STM32 floating-point performance issue (the loop runs faster than Teensy in some sections, slower in others). Tune `FOC_VOLTAGE_LPF_TAU` only if a clear capture shows a problem; otherwise leave gains alone.

---

## Stage 7 — ADC raw, software start

**Goal.** Read both current-sense channels off the ACS712 outputs via the shield, **software-triggered**, no DMA, no TRGO. This is the calibration step: we want to see the bias, see the response to a known phase current, and only then move to the hardware-triggered path.

**Work.**
- Add a minimal `InlineCurrentSenseSTM32G4` skeleton with `init()` that:
  - Calls `__HAL_RCC_ADCxxx_CLK_ENABLE` for the chosen ADC (same pattern as the existing `_adc_init_regular` at top of [stm32g4_hal.cpp](src/current_sense/stm32g4/stm32g4_hal.cpp)).
  - Configures `pinA` (`PA0`) and `pinB` (`PA4`) for analog mode via `pinmap_pinout(...)`.
  - Inits the ADC for regular conversions, `ScanConvMode = ENABLE`, `NbrOfConversion = 2`, `ExternalTrigConv = ADC_SOFTWARE_START`, `DMAContinuousRequests = DISABLE`.
  - Runs `HAL_ADCEx_Calibration_Start`.
- `getPhaseCurrents()` for this stage **does** poll (`HAL_ADC_Start`, `HAL_ADC_PollForConversion`, read `HAL_ADC_GetValue` per rank). This is a throw-away path — we replace it in Stage 8 — but it's the simplest correct way to validate hardware first.

**New test sketch.** `src/tests/current_sense/stm32g4_adc_raw_test.hpp`. Logs raw 12-bit code + millivolts for both channels at 100 Hz.

**Compile gate.** Both envs clean. (Teensy env excludes the new file via `build_src_filter`.)

**Bench tests** (PWM driver off, motor open-circuit unless step 3):
1. **Zero-current bias.** Both channels read close to `2048 ± 50` (≈ 1.65 V at the pin) for 30 s. Standard deviation `< 5 LSB`.
2. **Channel independence.** Force phase A high by hand (touch the shield's phase-A current sense input through a 1 kΩ resistor to 3.3 V via a meter — or use a bench supply to push a known current through the phase). Channel A swings, channel B does not.
3. **Polarity.** Push +0.5 A through phase A. Confirm direction of swing (positive or negative from bias). Record sign — Stage 9 uses this to set `gain_a` polarity.
4. **Cross-talk.** Same as 3 but on phase B. Confirm channel A barely moves.

**Pass criteria.** All four checks pass.

**Note.** If the bias is significantly away from `2048`, do not try to compensate in firmware — the shield's analog frontend is broken or the wrong ACS712 module is installed.

---

## Stage 8 — ADC hardware-triggered + DMA

**Goal.** Replace the polled path with the architecture: ADC triggered by master-timer TRGO, results streamed by DMA into `sample_buf[2]`. The motor is still driven open-loop; `getPhaseCurrents()` now becomes a non-blocking buffer read.

**Work.**
- `InlineCurrentSenseSTM32G4::init()` changes:
  - `ExternalTrigConv` = the regular-trigger source matching the driver's master timer (write a `stm32_timerToRegularTRGO()` helper analogous to the existing injected version in [stm32g4_hal.cpp](src/current_sense/stm32g4/stm32g4_hal.cpp:163)).
  - `ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING`.
  - `DMAContinuousRequests = ENABLE`, `EOCSelection = ADC_EOC_SEQ_CONV`, `Overrun = ADC_OVR_DATA_OVERWRITTEN`.
  - Init the DMA stream: half-word, P2M, circular, mem-inc, no IT.
  - `HAL_ADC_Start_DMA(adc, sample_buf, 2)`.
- `getPhaseCurrents()` becomes the non-blocking version from [STM32G4_upgrade_plan.md §3.4](STM32G4_upgrade_plan.md).

**New test sketch.** `src/tests/current_sense/stm32g4_adc_dma_test.hpp`. Logs the buffer contents at 100 Hz **and** at the same time toggles a GPIO inside a dummy `HAL_ADC_ConvCpltCallback` (or watches the DMA `NDTR` register decrement) so we can verify the sample rate on a scope.

**Compile gate.** Both envs clean.

**Bench tests** (PWM driver running, output to motor coils):
1. **Bias.** Same as Stage 7 step 1 — both channels around `2048 ± 50` when zero-volt is applied to all phases.
2. **Sample rate.** Scope-confirm the ADC EOS event rate equals the PWM frequency (25 kHz), or PWM/2 (12.5 kHz) if RCR could not be set on the chosen master. Document whichever was achieved.
3. **Match Stage 7.** Repeat the Stage 7 step-3 polarity test. The DMA-buffer-derived current matches the polled measurement within `±10 LSB`.
4. **PWM-modulation sync.** Drive `setPwm(6, 0, 0)` (open-loop, large duty on phase A). Capture `sample_buf[0]` over 1 s. Mean is offset from bias as expected; **ripple** in the buffer reading is at most a few LSB — confirming the trigger samples at a stable phase of the PWM cycle, not on a beat.
5. **Stability.** 10 min soak driving a sine voltage on phase A. No DMA halt (`NDTR` keeps decrementing), no overrun (`OVR` bit stays clear).

**Pass criteria.** All five checks pass.

**Failure exit.** If `OVR` keeps setting, drop the trigger rate (Section 2.2 step 6 in the plan: configure `RCR` or move TRGO to `OC4REF`). Don't increase ADC clock to compensate — that masks the real issue.

---

## Stage 9 — `InlineCurrentSenseSTM32G4` full class: offset cal + logging

**Goal.** Wrap Stages 7–8 into the final class shape and verify `Iq` / `Id` computation against open-loop voltage drives. Still no closed-loop control — we're proving the measurement side.

**Work.**
- Add `calibrateOffsets`, `getFOCCurrents`, signed `gain_a` / `gain_b`, `offset_ia` / `offset_ib`, as per [STM32G4_upgrade_plan.md §3.1](STM32G4_upgrade_plan.md).
- Wire `InlineCurrentSenseSTM32G4 current_sense1(ACS712_MV_PER_AMP, ADC_CS_X_1, ADC_CS_X_2, pwmDriver1);` into `main.cpp` (G4 branch).
- In `FOCController::loop()`, **for this stage only**, after the existing `setPhaseVoltage()` call, sample `dq = current_sense.getFOCCurrents(electrical_angle)` and stash `Iq` and `Id` for logging. No control change.

**New test sketch.** `src/tests/current_sense/stm32g4_iq_open_loop_test.hpp`. Runs a slow electrical-angle ramp via direct `setPhaseVoltage` (`Vq = 2.0 V, Vd = 0`) while logging `Iq_meas`, `Id_meas`, raw a/b currents, and electrical angle at 200 Hz.

**Compile gate.** Both envs clean.

**Bench tests.**
1. **Calibration.** `calibrateOffsets()` after `pwm_driver.init()` with `setPwm(0,0,0)`. Offsets within `1.60 V – 1.70 V` for both channels. `gain_a`, `gain_b` set from `ACS712_MV_PER_AMP` (signed, polarity set per Stage 7 step 3).
2. **Sign sweep.** With `Vq = +1.5 V`, slowly rotate the rotor by hand against the field. `Iq_meas` is positive and roughly constant; `Id_meas` stays near zero. If `Iq_meas` is negative, flip `gain_a` (and/or `gain_b`) sign and re-test.
3. **Magnitude sanity.** With `Vq = 0.5 V` at locked rotor, `Iq_meas ≈ Vq / motor_phase_resistance` within `±30 %`. (Resistance estimate is rough; this is a gross check that we're in amps not millivolts.)
4. **`i_c = -(i_a + i_b)` self-consistency.** Plot `i_a + i_b + i_c` from the log offline — should sum to zero within noise.
5. **Noise floor.** With `setPwm(0,0,0)`, `Iq_meas` standard deviation `< 0.05 A`. If higher, suspect ground loop on the shield — fix hardware, don't filter.

**Pass criteria.** All five checks pass.

---

## Stage 10 — Closed-loop Iq

**Goal.** Replace `Vq = estimate_Vq(target_q_current)` with the closed-loop form from [STM32G4_upgrade_plan.md §4.2](STM32G4_upgrade_plan.md). `Vd` is **still zero** at this stage — we close one axis at a time.

**Work.**
- Add `PIDControl PID_iq{...}` to `FOCController`.
- Reset `PID_iq` inside `enable()` / `disable()` / mode-switch in [FOCController.cpp](src/foc_controller/FOCController.cpp:151).
- In `loop()`: insert the current sense block, compute `iq_error = target_q_current - Iq_meas`, set `Vq = estimate_Vq(target_q_current) + PID_iq(iq_error)`. `Vd` stays zero.
- Add `IQ_P`, `IQ_I`, `IQ_D`, `IQ_RAMP` to [tweakable_params.h](src/utils/tweakable_params.h). Start with `IQ_P = 0.05`, `IQ_I = 0`, `IQ_D = 0`, `IQ_RAMP = 1e6`.

**New test sketch.** `src/tests/current_sense/stm32g4_iq_closed_loop_test.hpp`. Steps `target_q_current` (`0 A → 0.2 A → 0 A → -0.2 A`) at 1 Hz while running the existing angle-hold loop at a fixed `setTargetAngle(0)`.

**Compile gate.** Both envs clean.

**Bench tests.**
1. **P-only step response.** With `IQ_P = 0.05`, `0.2 A` step: `Iq_meas` settles to within `10 %` of target in `< 100 ms`, no oscillation. Capture and attach.
2. **Tune `I`.** Raise `IQ_I` until steady-state error is `< 5 %` of target. Stop when overshoot exceeds `10 %` or oscillation appears.
3. **Angle hold under disturbance.** Tap the rotor by hand at fixed target. Angle returns to target; `Iq_meas` spikes and decays smoothly. **No `Iq` ringing at the ADC sample rate.**
4. **Voltage limit.** Force `target_q_current = +5 A` (above `MOTOR_CURRENT_LIMIT`). `Vq` clamps at `MOTOR_VOLTAGE_LIMIT`; no integral windup beyond reset.
5. **Existing angle/velocity behaviour intact.** Run [sine_path_test.hpp](src/tests/scan_path/sine_path_test.hpp) on the G4. Angle tracking RMS matches the Stage 6 baseline within `±20 %`.
6. **Teensy regression.** Re-run the Teensy sine-path test. The shared `FOCController` changes must be gated by `#if defined(STM32G4xx)` or otherwise inert on Teensy — if the Teensy tracking changes, **fix the gating before promoting**.

**Pass criteria.** All six checks pass.

---

## Stage 11 — Closed-loop Id

**Goal.** Add `Vd` regulation. Target `Id = 0`. Same structure as Stage 10 on the d-axis.

**Work.**
- Add `PIDControl PID_id{...}` and `ID_*` constants. Same starting gains as `IQ_*`.
- In `loop()`: `id_error = 0 - Id_meas; Vd = PID_id(id_error);`. Constrain `Vd` to `±MOTOR_VOLTAGE_LIMIT`.

**New test sketch.** Extend `stm32g4_iq_closed_loop_test.hpp` to log `Id_meas` and `Vd`.

**Compile gate.** Both envs clean.

**Bench tests.**
1. **Idle.** At `target_q_current = 0`, `Id_meas` stays within `±0.05 A` of zero.
2. **Under torque.** At `target_q_current = +0.3 A` (locked rotor), `Id_meas` stays within `±0.1 A` of zero. Without the d-loop, `Id` would drift toward whatever the inverse Park error term produces.
3. **Step in q.** Step `target_q_current` `0 → 0.2 A`. `Id_meas` glitches no more than `±0.15 A` during the transient and settles back to zero within `200 ms`.
4. **Sine path with d-loop closed.** Run sine path. Angle RMS error matches Stage 10's value within `±20 %`. `|Id_meas|` 95th percentile `< 0.1 A`.

**Pass criteria.** All four checks pass.

---

## Stage 12 — Both motors, full FOC

**Goal.** Bring the Y motor up to parity. Validate that timer / DMA / ADC allocation does not collide.

**Work.**
- Instantiate `pwmDriver2 = PWMDriverSTM32G4(N1_Y, N2_Y, N3_Y, EN_Y)` and `current_sense2 = InlineCurrentSenseSTM32G4(ACS712_MV_PER_AMP, ADC_CS_Y_1, ADC_CS_Y_2, pwmDriver2)`.
- Init both in `main.cpp` (G4 branch). Print `stm32_getNumMotorsUsed()`, `stm32_getNumTimersUsed()`, and the master-timer instance for each motor at startup.

**New test sketch.** `src/tests/system/stm32g4_dual_motor_test.hpp`. Hold motor X at `0` and motor Y at `+π/4`. Then drive the existing sine-scan path on both.

**Compile gate.** Both envs clean.

**Bench tests.**
1. **Startup print.** `numMotorsUsed == 2`. Master timers are distinct instances. If they collide, fail and read the section "Dual-motor allocation risk" in [STM32G4_architecture_report.md §4.3](STM32G4_architecture_report.md) — do not paper over.
2. **Independent holds.** X holds `0`, Y holds `+π/4`. Disturbing one does not move the other.
3. **Sine path both motors.** Both axes track within Stage 11 RMS bounds.
4. **ADC ownership.** Print which ADC each `InlineCurrentSenseSTM32G4` selected. They are different ADC instances (or, if shared, both DMA streams are independent and `OVR` never fires).
5. **15 min soak.** No drift, no resets, no `OVR`, no I2C / SPI degradation (Garmin and AS5048 are not active in this test — they come back in Stage 13).

**Pass criteria.** All five checks pass.

---

## Stage 13 — Full system integration

**Goal.** Wire LDM (Stage 2) and both motors (Stage 12) into the production `loop()`. This is the "does the whole product work on STM32G4" check.

**Work.**
- Restore the original `main.cpp` shape (Interface + StateMachine + DataStream + ScanPath + Garmin + two motors) under the G4 branch, with `PWMDriverSTM32G4` and `InlineCurrentSenseSTM32G4`.

**Bench tests.**
1. **Cold start.** `Serial` comes up, all init prints land, calibration succeeds for both motors, Garmin reports first distance within 2 s.
2. **`enable` + `scan`.** Issue the existing serial commands to enable and start scanning (matches Teensy flow). Both motors track; Garmin streams; `DataStream.sendData()` emits records.
3. **Telemetry rate.** `sendData()` rate ≥ existing Teensy baseline (record actual on Teensy first if not already known).
4. **30 min soak under scan.** No resets, no missed distance reads, no Iq runaway, no thermal trip on DRV8313s.
5. **Disable / re-enable cycle.** 10 cycles. Outputs go safe each disable, recover each enable, no leak in `motorsUsed[]`.
6. **Teensy regression.** Final Teensy build is functionally identical to the pre-port version on the same scan path.

**Pass criteria.** All six checks pass.

---

## Stage 14 — Dead-code removal

**Goal.** Delete every SimpleFOC import path the new system does not reach. This stage exists separately so that if anything breaks, the breakage is a single isolated PR.

**Work.** Remove per [STM32G4_upgrade_plan.md §5.3](STM32G4_upgrade_plan.md):
- [src/current_sense/CurrentSense.h](src/current_sense/CurrentSense.h), [.cpp](src/current_sense/CurrentSense.cpp)
- [src/current_sense/InlineCurrentSense.hpp](src/current_sense/InlineCurrentSense.hpp), [.cpp](src/current_sense/InlineCurrentSense.cpp)
- [src/current_sense/hardware_api.h](src/current_sense/hardware_api.h)
- [src/current_sense/stm32g4/stm32g4_mcu.cpp](src/current_sense/stm32g4/stm32g4_mcu.cpp)
- Injected-channel / IRQ paths in [stm32g4_hal.cpp](src/current_sense/stm32g4/stm32g4_hal.cpp), [stm32g4_utils.cpp](src/current_sense/stm32g4/stm32g4_utils.cpp)
- `_configure{1,2,4,6}PWM`, `_writeDutyCycle{1,2,4,6}PWM`, `_setSinglePhaseState`, MBED branches in [stm32_mcu.cpp](src/pwm_driver/stm32g4/stm32_mcu.cpp)
- The compile-check `_stm32g4_main_stub.cpp` (once the real `main.cpp` G4 branch ships).

**Compile gate.** Both envs clean.

**Bench tests.** Re-run Stage 13 in full. Every criterion must still pass with byte-for-byte equivalent telemetry (or within noise of it).

**Pass criteria.** Stage 13 passes again; binary size for the G4 env drops measurably (record before/after).

---

## What promotion looks like

For each stage:

1. Branch off `main`.
2. Implement only the work listed for that stage.
3. Land the new test sketch under `src/tests/<area>/`.
4. Run both compile gates.
5. Run all bench tests with the rig described in the stage map. Record serial transcripts and scope captures.
6. Open the PR; paste pass/fail per criterion and attach captures.
7. Merge.
8. **Do not start the next stage until merged** — keeps regressions isolated and keeps the Teensy build always-shippable.

If a bench test fails, the bias is to fix the cause, not to loosen the criterion. The criteria above are tight on purpose: they represent the minimum standard for trusting the next stage's measurements.
