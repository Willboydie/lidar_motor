# STM32G4 Upgrade Plan

Concrete plan for porting the lidar_motor firmware from Teensy 4.1 to an STM32G4 Nucleo with the SimpleFOC Shield 3.2, DRV8313, GM2804 + AS5048A encoder, and ACS712 inline current sensing. This document supersedes [STM32G4_architecture_report.md](STM32G4_architecture_report.md) and pins down a set of design decisions that resolve the open questions in that report.

---

## 0. Architecture decisions (this report)

These are fixed and drive every section that follows. Treat anything in the prior architecture report that conflicts with them as obsolete.

1. **ADC acquisition is hardware-triggered.** Phase-current sampling is started by `TRGO` from the **master timer** of the 3PWM driver. Conversion results are transferred to a small ring/double-buffer via **DMA**. The CPU never polls or starts conversions.
2. **`getPhaseCurrents()` is a non-blocking buffer read.** It returns the most recent samples already deposited by DMA, scaled to amps, with no `HAL_ADC_Start` / wait. The FOC loop runs at its own rate against whatever the latest ADC frame is.
3. **`stopPwmHardware()` zeros all duty cycles only.** It mirrors the current Teensy implementation: set all three outputs to a safe state and stop. **No deinit, no `HAL_TIM_PWM_DeInit`, no freeing of `STM32DriverParams`, no removal from `motorsUsed`.** Driver objects are long-lived; bring-up is one-shot.
4. **FOC loop uses measured currents.** Inside `FOCController::loop()`, `getPhaseCurrents()` is called, Clarke + Park give `Id_meas` / `Iq_meas`, and the voltage model is rewritten so that `Vq` is driven by the **error `(target_q_current - Iq_meas)`** instead of `estimate_Vq(target_q_current)` against the resistor/Kv model alone. `Vd` is driven by `(0 - Id_meas)`.
5. **5 V → 3.3 V conditioning is handled by the SimpleFOC Shield 3.2.** Firmware sees a 0–3.3 V signal at the MCU pin. No divider math, no gain scaling for a divider; offsets come from `calibrateOffsets()` at the pin.
6. **`PWMDriverSTM32G4` implements centre-aligned 3PWM.** Same public contract as `PWMDriverTeensy41`: `configure3PWM`, `write3PWM`, `stopPwmHardware`, `cleanupParams`. Centre alignment (`TIM_COUNTERMODE_CENTERALIGNED2`) gives a usable TRGO mid-period for ADC sync.
7. **The system is built by adapting existing SimpleFOC code, with all unused code removed.** We keep the timer-allocation / alignment / centre-aligned-PWM and ADC injected/regular helpers we actually need; everything else in the imported SimpleFOC subtrees gets deleted before merge.

---

## 1. Target system at a glance

```
AS5048A (SPI) ─► Sensor ─┐
                         │
                         ▼
                  FOCController::loop()
   InlineCurrentSenseSTM32G4 ──► Id_meas, Iq_meas
                         │            │
                         ▼            ▼
            Vd = Kd · (0 − Id_meas)   Vq = Kq · (Iq_target − Iq_meas)
                         │            │     + feed-forward (resistor + Kv)
                         ▼            ▼
                  inverse Park + Clarke ─► Va, Vb, Vc
                         │
                         ▼
                  PWMDriverSTM32G4
                  centre-aligned 3PWM, master TIM emits TRGO
                         │            │
                         ▼            ▼
                       DRV8313 IN[A:C]   master_timer ──► ADC trigger
                         │                       │
                         ▼                       ▼
                       GM2804                 ADC + DMA ─► sample buffer
                                                          ▲
                                                          │
                                              getPhaseCurrents() reads buffer
```

Two motors run two independent 3PWM groups, each with its own master timer and its own ADC. There is **no** cross-motor synchronisation; current sensing for motor X is triggered by motor X's master timer, and likewise for motor Y.

---

## 2. PWM subsystem

### 2.1 Public interface (unchanged)

[src/pwm_driver/PWMDriver.hpp](src/pwm_driver/PWMDriver.hpp) defines `init`/`deinit`/`enable`/`disable`/`setPwm`/`getVoltageLimit` and four pure-virtual hooks:

- `configure3PWM(freq, pinA, pinB, pinC) → void* params`
- `write3PWM(dutyA, dutyB, dutyC, params)`
- `stopPwmHardware(params)`
- `cleanupParams(params)`

`PWMDriverSTM32G4` matches that contract exactly so `FOCController` is untouched at the type level.

### 2.2 `configure3PWM` (centre-aligned, master TRGO)

Implementation steps inside `configure3PWM`:

1. Constrain `pwm_frequency` to `[0, MOTOR_PWM_FREQUENCY_MAX]`; default to `MOTOR_PWM_FREQUENCY` (25 kHz) if zero.
2. Resolve the three pins to `PinMap_TIM` entries via the existing `stm32_findBestTimerCombination(3, pins, pinTimers)` in [src/pwm_driver/stm32g4/searchtimers.cpp](src/pwm_driver/stm32g4/searchtimers.cpp).
3. For each of the three pins, call `stm32_initPinPWM(freq, pinTimer, TIM_OCMODE_PWM1, TIM_OCPOLARITY_HIGH)` — **but** override the counter mode to `TIM_COUNTERMODE_CENTERALIGNED2` in the helper (or add a centre-aligned variant of `stm32_initPinPWM` that does so). This is the only behavioural change from the SimpleFOC 3PWM helper we already have.
4. Run `stm32_checkTimerFrequency(...)` and store channels / LL channels in a freshly allocated `STM32DriverParams`.
5. Call `stm32_alignTimers(timers, 3)` to pick the master and chain slaves in slave-mode reset. Store the chosen master in `params->master_timer`.
6. Configure the master timer to emit `TRGO = TIM_TRGO_UPDATE`. The update event fires twice per PWM period in centre-aligned mode; configure the timer's `RCR` (repetition counter, where available — TIM1/TIM8 have one) so the ADC fires once per PWM period, at the count-zero (centre-of-low) event.
7. Register the driver in `motorsUsed[]` so timer-channel collisions across the two motors are still tracked.
8. Return the `STM32DriverParams*`.

Pins are fixed in [src/utils/constants.h](src/utils/constants.h) (`N1_X=PA7, N2_X=PC7, N3_X=PB3, EN_X=PA8` and `N1_Y=PA5, N2_Y=PB6, N3_Y=PB4, EN_Y=PA9`). The allocator must come out the other side with **six distinct timer instances across both motors**, or with intentional sharing documented. This is a build-time/init-time check; if it fails, the only resolutions are hardware rework or accepting a different STM32G4 part — there is nothing the driver can do at runtime.

### 2.3 `write3PWM`

Thin wrapper around the existing helper:

```cpp
void PWMDriverSTM32G4::write3PWM(float dcA, float dcB, float dcC, void* p) {
    _writeDutyCycle3PWM(dcA, dcB, dcC, p);
}
```

The helper at [src/pwm_driver/stm32g4/stm32_mcu.cpp](src/pwm_driver/stm32g4/stm32_mcu.cpp) already scales each duty by `ARR+1` and writes via `stm32_setPwm` per timer/channel. It also handles the single-timer-three-channel case with `LL_TIM_DisableUpdateEvent` / `LL_TIM_EnableUpdateEvent` around the writes — we keep that path even though our pin set forces three timers, because the helper is reused unchanged.

### 2.4 `stopPwmHardware` (decision: zero outputs only)

```cpp
void PWMDriverSTM32G4::stopPwmHardware(void* p) {
    _writeDutyCycle3PWM(0.0f, 0.0f, 0.0f, p);
}
```

That is the whole function. It mirrors `PWMDriverTeensy41::stopPwmHardware`'s intent — bring the high-side drivers to a known idle — without touching the timer peripherals, `motorsUsed[]`, or any allocated state. Counters keep running; `TRGO` keeps firing; ADC keeps sampling. **No `HAL_TIM_PWM_DeInit`, no `HAL_NVIC_DisableIRQ`, no `delete`.**

### 2.5 `cleanupParams`

`PWMDriver::deinit()` calls this after `stopPwmHardware`. Because we explicitly do not tear down timer hardware and because driver objects are long-lived, **`cleanupParams` is a no-op** (`{}`). The `STM32DriverParams*` returned by `configure3PWM` is owned by the global `motorsUsed[]` array; we deliberately do not free it. This avoids the lifecycle mismatch between the imported SimpleFOC `_configure3PWM` (which allocates and registers globally) and the Teensy-style ownership the `PWMDriver` base class implies.

The base-class `deinit()` in [src/pwm_driver/PWMDriver.cpp](src/pwm_driver/PWMDriver.cpp) sets `params = nullptr` after `cleanupParams`; that is acceptable — it only breaks the local pointer, not the global registration.

### 2.6 What gets deleted from the SimpleFOC import

[src/pwm_driver/stm32g4/stm32_mcu.cpp](src/pwm_driver/stm32g4/stm32_mcu.cpp) is currently 751 lines and contains paths for **1/2/3/4/6 PWM** plus 6PWM software dead-time, phase-state handling, and various unused entry points. After adaptation we keep:

- timer-bookkeeping (`numTimersUsed`, `motorsUsed`, `stm32_isTimerUsed`, `stm32_isChannelUsed`, `stm32_useTimer`, `stm32_reserveTimer`, `stm32_getMotorUsed`) — used for the dual-motor collision check;
- `stm32_pause` / `stm32_resume` — small, harmless, useful when we later add MOE gating;
- `_configure3PWM` — adapted for centre alignment + TRGO + RCR;
- `_writeDutyCycle3PWM` — unchanged;
- the supporting helpers in [src/pwm_driver/stm32g4/timerutils.cpp](src/pwm_driver/stm32g4/timerutils.cpp) and [src/pwm_driver/stm32g4/searchtimers.cpp](src/pwm_driver/stm32g4/searchtimers.cpp) that those call.

Everything else — `_configure1PWM`, `_configure2PWM`, `_configure4PWM`, `_configure6PWM`, `_writeDutyCycle{1,2,4,6}PWM`, `_setSinglePhaseState`, MBED branches, stepper-specific code — is removed.

---

## 3. Current sense subsystem

### 3.1 Class shape: `InlineCurrentSenseSTM32G4`

Header: a small class **not** inheriting from SimpleFOC's `CurrentSense` (which drags `FOCDriver`, `BLDCDriver`, `StepperDriver`, `linkDriver`, `driverAlign`, `getDCCurrent`, etc.).

```cpp
class InlineCurrentSenseSTM32G4 {
public:
    InlineCurrentSenseSTM32G4(float mVpA, int pinA, int pinB, PWMDriverSTM32G4& driver);

    bool init();                       // GPIO + ADC + DMA + TRGO wiring
    void calibrateOffsets(int rounds = 1000);
    PhaseCurrent_s getPhaseCurrents(); // non-blocking buffer read
    DQCurrent_s    getFOCCurrents(float angle_el); // Clarke + Park

    float gain_a, gain_b, gain_c;      // signed; flip to correct polarity
    float offset_ia, offset_ib, offset_ic;

private:
    int pinA, pinB;
    PWMDriverSTM32G4& driver;          // for TRGO source / master_timer
    ADC_HandleTypeDef* adc;
    DMA_HandleTypeDef  hdma;
    volatile uint16_t  sample_buf[2];  // raw 12-bit codes, one per channel
};
```

Two channels per motor; phase C is reconstructed as `i_c = -(i_a + i_b)`.

### 3.2 `init()`: hardware-triggered ADC + DMA

Step-by-step, all inside `init()`:

1. **GPIO**: configure `pinA` and `pinB` as analog inputs via `pinmap_pinout(analogInputToPinName(pin), PinMap_ADC)`.
2. **ADC pick**: choose an ADC that maps both pins to channels on the same instance (use the existing `_findBestADC...` helper, adapted to regular channels rather than injected).
3. **ADC init** (`HAL_ADC_Init`):
   - `Resolution = ADC_RESOLUTION_12B`
   - `ScanConvMode = ADC_SCAN_ENABLE`
   - `ContinuousConvMode = DISABLE` (one trigger → one scan of two channels)
   - `ExternalTrigConv = ADC_EXTERNALTRIG_T<N>_TRGO` chosen to match the master timer (`stm32_timerToRegularTRGO(master)` — analogous to the existing injected helper)
   - `ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING`
   - `DMAContinuousRequests = ENABLE`
   - `EOCSelection = ADC_EOC_SEQ_CONV`
   - `Overrun = ADC_OVR_DATA_OVERWRITTEN` (we always want the freshest sample)
   - `NbrOfConversion = 2`
4. **Channel configs**: rank 1 → phase A pin's ADC channel; rank 2 → phase B pin's ADC channel. Sampling time = `ADC_SAMPLETIME_2CYCLES_5` (matches the existing SimpleFOC G4 setting and is fine for ACS712 bandwidth).
5. **DMA**: configure a DMA stream/channel attached to the chosen ADC for **half-word, peripheral-to-memory, circular, memory-increment**. Target buffer is `sample_buf[2]`.
6. **ADC calibration**: `HAL_ADCEx_Calibration_Start(adc, ADC_SINGLE_ENDED)`.
7. **TRGO source**: on `driver.params->master_timer->Instance`, set `LL_TIM_SetTriggerOutput(..., LL_TIM_TRGO_UPDATE)`. In centre-aligned mode, the update event fires twice per PWM period (count = 0 and count = ARR). Use the timer's `RCR` (repetition counter) on TIM1/TIM8 to halve that so the ADC fires once per period, in the middle of the low side of the PWM — this is where current ripple is at its mean.
8. **Start**: `HAL_ADC_Start_DMA(adc, (uint32_t*)sample_buf, 2)`.

After this, the master timer continuously fires the ADC, DMA continuously overwrites `sample_buf`, and no CPU is involved.

### 3.3 `calibrateOffsets()`

```cpp
void InlineCurrentSenseSTM32G4::calibrateOffsets(int rounds) {
    // PWM duty = 0 must already have been set by the caller.
    double sum_a = 0, sum_b = 0;
    for (int i = 0; i < rounds; ++i) {
        sum_a += sample_buf[0] * adc_voltage_conv;
        sum_b += sample_buf[1] * adc_voltage_conv;
        delayMicroseconds(50);
    }
    offset_ia = sum_a / rounds;
    offset_ib = sum_b / rounds;
    offset_ic = 0.0f; // unused
}
```

`adc_voltage_conv = 3.3f / 4096.0f`. Because the Shield 3.2 has already mapped the ACS712 0–5 V swing into 0–3.3 V at the pin, `offset_ia`/`offset_ib` come out close to `1.65 V` and stay there.

### 3.4 `getPhaseCurrents()`: non-blocking buffer read

```cpp
PhaseCurrent_s InlineCurrentSenseSTM32G4::getPhaseCurrents() {
    float Va = sample_buf[0] * adc_voltage_conv;
    float Vb = sample_buf[1] * adc_voltage_conv;
    PhaseCurrent_s i;
    i.a = (Va - offset_ia) * gain_a;
    i.b = (Vb - offset_ib) * gain_b;
    i.c = -(i.a + i.b);
    return i;
}
```

No `HAL_ADC_Start`, no polling, no `HAL_ADC_PollForConversion`. The FOC loop pays exactly two volatile reads, two subtracts, two multiplies, and one negate-add per call.

`gain_a`, `gain_b` start as `1000.0f / mVpA` (e.g. for the 5 A grade: `1000 / 185 ≈ 5.405 A/V`). Polarity is fixed by sign during bring-up (Section 6).

### 3.5 What gets deleted from the SimpleFOC current-sense import

[src/current_sense/CurrentSense.h](src/current_sense/CurrentSense.h) drags `FOCDriver`, `BLDCDriver`, `StepperDriver`, `linkDriver`, `driverAlign`, `getDCCurrent`, `alignBLDCDriver`, `alignStepperDriver`, `alignHybridDriver`, and `readAverageCurrents`. None of those are reachable from the new design.

Removed wholesale:

- [src/current_sense/CurrentSense.h](src/current_sense/CurrentSense.h) and [src/current_sense/CurrentSense.cpp](src/current_sense/CurrentSense.cpp) — base class no longer used.
- [src/current_sense/InlineCurrentSense.hpp](src/current_sense/InlineCurrentSense.hpp) and [src/current_sense/InlineCurrentSense.cpp](src/current_sense/InlineCurrentSense.cpp) — replaced by `InlineCurrentSenseSTM32G4`.
- [src/current_sense/hardware_api.h](src/current_sense/hardware_api.h) (`_configureADCLowSide`, `_configureADCInline`, `_driverSyncLowSide`, `_readADCVoltageLowSide`, `_readADCVoltageInline`) — direct HAL calls inside `InlineCurrentSenseSTM32G4::init()` replace this layer.
- [src/current_sense/stm32g4/stm32g4_mcu.cpp](src/current_sense/stm32g4/stm32g4_mcu.cpp) — injected-channel + interrupt path; not needed for regular-channel DMA.
- The injected-channel arm of [src/current_sense/stm32g4/stm32g4_hal.cpp](src/current_sense/stm32g4/stm32g4_hal.cpp) (`_adc_init`, `Stm32CurrentSenseParams`, `Stm32AdcInterruptConfig`, `HAL_ADCEx_InjectedConvCpltCallback`). The clock-enable block at the top (`__HAL_RCC_ADCxxx_CLK_ENABLE`) and `_adc_gpio_init` are kept and moved inline into `InlineCurrentSenseSTM32G4::init()` (or a small `stm32g4_adc_utils.cpp`).

The ADC IRQ handlers (`ADC1_2_IRQHandler` etc.) are removed; DMA half/full-complete callbacks are not used either — the FOC loop just samples whatever DMA last deposited.

---

## 4. FOC controller changes

### 4.1 Today's loop (voltage model only)

In [src/foc_controller/FOCController.cpp](src/foc_controller/FOCController.cpp:43-86) the loop is:

```
sensor.update() → mechanical_angle, electrical_angle, angular_velocity
[ANGLE mode]    → angular_error → PID_angle → target_velocity
velocity_error   = target_velocity − angular_velocity
target_q_current = PID_velocity(velocity_error) + K_vff · target_velocity
Vq               = estimate_Vq(target_q_current)         ← no measured current
Vq               = LPF(Vq)
setPhaseVoltage()                                         ← inverse Park + Clarke, midpoint clamp, setPwm
```

`estimate_Vq` is `I·R + ω/(K_v·√3·RPM_TO_RADS)`.

### 4.2 New loop (current closed-loop)

The decision is: **`Vq` is driven by the error `(target_q_current − Iq_meas)`**. The resistor/Kv model becomes a feed-forward term, not the primary control output.

```
sensor.update()
[ANGLE mode] → target_velocity = PID_angle(angular_error)
velocity_error   = target_velocity − angular_velocity
target_q_current = PID_velocity(velocity_error) + K_vff · target_velocity

PhaseCurrent_s i = current_sense.getPhaseCurrents();   // non-blocking
DQCurrent_s    dq = current_sense.getFOCCurrents(electrical_angle);
Iq_meas = dq.q;
Id_meas = dq.d;

iq_error = target_q_current − Iq_meas
id_error = 0.0f          − Id_meas

Vq = estimate_Vq(target_q_current)           // feed-forward
   + PID_iq(iq_error);                        // closed-loop correction
Vd = PID_id(id_error);                        // d-axis regulation, target = 0

Vq = LPF_voltage(Vq);
Vq = constrain(Vq, ±MOTOR_VOLTAGE_LIMIT);
Vd = constrain(Vd, ±MOTOR_VOLTAGE_LIMIT);
setPhaseVoltage();   // existing inverse Park + Clarke + midpoint clamp + setPwm
```

Concrete code changes inside `FOCController`:

- Constructor: take an extra `InlineCurrentSenseSTM32G4&` and store as a reference member.
- `init()`: after `pwm_driver.init()`, call `current_sense.init()` and then `current_sense.calibrateOffsets()` while the driver is enabled with 0 V on all phases (already the post-`init` state).
- New members:
  - `PIDControl PID_iq{IQ_P, IQ_I, IQ_D, IQ_RAMP, -MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT};`
  - `PIDControl PID_id{ID_P, ID_I, ID_D, ID_RAMP, -MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT};`
  - getters for `Iq`, `Id`, `Vd`, and the new targets (already partly present).
- `loop()`: insert the current read + DQ + PID block above between `target_q_current` saturation and the existing `Vq = estimate_Vq(...)` line. Replace the existing `Vq = estimate_Vq(...)` line with the sum of feed-forward + `PID_iq`.
- `enable()` / `disable()`: also reset `PID_iq` and `PID_id`.
- `setPhaseVoltage()` is unchanged — it already does inverse Park (`Valpha = ca·Vd − sa·Vq`, `Vbeta = sa·Vd + ca·Vq`) and the midpoint clamp; `Vd` is just no longer hard-zero.

New gains live in [src/utils/tweakable_params.h](src/utils/tweakable_params.h) (e.g. `IQ_P`, `IQ_I`, `IQ_D`, `IQ_RAMP`, `ID_P`, `ID_I`, `ID_D`, `ID_RAMP`). Initial values: very small `P` (~0.05 V/A), zero `I` and `D`, low `RAMP` — see Section 6.

### 4.3 What is kept vs deleted in the FOC layer

Kept:
- The angle/velocity cascade, mechanical-angle tracking, `setPhaseVoltage`, midpoint clamp, `LPF_voltage`, `K_vff`, all the existing getters and setters.
- `estimate_Vq` as a feed-forward helper (no longer the sole `Vq` source).

Deleted:
- Nothing structural. `target_d_current` becomes meaningful (was hard-zero); rename `target_d_current` to `Id_target` and route it through `PID_id` if we ever want field weakening — for now leave it at 0.

---

## 5. File-level delta

### 5.1 Added

- `src/pwm_driver/stm32g4/PWMDriverSTM32G4.hpp` (currently empty stub)
- `src/pwm_driver/stm32g4/PWMDriverSTM32G4.cpp` (currently empty stub)
- `src/current_sense/stm32g4/InlineCurrentSenseSTM32G4.hpp`
- `src/current_sense/stm32g4/InlineCurrentSenseSTM32G4.cpp`
- `src/current_sense/stm32g4/stm32g4_adc_utils.{h,cpp}` (small: ADC-instance picking, clock enable, pin → ADC channel lookup, TRGO source for regular conversions). Anything that doesn't fit cleanly stays inline in `InlineCurrentSenseSTM32G4::init()`.

### 5.2 Modified

- [src/pwm_driver/PWMDriver.cpp](src/pwm_driver/PWMDriver.cpp): no API change, but `deinit()` keeps calling `stopPwmHardware` + `cleanupParams` as today; behaviour change comes from the STM32G4 overrides being no-ops.
- [src/foc_controller/FOCController.hpp](src/foc_controller/FOCController.hpp) / [.cpp](src/foc_controller/FOCController.cpp): constructor signature gains a current-sense reference; `loop()` gains the current read + DQ + PID block; new `PID_iq` / `PID_id` members.
- [src/utils/tweakable_params.h](src/utils/tweakable_params.h): add `IQ_*` / `ID_*` constants and the `mV/A` setting for ACS712 (`ACS712_MV_PER_AMP`).
- [src/utils/constants.h](src/utils/constants.h): no pin changes; optionally rename `ADC_CS_X_1`/`_2` → `ISENSE_A_X`/`ISENSE_B_X` for clarity.
- [src/pwm_driver/stm32g4/stm32_mcu.cpp](src/pwm_driver/stm32g4/stm32_mcu.cpp): trimmed to the bookkeeping + 3PWM + helpers listed in Section 2.6; the `_configure3PWM` path gains centre-aligned counter mode and TRGO setup.

### 5.3 Removed

- [src/current_sense/CurrentSense.h](src/current_sense/CurrentSense.h), [src/current_sense/CurrentSense.cpp](src/current_sense/CurrentSense.cpp)
- [src/current_sense/InlineCurrentSense.hpp](src/current_sense/InlineCurrentSense.hpp), [src/current_sense/InlineCurrentSense.cpp](src/current_sense/InlineCurrentSense.cpp)
- [src/current_sense/hardware_api.h](src/current_sense/hardware_api.h)
- [src/current_sense/stm32g4/stm32g4_mcu.cpp](src/current_sense/stm32g4/stm32g4_mcu.cpp)
- The injected-channel and interrupt sections of [src/current_sense/stm32g4/stm32g4_hal.cpp](src/current_sense/stm32g4/stm32g4_hal.cpp) and [src/current_sense/stm32g4/stm32g4_utils.cpp](src/current_sense/stm32g4/stm32g4_utils.cpp)
- The `_configure{1,2,4,6}PWM`, `_writeDutyCycle{1,2,4,6}PWM`, `_setSinglePhaseState`, MBED-only branches in [src/pwm_driver/stm32g4/stm32_mcu.cpp](src/pwm_driver/stm32g4/stm32_mcu.cpp)
- Once the STM32G4 `main.cpp` is real, `_stm32g4_main_stub.cpp` (lives in the main repo, not this worktree)

---

## 6. Bring-up sequence

Strict order; do not skip steps.

1. **Skeleton compile.** `PWMDriverSTM32G4` and `InlineCurrentSenseSTM32G4` headers/empty methods compile under `env:stm32g4` in [platformio.ini](platformio.ini). The existing compile-check stub already forces the vtable to emit.
2. **PWM only, one motor.** Wire `configure3PWM` + `write3PWM` + zero-output `stopPwmHardware`. On the scope, with a static duty cycle written via a serial command, confirm at the DRV8313 INx pins:
   - 25 kHz ± 0.5 %
   - centre-aligned (rising edges of the three phases coincide, not offset)
   - duty resolves to expected value with no glitch on `enable`/`disable`.
3. **PWM, both motors.** Repeat with both motors instantiated. Check timer instances chosen for motor Y do not overlap motor X. If they do, fail loudly at init — that is a hardware/PinMap problem, not something to paper over.
4. **ADC raw, untriggered.** Temporarily put the ADC in software-start mode and dump raw codes to serial. Confirm `pinA`/`pinB` bias sits near `2048` (1.65 V) when motors are off, and shifts when a known current is forced through a phase.
5. **ADC triggered by TRGO + DMA.** Switch to the design path (Section 3.2). Re-dump `sample_buf` over serial. Confirm the values match the untriggered version under the same conditions, and that the sampling rate matches the PWM period (or PWM-period / 2 if RCR is not configured).
6. **Offset calibration.** `pwm_driver.init()` already calls `setPwm(0,0,0)` post-init through the FOC controller's startup. With motors enabled at zero voltage, run `calibrateOffsets()`. `offset_ia` / `offset_ib` should be `~1.65 V`.
7. **Sign + gain check.** Set `target_q_current = +0.3 A`, spin slowly by hand, log `Iq_meas`. It should be positive. Flip `gain_a` and/or `gain_b` signs until polarity is correct. Repeat for `Id_meas` (expect ~0 at slow rotation).
8. **Close `Iq` loop.** Set `IQ_P` very low (~0.05 V/A), `IQ_I = IQ_D = 0`. Confirm a small step in `target_q_current` produces a tracking `Vq` and a settled `Iq_meas`. Add `I` term once `P` is stable.
9. **Close `Id` loop.** Same as above for `Id`, target = 0.
10. **Both motors live, full FOC.** Run the full scan path. Watch for `Iq` oscillation — if present, lower `IQ_P`, increase `LPF_voltage` tau slightly, or sample the FOC loop more slowly than the ADC trigger.

If step 5 or step 8 fails, the failure mode is usually a TRGO source mis-pick (master timer with no TRGO mapping) or a DMA stream collision with another peripheral. Both are diagnosable from the HAL return codes; do not push through with software-start as a fallback.

---

## 7. Risk register

| Risk | Likelihood | Mitigation |
| --- | --- | --- |
| Master timer chosen by `stm32_alignTimers` has no TRGO source compatible with the chosen ADC | low–medium | Validate at init: if `_timerToRegularTRGO(master)` returns `_TRGO_NOT_AVAILABLE`, log and refuse to start. Hardware fix is the only resolution. |
| DMA stream conflict with another peripheral (SPI for AS5048A, future UART DMA) | low | Use the per-ADC default DMA channel and assert no other init has claimed it. |
| Centre-aligned `TRGO_UPDATE` fires twice per period and overruns the ADC | medium | Use RCR=1 on TIM1/TIM8 to halve the rate; or use `LL_TIM_TRGO_OC4REF` aimed at mid-low-side. |
| ACS712 noise saturates current PID | medium | LPF on currents (1–2 kHz cutoff), low `IQ_P`, keep feed-forward (`estimate_Vq`) as the bulk of `Vq`. |
| Two-motor pin set forces overlapping timer instances | medium | Section 2.2 step 6 emits an error and halts at init; fall back to single-motor operation while the hardware is reworked. |
| `STM32DriverParams` lifetime mismatch causes a leak | low | Deliberate decision: driver is long-lived, `stopPwmHardware` does not free, `cleanupParams` is a no-op. Documented here so future readers don't "fix" it. |

---

## 8. Out of scope

These came up in the prior report and are deferred until the closed-loop system is running and stable:

- Cross-motor master-timer chaining (motor X and motor Y currently run with independent masters).
- Field weakening (`Id_target ≠ 0`).
- 6-PWM low-side / dead-time configuration.
- ADC oversampling.
- DRV8313 nFAULT line into a GPIO EXTI.
- Replacing `estimate_Vq` feed-forward with a measured `R` / `K_v` from a calibration routine.

---

## 9. Summary

- **PWM:** centre-aligned 3PWM via a thin `PWMDriverSTM32G4` adapter over a trimmed `_configure3PWM` / `_writeDutyCycle3PWM`. `stopPwmHardware` zeros outputs and nothing else; `cleanupParams` is empty.
- **Current sense:** regular-channel ADC, triggered by the 3PWM master timer's TRGO, DMA into a two-element buffer. `getPhaseCurrents()` is a non-blocking read of that buffer scaled to amps. The SimpleFOC Shield 3.2 hardware handles 5 V → 3.3 V.
- **FOC:** `Vq` driven by `(target_q_current − Iq_meas)` (with the resistor/Kv model as feed-forward); `Vd` driven by `(0 − Id_meas)`. Inverse Park + Clarke + midpoint clamp unchanged.
- **Scope discipline:** delete every SimpleFOC import path the new design does not reach — stepper, 1/2/4/6-PWM variants, injected ADC + interrupt handlers, `CurrentSense` / `InlineCurrentSense` base classes.
