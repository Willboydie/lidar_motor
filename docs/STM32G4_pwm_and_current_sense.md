# STM32G4 PWM and Current Sensing — Technical Reference

This document describes the STM32G4 motor-drive stack used in the **lidar_motor** project: how three-phase PWM is generated for the DRV8313, how phase currents are acquired from ACS712 Hall-effect sensors, and how the two subsystems are coupled for field-oriented control (FOC). It is written for a motor-drive engineer who already understands BLDC inverters, space-vector modulation, and synchronous sampling — and who wants the implementation detail, not a tutorial on FOC itself.

Companion material: [STM32G4_architecture_report.md](STM32G4_architecture_report.md), [STM32G4_upgrade_stages.md](STM32G4_upgrade_stages.md), [STM32G4_upgrade_plan copy.md](STM32G4_upgrade_plan%20copy.md).

---

## 1. Platform and scope

### 1.1 Hardware

| Element | Part / configuration |
|--------|----------------------|
| MCU | STM32G474 (Nucleo-G474RE), Cortex-M4 @ 170 MHz |
| Gate driver | DRV8313 (3× half-bridge, logic-level INx inputs) |
| Motor | GM2804 gimbal BLDC + AS5048A magnetic encoder (SPI) |
| Current sensors | ACS712ELCTR-05B-T (±5 A), two per motor on phase wires |
| Shield | SimpleFOC Shield 3.2 — scales ACS712 0–5 V output to 0–3.3 V at the MCU pin |
| Supply | 12 V rail (`VOLTAGE_POWER_SUPPLY`), `MOTOR_VOLTAGE_LIMIT = 12 V` |

Fixed pin assignments live in `src/utils/constants.h` under `#if defined(STM32G4xx)`:

| Signal | Motor X | Motor Y |
|--------|---------|---------|
| Phase A PWM (`N1`) | PA7 | PA5 |
| Phase B PWM (`N2`) | PC7 | PB6 |
| Phase C PWM (`N3`) | PB3 | PB4 |
| Enable (`EN`) | PA8 | PA9 |
| Current sense A | PA0 (ADC1_IN1) | PA1 (ADC1_IN2) |
| Current sense B | PA4 (ADC2_IN17) | PB0 (ADC1_IN15) |

Each motor therefore consumes **three independent timer channels** (almost certainly on **three distinct timer instances**, because the fixed pinout rarely collapses onto one advanced timer) plus **two ADC channels** on one or two ADC peripherals.

### 1.2 Operating point

- PWM frequency: **25 kHz** (`MOTOR_PWM_FREQUENCY`), clamped to 50 kHz max.
- Modulation: **centre-aligned mode 2** (`TIM_COUNTERMODE_CENTERALIGNED2`), PWM1, active-high.
- Interface to the DRV8313: **3PWM** (high-side PWM on INx; the DRV8313 integrates dead time internally). No STM32 complementary outputs or break/MOE gating are used in the active code path beyond enabling MOE on advanced timers.
- FOC loop rate: **500 Hz** (`SCAN_PATH_UPDATE_PERIOD_US = 2000 µs`). ADC sampling runs at **25 kHz** (once per PWM period), decoupled from the FOC rate.

### 1.3 Implementation status

| Component | Status |
|-----------|--------|
| `PWMDriverSTM32G4` + SimpleFOC timer core (`stm32_mcu.cpp`, `stm32_timerutils.cpp`, `stm32_searchtimers.cpp`) | **Implemented** — centre-aligned 3PWM, master/slave sync, TRGO for ADC |
| `InlineCurrentSenseSTM32G4` (ACS712 + DMA + TRGO trigger) | **Planned** — specified in upgrade plan; not yet in tree |
| Legacy SimpleFOC low-side path (`stm32g4_hal.cpp` injected ADC) | **Present but not used** for this hardware — retained as HAL reference only |

The sections below describe the **target system** as a unified design. Where the current-sense class is still planned, behaviour is taken from the upgrade plan and matches the PWM-side hooks already wired in `_configure3PWM`.

---

## 2. Software architecture

```
FOCController::loop()  (500 Hz)
    │
    ├─► Sensor (AS5048) ──► θ_mech, θ_el, ω
    │
    ├─► InlineCurrentSenseSTM32G4::getPhaseCurrents()  ──► i_a, i_b, i_c
    │       └─ volatile read of DMA-filled sample_buf[2]
    │
    ├─► Clarke + Park ──► I_d, I_q
    │
    ├─► Current / velocity / angle PIDs ──► V_d, V_q
    │
    └─► inverse Park + Clarke + midpoint clamp
            │
            ▼
        PWMDriver::setPwm(Va, Vb, Vc)
            │
            ▼
        PWMDriverSTM32G4::write3PWM(d_a, d_b, d_c)
            │
            ▼
        _writeDutyCycle3PWM  ──► stm32_setPwm  ──► TIMx.CCRy  ──► DRV8313 INx

Parallel hardware path (no CPU):

    master_timer TRGO (UPDATE, once/period)
            │
            ▼
        ADC regular sequence (2 channels, scan)
            │
            ▼
        DMA circular  ──► sample_buf[2]
```

The PWM and current paths share one resource: the **master timer's TRGO**. The PWM driver configures it; the current-sense driver consumes it. That coupling is intentional and is the heart of synchronous inline sensing on this platform.

---

## 3. PWM subsystem

### 3.1 The `PWMDriver` contract

`PWMDriver` (`src/pwm_driver/PWMDriver.hpp`) is the platform-neutral façade. The FOC stack calls only:

- `init()` — configure GPIO, invoke `configure3PWM`, assert `EN`.
- `setPwm(Va, Vb, Vc)` — clamp voltages, convert to duty cycles $d = V / V_\mathrm{supply}$, call `write3PWM`.
- `enable()` / `disable()` — GPIO on `EN`; `disable()` also zeros PWM.
- `deinit()` — zero outputs, release pins, call `stopPwmHardware` + `cleanupParams`.

Four pure virtual hooks isolate MCU specifics:

```cpp
virtual void* configure3PWM(long freq, int pinA, int pinB, int pinC) = 0;
virtual void  write3PWM(float dA, float dB, float dC, void* params) = 0;
virtual void  stopPwmHardware(void* params) = 0;
virtual void  cleanupParams(void* params) = 0;
```

`PWMDriverSTM32G4` is a **thin adapter** (~30 lines) that delegates to the imported SimpleFOC STM32 driver core. It adds `getMasterTimerInstance()` / `getMasterTimerHandle()` so current sense and bench tests can inspect which timer emits TRGO.

**Design choice — long-lived drivers:** `stopPwmHardware` writes `(0, 0, 0)` duty only. Timers keep running; TRGO keeps firing; ADC DMA is uninterrupted. `cleanupParams` is a no-op. `STM32DriverParams` lives in the global `motorsUsed[]` registry for the lifetime of the firmware. This mirrors the Teensy driver's "safe idle" semantics without tearing down timer state that other subsystems depend on.

### 3.2 Timer allocation

STM32 pins map to `(TIM instance, channel, AF)` through stm32duino's `PinMap_TIM[]`. For three arbitrary GPIOs the allocator must pick one valid entry per pin such that:

1. No channel is already claimed by another motor (`stm32_isChannelUsed`).
2. No duplicate `(timer, channel)` within the set.
3. Inverted/complementary map entries are rejected for 3PWM.
4. The combination **minimises distinct timer count** (lower score is better).

`stm32_findBestTimerCombination()` performs depth-first search over each pin's PinMap slice (`_stm32_findIndexOfFirstPinMapEntry` … `Last`). `_stm32_scoreCombination()` returns `-2…-4` for invalid sets, or a non-negative score equal to the number of distinct timer peripherals used.

For the fixed X-motor pins `(PA7, PC7, PB3)` the search typically yields **three timers** — not one TIM1 with CH1/CH2/CH3, because those pins do not all alternate-function to the same instance. Expect TIM1/TIM8-class advanced timers on some phases and general-purpose timers on others; the exact mapping is determined at init time from `PinMap_TIM`, not hard-coded.

**Dual-motor hazard:** `stm32_alignTimers()` warns (debug build) if a timer handle is already registered in `motorsUsed[]` for another motor. Overlapping timer instances between motors X and Y are a **bring-up failure**, not something the allocator resolves at runtime. All six PWM pins should be audited against PinMap before PCB sign-off.

Global bookkeeping:

- `timersUsed[]` — every distinct `TIM_HandleTypeDef*` allocated via `stm32_useTimer`.
- `motorsUsed[]` — one `STM32DriverParams*` per motor.
- `stm32_reserveTimer()` — optional exclusion list for non-motor peripherals.

### 3.3 Centre-aligned PWM generation

Each phase channel is initialised through `stm32_initPinPWM()`:

| Parameter | Value |
|-----------|-------|
| Counter mode | `TIM_COUNTERMODE_CENTERALIGNED2` |
| OC mode | `TIM_OCMODE_PWM1`, `TIM_OCPOLARITY_HIGH` |
| ARR preload | enabled |
| Repetition counter (init) | `RCR = 1` on instances that support it (advanced timers) |

**Mode 2 behaviour:** in the up-counting half-period the output is active while `CNT < CCR`; in the down-counting half it is inactive while `CNT > CCR`. Pulse centres align at the counter turnaround (count = 0 after a full up-down cycle in symmetric centre-aligned counting).

**Frequency equation** (from `stm32_setClockAndARR` and `stm32_checkTimerFrequency`):

$$
f_\mathrm{PWM} = \frac{f_\mathrm{TIM}}{(\mathrm{PSC}+1)\,(\mathrm{ARR}+1)\,2}
$$

The factor of 2 is the centre-aligned full period. `stm32_setClockAndARR` chooses `PSC` and `ARR` to hit the requested frequency with maximum resolution subject to 16-bit `ARR`. A warning is logged if effective resolution falls below 8 bits (`SIMPLEFOC_STM32_MIN_RESOLUTION = 255`).

**Duty update:** `_writeDutyCycle3PWM` computes

$$
\mathrm{CCR} = d \cdot (\mathrm{ARR}+1)
$$

and writes via `stm32_setPwm()` — a direct register poke to `TIMx_CCRy` (LL-style hot path, no HAL in the loop). If all three channels share one timer (`num_timers == 1`), update events are masked with `LL_TIM_DisableUpdateEvent` / `EnableUpdateEvent` around the three writes so CCR registers load coherently at the next update.

**Advanced timers:** when `IS_TIM_BREAK_INSTANCE`, `__HAL_TIM_MOE_ENABLE` is called during init so outputs reach the pin despite the break dead-time block on TIM1/TIM8/TIM20.

### 3.4 Multi-timer synchronisation

When `num_timers > 1`, `stm32_alignTimers()` elects a **master** and chains slaves:

1. Collect distinct timer handles.
2. Among master-capable instances (`IS_TIM_MASTER_INSTANCE` — TIM1–5, TIM8 on G4), find one that can drive an internal trigger (`ITR`) to every other timer in the set. Timers already configured with `TRGO = UPDATE` (reserved for ADC) are skipped as master candidates during alignment.
3. **Master:** `SlaveMode = DISABLED`, `TRGO = ENABLE` (counter enable signal exported).
4. **Slaves:** `TriggerInput = ITRx` from `stm32_getInternalSourceTrigger(master, slave)`, `SlaveMode = GATED`. Slaves run only while the master's enable gate is high.
5. All slaves are refreshed and resumed first; **master resumed last** so counters start together.

On STM32G4 the ITR routing is fixed per master instance (TIM1 → ITR0, TIM2 → ITR1, … TIM8 → ITR5). Unlike Teensy FlexPWM + XBAR, you cannot route arbitrary timers to arbitrary partners — the PinMap search must yield a combination whose topology is supported by the silicon trigger matrix.

If no master is found, timers start independently. Frequency still matches (verified by `stm32_checkTimerFrequency`), but **phase alignment between phases is not guaranteed** — unacceptable for symmetric ripple and for ADC trigger placement. Treat "no master timer" as a init-time defect.

The returned `TIM_HandleTypeDef*` is stored in `STM32DriverParams::master_timer`.

### 3.5 TRGO configuration for ADC triggering

After alignment, `_configure3PWM` calls `stm32_configureMasterTrgoForAdc()` on the master:

```cpp
LL_TIM_SetTriggerOutput(master, LL_TIM_TRGO_UPDATE);
LL_TIM_SetRepetitionCounter(master, 1);  // TIM1/TIM8/TIM20 only
master->EGR = TIM_EGR_UG;
```

**Why this matters:** in centre-aligned mode, an **update event fires twice per PWM period** — at both counter extremes (top and bottom of the triangle). The ADC needs **one conversion per period** at a stable electrical angle. Setting `RCR = 1` on repetition-counter timers divides the update rate by two, yielding **one TRGO per full PWM cycle**, typically aligned with the count-zero crossing (centre of the low-side interval in PWM1 centre-aligned convention).

Motor current sense then uses:

- `ExternalTrigConv = ADC_EXTERNALTRIG_Tx_TRGO` (regular-channel trigger, mapped from master timer instance),
- `ExternalTrigConvEdge = RISING`,
- scan sequence of two ranks (phase A pin, phase B pin),
- DMA circular into `sample_buf[2]`.

The legacy injected-channel path in `stm32g4_hal.cpp` (`_adc_init` + `ExternalTrigInjecConv`) solves the same problem for **low-side shunt** sensing; this project uses **regular** conversions because ACS712 sensors sit inline on the phase wire, not on the low-side FET return.

### 3.6 Enable path and safe stop

`PWMDriver::init()` sets `EN` high after successful `configure3PWM`. The DRV8313 enable is **active high** and gates all three half-bridges together.

`disable()`:

1. `setPwm(0, 0, 0)` — zero duty on all phases.
2. `digitalWrite(en, LOW)` — hardware disable.

`stopPwmHardware()` (called from `deinit`) again only zeroes duties. It does **not** stop counters, disable MOE, or free `STM32DriverParams`. Rationale: ADC triggering and timer registry state must survive disable/enable cycles used during calibration and fault recovery.

---

## 4. Current sensing subsystem

### 4.1 Sensor and analog front-end

Each motor carries **two ACS712-05B** modules, one in series with phase A and one with phase B. Phase C current is reconstructed:

$$
i_c = -(i_a + i_b)
$$

valid for a wye-connected motor with no neutral return (the GM2804 satisfies this).

**ACS712-05B key parameters** (see [ACS712_spec.txt](ACS712_spec.txt)):

| Parameter | Typical value |
|-----------|---------------|
| Sensitivity | 185 mV/A (typ at 25 °C) |
| Zero-current output | Vcc/2 = 2.5 V at 5 V supply |
| Bandwidth | 80 kHz (−3 dB), set by CF capacitor |
| Full-scale error | ±1.5 % of Ip (≈ ±75 mA at ±5 A) |

The SimpleFOC Shield 3.2 divides the 0–5 V sensor swing to **0–3.3 V at the STM32 pin**. Firmware therefore:

- Uses `adc_voltage_conv = 3.3 / 4096` for 12-bit codes.
- Calibrates **offset at the pin** (`calibrateOffsets()` with zero phase current), expecting ≈ **1.65 V** (code ≈ 2048), not 2.5 V.
- Sets `gain = 1000 / mVpA` (e.g. `1000/185 ≈ 5.41 A/V`). No additional divider factor — the shield has already absorbed it.

**Polarity:** ACS712 output increases for current flowing in the marked direction on the module. Wiring orientation and DRV8313 phase assignment determine whether `gain_a` / `gain_b` are positive or negative. Bring-up applies a known torque and flips gain sign until `I_q` correlates with `V_q` command (Stage 9 in upgrade plan).

### 4.2 Why inline Hall sensing differs from low-side shunt FOC

Classic STM32 FOC reference designs sample **low-side shunt voltage** synchronously at the centre of the PWM on-period, when the low FET is conducting and the shunt sees phase current. The imported `stm32g4_hal.cpp` implements exactly that: **injected ADC channels**, trigger from timer TRGO, sample time 2.5 cycles.

This project uses **inline** ACS712 sensors:

- They measure current whenever it flows in the phase wire, regardless of which FET is on.
- Their bandwidth (~80 kHz) and noise floor (~100 mA p-p with shield filtering) are **coarse** compared to milliohm shunts.
- PWM-synchronous sampling still helps **repeatability** (same point in the ripple envelope every period) but does not eliminate switching-frequency content the way a true zero-vector sample does.

Engineering expectation: closed-loop `I_q` regulation is viable for **limiting and damping** on a gimbal motor with sub-amp phase currents, not for high-bandwidth torque control comparable to a servo drive with discrete CSAs.

### 4.3 Target class: `InlineCurrentSenseSTM32G4`

Planned API (not yet in repository):

```cpp
class InlineCurrentSenseSTM32G4 {
public:
    InlineCurrentSenseSTM32G4(float mVpA, int pinA, int pinB, PWMDriverSTM32G4& driver);
    bool init();
    void calibrateOffsets(int rounds = 1000);
    PhaseCurrent_s getPhaseCurrents();
    DQCurrent_s    getFOCCurrents(float angle_el);
    // gain_a/b, offset_ia/ib — signed, set at cal time
private:
    PWMDriverSTM32G4& driver;       // TRGO source
    volatile uint16_t sample_buf[2];
    ADC_HandleTypeDef* adc;
    DMA_HandleTypeDef  hdma;
};
```

**`init()` sequence:**

1. Configure `pinA`, `pinB` as analog inputs (`pinmap_pinout` + `PinMap_ADC`).
2. Select an ADC instance that muxes both pins (`_findBestADCForRegularPins` — dual-channel scan on one ADC).
3. `HAL_ADC_Init`: 12-bit, scan enabled, discontinuous off, **external trigger rising**, DMA continuous, overrun overwrite, EOC = end of sequence.
4. Rank 1 → channel A, rank 2 → channel B; sample time `ADC_SAMPLETIME_2CYCLES_5` (adequate for ACS712; conversion completes well within one PWM period).
5. `HAL_ADCEx_Calibration_Start`.
6. Map `driver.getMasterTimerHandle()->Instance` to `ADC_EXTERNALTRIG_Tx_TRGO` via a `stm32_timerToRegularTRGO()` helper (analogous to existing `_timerToInjectedTRGO` in the legacy HAL).
7. `HAL_ADC_Start_DMA(adc, sample_buf, 2)`.

**No CPU in the acquisition loop.** The master timer configured in Section 3.5 fires the ADC at 25 kHz; DMA overwrites `sample_buf` continuously.

**`getPhaseCurrents()` — non-blocking read:**

```cpp
float Va = sample_buf[0] * adc_voltage_conv;
float Vb = sample_buf[1] * adc_voltage_conv;
i.a = (Va - offset_ia) * gain_a;
i.b = (Vb - offset_ib) * gain_b;
i.c = -(i.a + i.b);
```

Two volatile loads, two subtractions, two multiplies — suitable for a 500 Hz FOC loop reading the **latest** hardware sample (inherent ~0–40 µs staleness plus one PWM period of filter memory in the sensor).

**`calibrateOffsets()`:** average `sample_buf` over ~1000 iterations with `setPwm(0,0,0)` and motor enabled. Offsets are **voltages at the pin**, not amperes.

### 4.4 Clarke and Park transforms

`getFOCCurrents(angle_el)` applies the standard two-current Clarke transform (phase C implicit):

$$
i_\alpha = i_a,\quad i_\beta = \frac{1}{\sqrt{3}} i_a + \frac{2}{\sqrt{3}} i_b
$$

then Park:

$$
i_d = i_\alpha \cos\theta_\mathrm{el} + i_\beta \sin\theta_\mathrm{el},\quad
$$
i_q = i_\beta \cos\theta_\mathrm{el} - i_\alpha \sin\theta_\mathrm{el}
$$

Implementation matches `CurrentSense::getABCurrents` / `getDQCurrents` in the legacy SimpleFOC tree (`src/current_sense/CurrentSense.cpp`). The new class will embed equivalent math without inheriting SimpleFOC's driver-linkage machinery.

### 4.5 Dual-motor ADC allocation

Motor X uses PA0 + PA4; motor Y uses PA1 + PB0. On STM32G474 these map across **ADC1 and ADC2** instances. Each `InlineCurrentSenseSTM32G4` object owns its ADC + DMA stream. Requirements:

- Distinct DMA channels per ADC.
- No shared injected/regular trigger contention (each motor's TRGO comes from **its own** master timer — motors are not phase-locked to each other).
- Init must fail loudly if `HAL_ADC_Init` or `HAL_ADC_Start_DMA` returns error (stream collision with SPI DMA for AS5048 is unlikely on G474 but must be checked in the final pin/DMA map).

---

## 5. PWM–ADC timing

Centre-aligned PWM1 with three symmetric phases (ideal case):

```
Counter:  ARR ─╮     ╭─ ARR
                 ╲   ╱
                  ╲ ╱
                   0  ← TRGO (with RCR=1): once per period here
                   ↑
            sample instant (target)

Phase A PWM:  ──┐     ┌──
                └─────┘
                symmetric about counter turnaround
```

With **TRGO = UPDATE** and **RCR = 1**, the ADC trigger occurs once per PWM period at the counter update associated with the triangle bottom (implementation detail depends on centre-aligned sub-mode; bench validation compares scope on TRGO/MCO against phase PWM — Stage 4 and Stage 8 tests in upgrade plan).

**Jitter budget:** timer TRGO jitter is sub-nanosecond. ADC acquisition + DMA write adds microseconds but occurs at fixed phase relative to PWM. FOC reads the buffer asynchronously at 500 Hz — it does **not** synchronise to TRGO — so the measured current is a rolling snapshot, not an averaged interval. For a gimbal this is acceptable; for a high-speed spindle drive you would either raise FOC rate or accumulate ADC samples in the DMA half-complete callback.

**Disable semantics:** because `stopPwmHardware` does not halt the master timer, **ADC continues sampling during driver disable** at the same TRGO rate. Offsets remain valid; only the physical current changes when `EN` drops.

---

## 6. Integration with FOC

### 6.1 Voltage path (existing)

`PWMDriver::setPwm` receives phase voltages after inverse Park/Clarke and midpoint clamp in `FOCController::setPhaseVoltage`. Duties are:

$$
d_x = \mathrm{clamp}\left(\frac{V_x}{V_\mathrm{supply}}, 0, 1\right)
$$

The DRV8313 sees effective phase voltage $\approx d_x \cdot V_\mathrm{supply}$ filtered by motor inductance.

### 6.2 Current path (target)

Inside `FOCController::loop()` after velocity loop produces `target_q_current`:

```
PhaseCurrent_s i = current_sense.getPhaseCurrents();
DQCurrent_s    dq = current_sense.getFOCCurrents(electrical_angle);

iq_error = target_q_current - dq.q;
id_error = 0.0f            - dq.d;

Vq = estimate_Vq(target_q_current) + PID_iq(iq_error);   // feed-forward + correction
Vd = PID_id(id_error);

Vq = LPF(Vq);  constrain to ±MOTOR_VOLTAGE_LIMIT
Vd = constrain to ±MOTOR_VOLTAGE_LIMIT
setPhaseVoltage(Vq, Vd, angle_el);
```

`estimate_Vq` (resistor + Kv model) remains as **feed-forward** so the current loop only handles error, keeping gains small in the face of ACS712 noise.

Staged bring-up (upgrade plan): log `I_q` only → add error damping → full PI on `I_q` → add `I_d` loop.

### 6.3 Rate decoupling

| Loop | Rate | Notes |
|------|------|-------|
| PWM / TRGO / ADC | 25 kHz | hardware |
| FOC position/velocity/current | 500 Hz | `loop()` |
| AS5048 angle | SPI, polled in `loop()` | 14-bit absolute |

The current loop therefore sees **50 ADC samples per FOC tick** but uses only the latest. Optional future work: boxcar average `sample_buf` in software or DMA half-transfer ISR to reduce noise before PID.

---

## 7. Resource and risk summary

| Risk | Mitigation |
|------|------------|
| Six PWM pins require overlapping timers between motors | PinMap audit at init; `stm32_getNumMotorsUsed()` / debug print of timer instances |
| No eligible master timer for a pin combination | Init failure; hardware pin change only fix |
| TRGO fires twice per period without RCR | `stm32_configureMasterTrgoForAdc` sets RCR=1 on advanced timers; verify 25 kHz sample rate on scope |
| ACS712 noise limits current-loop gain | Low `IQ_P`, keep `estimate_Vq` feed-forward, optional current LPF |
| ADC overrun at 25 kHz | `ADC_OVR_DATA_OVERWRITTEN`; shorten scan (2 ranks) and sample time; never switch to polling in production |
| 5 V sensor on 3.3 V ADC | Shield divider — firmware uses pin volts only |
| Legacy SimpleFOC injected HAL confusion | Do not call `_adc_init` for ACS712; use regular + DMA path only |

---

## 8. Source file map

| File | Role |
|------|------|
| `src/pwm_driver/PWMDriver.{hpp,cpp}` | Platform-neutral driver API |
| `src/pwm_driver/stm32g4/PWMDriverSTM32G4.{hpp,cpp}` | STM32G4 adapter |
| `src/pwm_driver/stm32g4/stm32_mcu.{h,cpp}` | `_configure3PWM`, `_writeDutyCycle3PWM`, timer registry |
| `src/pwm_driver/stm32g4/stm32_timerutils.{h,cpp}` | ARR/PSC, align, TRGO, `stm32_setPwm` |
| `src/pwm_driver/stm32g4/stm32_searchtimers.cpp` | PinMap DFS + scoring |
| `src/current_sense/stm32g4/stm32g4_hal.cpp` | Legacy low-side injected ADC (reference only) |
| `src/current_sense/stm32g4/InlineCurrentSenseSTM32G4.*` | **Planned** ACS712 + DMA class |
| `src/utils/constants.h` | Pins, `MOTOR_PWM_FREQUENCY` |
| `src/foc_controller/FOCController.cpp` | Consumes `setPwm` and (target) current sense |

Bench tests under `src/tests/driver/` and `src/tests/current_sense/` exercise each layer in isolation per [STM32G4_upgrade_stages.md](STM32G4_upgrade_stages.md).

---

## 9. Summary

The STM32G4 drive stack separates **concerns** cleanly: `PWMDriverSTM32G4` owns timer allocation, centre-aligned 3PWM, and master-timer TRGO; `InlineCurrentSenseSTM32G4` (planned) owns ADC+DMA triggered from that TRGO; `FOCController` runs at 500 Hz and treats currents as a non-blocking buffer read. The PWM core is imported from SimpleFOC's STM32 port and already implements the hard parts — multi-timer alignment, MOE enable, and post-align TRGO reprogramming with RCR — while the current path is deliberately **not** the SimpleFOC low-side injected model, because ACS712 inline sensors on a conditioned 0–3.3 V front-end call for regular conversions, DMA, and honest pin-referred calibration.

For a motor-drive engineer joining the project: start on the scope with Stage 4 (centre-aligned + TRGO), then Stage 8 (DMA buffer vs PWM phase), then Stage 9 (Clarke/Park sign and magnitude). The mathematics is standard FOC; the platform-specific insight is that **one master timer per motor** is both the phase synchroniser for three PWM channels and the timebase for current acquisition — and that those timers are deliberately left running even when the inverter is logically disabled.
