# STM32G4 Bring-Up: PWM, Current Sense, and FOC Architecture

Revised design report for porting this project from Teensy 4.1 to **STM32G4 Nucleo** with **DRV8313**, **GM2804 + magnetic encoder**, and **ACS712** phase-current sensing. Pin assignments are fixed in `src/utils/constants.h` under `STM32G4xx`.

---

## 1. Changes from the first-pass proposal


| Topic                          | Optimization                                                                                                                                                                                                                                                                                                                                                                                          |
| ------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **PWM implementation**         | Prefer a **thin `PWMDriverSTM32G4` adapter** that calls the existing `src/pwm_driver/stm32g4/stm32_mcu.cpp` helpers (`_configure3PWM`, `_writeDutyCycle3PWM`, teardown aligned with `STM32DriverParams`) instead of re-deriving prescaler/ARR, master/slave sync, and MOE rules. That preserves `stm32_findBestTimerCombination` and `stm32_alignTimers` behavior already tuned for multi-timer 3PWM. |
| **Two-motor timer allocation** | Validate **all six phase pins in one pass**. Imported code tracks `motorsUsed[]` and warns when a timer instance is already claimed; **per-motor “works alone” does not imply two motors both work**. If the fixed pinout forces overlap, the fix is **timer reservation / different `PinMap` choices** (not available here) or **forking allocator policy**—catch this before hardware bring-up.     |
| **Current sense HAL**          | `src/current_sense/stm32g4/stm32g4_hal.cpp` targets **low-side shunt + injected ADC + driver trigger** patterns from SimpleFOC. For **ACS712 on phase wires**, a **dedicated, smaller ADC path** (regular conversions, optional software trigger) is usually **less coupling** than forcing `linkDriver` + injected sequencing unless you explicitly need PWM-synchronous sampling.                   |
| **Analog front-end**           | Treat **5 V ACS712 output → 3.3 V ADC** conditioning as a **hard requirement** in the BOM/checklist, not an aside—Firmware gains and offsets must match **the divider** at the pin.                                                                                                                                                                                                                   |
| `**FOCController` migration**  | Stage **measurement and control**: (1) log `Iq_meas` only, (2) replace `estimate_Vq` with feedforward + measured error damping, (3) optional full **cascade current loop**—avoids retuning everything at once.                                                                                                                                                                                        |


---

## 2. System overview

### 2.1 Hardware data paths

- **Angle:** AS5048 over SPI (`SENSOR_SPI_CS_X` / `_Y`, bus pins in `constants.h`).
- **Torque command:** `FOCController` → inverse Park + Clarke → three phase voltages → `PWMDriver::setPwm()` → duty cycles → **three STM32 timer channels** → DRV8313 INx → GM2804 windings.
- **Current:** Two ACS712 modules per motor → conditioned analog → `PA0`/`PA4` (motor X), `PA1`/`PB0` (motor Y) → ADC → `InlineCurrentSenseSTM32G4::getPhaseCurrents()` (or `getFOCCurrents()`).

### 2.2 Software layers (target)


| Layer                                             | Role                                                                                                                                                                          |
| ------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Application (`main`, `Interface`, `StateMachine`) | Wiring, user I/O, scan path.                                                                                                                                                  |
| `FOCController`                                   | Angle/velocity loops, `setPhaseVoltage()`, optional **current feedback** inside `loop()`.                                                                                     |
| `PWMDriver`                                       | Shared API: `init` / `deinit` / `enable` / `disable` / `setPwm` / `getVoltageLimit`; virtual hooks for hardware.                                                              |
| `PWMDriverSTM32G4`                                | STM32-specific: holds opaque `void* params` (typically `STM32DriverParams*` from imported code), implements `configure3PWM`, `write3PWM`, `stopPwmHardware`, `cleanupParams`. |
| `InlineCurrentSenseSTM32G4`                       | ADC init, offset calibration, **Volts→Amps** at the pin (including divider), `getPhaseCurrents()`, optional DQ helper.                                                        |


`FOCController` today uses `**estimate_Vq()`** (resistor + Kv model), not ADC current. New sense class feeds `**Iq` / `Id`** once you extend `loop()`.

---

## 3. PWM subsystem (DRV8313 + `PWMDriver`)

### 3.1 Contract to preserve (`PWMDriver` / Teensy parity)

`PWMDriverTeensy41` does not extend the public API; it only overrides:

- `configure3PWM(long freq, pinA, pinB, pinC)` → returns hardware params or `nullptr` on failure.
- `write3PWM(dutyA, dutyB, dutyC, params)`.
- `stopPwmHardware(params)` / `cleanupParams(params)`.

`PWMDriver::init()` (in `PWMDriver.cpp`) already sets `OUTPUT` on `n1..n3` and `en`, calls `configure3PWM`, sets enable high, and gates success on non-null `params`. `**PWMDriverSTM32G4` should match this contract** so `FOCController` stays unchanged at the type level.

### 3.2 Imported STM32 behavior worth keeping

From `stm32_mcu.cpp` / `stm32_mcu.h`:

- **Center-aligned PWM** (`TIM_COUNTERMODE_CENTERALIGNED2`) for cleaner ripple vs. edge-aligned.
- **Shared frequency** via `stm32_setClockAndARR` + `stm32_checkTimerFrequency` across the handles used for one motor.
- **Advanced timers:** outputs gated by **MOE** where applicable.
- **Duty update:** `_writeDutyCycle3PWM` scales float duty by `(ARR+1)` and uses `stm32_setPwm` per distinct timer/channel; it also handles the rare **single-timer, three-channel** case with `LL_TIM_DisableUpdateEvent` around updates.

### 3.3 Adapter pattern (recommended)

Implementation sketch:

- `**configure3PWM`**: return `_configure3PWM(pwm_frequency, pinA, pinB, pinC)` (or a thin wrapper that registers the motor in `motorsUsed` exactly once and maps `SIMPLEFOC_DRIVER_INIT_FAILED` to `nullptr` if you need `void*` from `PWMDriver`).
- `**write3PWM`**: call `_writeDutyCycle3PWM(dutyA, dutyB, dutyC, params)`.
- `**stopPwmHardware` / `cleanupParams`**: stop counters, disable channels, `delete` / free policy consistent with `STM32DriverParams` allocation in `_configure3PWM` (today `**new STM32DriverParams**` and HAL handles managed globally in `timersUsed`; review for leaks when **deinit** runs—may need mirrored cleanup from SimpleFOC’s `stop` path or a small patch so Teensy-like lifecycle holds).

This keeps **one** implementation of timer scoring (`stm32_searchtimers.cpp`) and alignment (`stm32_alignTimers` in `stm32_timerutils.cpp`).

---

## 4. Timer architecture (multi-timer 3PWM)

### 4.1 Why “different timers per motor” is expected

For motor X, fixed pins are `**PA7`, `PC7`, `PB3`**; for motor Y, `**PA5`, `PB6`, `PB4`**. On STM32G4, each pin selects a **timer instance + channel** via alternate function. A single advanced timer with three independent channels on those exact pins is **unlikely**; the practical outcome is **three timer peripherals per motor** (order and instance IDs come from **stm32duino `PinMap_TIM`**, not from this repo).

`_configure3PWM()` already:

1. Calls `stm32_findBestTimerCombination(3, pins, pinTimers)`.
2. Initializes **three** `TIM_HandleTypeDef*`.
3. Calls `stm32_alignTimers(timers, 3)` and stores `**master_timer`** on the result.

### 4.2 Synchronization (`stm32_alignTimers`)

For STM32G4, the utility selects a **master** instance (when possible) that can export an internal trigger; slaves use **slave mode** so enable/start tracks the master—**Reducing phase drift** between timers compared to starting them manually.

**Implications:**

- **Scope:** Alignment is **per motor** at init time. It does **not** automatically align **motor 1 vs motor 2** unless you deliberately chain masters (not required for independent FOC axes).
- **Debugging:** If alignment warns (“no master timer”), waveforms may still run at the same **frequency** (ARR matching) but with **arbitrary phase**—bad for inter-phase symmetry; fix master selection or pin/timer choice (here, pins are fixed—verify combination score and master eligibility on your **exact** G4 part).

### 4.3 Dual-motor allocation risk

`stm32_alignTimers` compares new timers against `**motorsUsed`**. Overlapping `TIM_TypeDef*` between motors can be flagged or lead to subtle bugs. Before PCB sign-off, run a static check (script or spreadsheet) listing `**PinMap_TIM` resolutions for all six pins** and assert **six distinct timer instances** (or document intentional sharing and change policy).

---

## 5. Current sense (ACS712 + `InlineCurrentSenseSTM32G4`)

### 5.1 Sensor and scaling

Per `docs/ACS712_spec.txt` (±5 A “05B” typical):

- Sensitivity ~**185 mV/A** (use min/typ/max for your grade).
- Zero-current output ~**Vcc/2** on a **5 V** module supply.
- Error and noise dominate **sub-amp** resolutions; still useful for **coarse Iq feedback**, logging, and **fault/limiting**.

Firmware must use:

I = \frac{V_{\mathrm{ADCpin}} - V_{\mathrm{offset}}}{(\mathrm{mV/A}) \times 10^{-3}} \times \text{(if voltage divider: divide by gain)}

`**offset`** from `**calibrateOffsets()`** at zero current matches **the pin**, not the raw ACS712 pin pre-divider.

### 5.2 Two sensors per motor → three phase currents

Standard reconstruction with **phase A and phase B** measured:

- i_a from sensor 1, i_b from sensor 2, **i_c = -(i_a + i_b)** (assuming no significant neutral current).

If wiring swaps polarity, use **sign flips** (`gain_a = -gain_a`) or swap pin definitions—same as SimpleFOC `driverAlign` philosophy.

### 5.3 STM32G4 ADC design choice

**Option A — Minimal (recommended first):**

- Single ADC or **ADC1+ADC2 interleaved** only if needed for sample rate.
- **Regular channels**, **software start** in `getPhaseCurrents()`, **sequential** reads of the two (or three) channels.
- Pros: simple, no FOCDriver coupling. Cons: **non-deterministic** beat vs 25 kHz PWM unless you add later synchronization.

**Option B — PWM-synchronous:**

- Trigger from `**master_timer`** TRGO / **compare** event; **DMA** into a small buffer.
- Pros: stable sampling phase. Cons: more bring-up; benefit is **moderate** for ACS712 (bandwidth and noise dominate).

**Imported `stm32g4_hal.cpp`** is a strong reference for **HAL init patterns** but is **angled at injected + driver params**; for ACS712, **copy only the pieces you need** (clock, calibration, channel config, read path) to avoid pretending you are doing true low-side FOC.

### 5.4 API surface for `FOCController`

SimpleFOC names `**getPhaseCurrents()`** as the primitive; `**getFOCCurrents(float angle_el)`** applies Clarke/Park (`CurrentSense.cpp` in tree).

Suggested API for your class:

- `PhaseCurrent_s getPhaseCurrents();`
- `PhaseCurrent_s getCurrents() { return getPhaseCurrents(); }` — alias for readability in `loop()`.
- Optionally `DQCurrent_s getFOCCurrents(float angle_el);` — thin forwarder to existing math.

---

## 6. Integrating current into `FOCController::loop()`

Current flow (`FOCController.cpp`): sensor → angles → PIDs → `target_q_current` → `**Vq = estimate_Vq()`** → LPF → `setPhaseVoltage()`.

**Staged integration:**

1. **Passive:** call `getPhaseCurrents()` / `getFOCCurrents()`, store in `Iq`/`Id`, stream on serial—**no control change**.
2. **Damping:** `Vq = estimate_Vq(target_q) + K_{err}(target_q - Iq_meas)` with small K_{err}.
3. **Cascade:** velocity PI → **Iq target**; new **current PI** → **Vq**; retune gains; consider **decimation** of current loop if ADC noise is high.

This avoids destabilizing the gimbal while verifying offsets and sign.

---

## 7. Build / bring-up checklist

1. **PlatformIO `env:stm32g4`** (see `platformio.ini`): maintain `src/_stm32g4_main_stub.cpp` or equivalent until full `main` links; it already forces `PWMDriverSTM32G4` to compile.
2. **PinMap audit:** all six PWM pins → **timer instances** unique across motors where required.
3. **Scope:** one motor, verify **25 kHz**, **center alignment**, symmetric dead-time of **zero** (3PWM high-side drive) as expected for DRV8313 inputs.
4. **ACS712:** verify **ADC voltage at rest** ≈ expected bias **at the MCU pin**; run offset calibration.
5. **Sign test:** slow rotation / known torque → **Iq_meas** correlates with command.
6. **FOC:** enable staged current feedback (Section 6).

---

## 8. Risks and limits


| Risk                            | Mitigation                                                                                                                                                |
| ------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **ACS712 5 V vs 3.3 V ADC**     | Resistor divider + tolerance in gain; optionally **buffer** for low impedance.                                                                            |
| **Noise / error vs FOC**        | LPF on currents; slower current loop; keep **voltage model** as baseline.                                                                                 |
| **Timer sharing across motors** | Up-front PinMap audit; adjust only if pins were flexible (they are not).                                                                                  |
| **Imported HAL lifecycle**      | Ensure `deinit()` on `PWMDriverSTM32G4` matches how `timersUsed` / `motorsUsed` **release** handles—today’s SimpleFOC port may assume long-lived drivers. |


---

## 9. Minor `constants.h` note

Under `STM32G4xx`, `**EN_Y`** is commented `// D8` while `**EN_X`** is also `D8` in the comment block—likely silkscreen copy-paste; **GPIO macros `PA8` / `PA9` are the meaningful definitions**.

---

## 10. Summary

- `**PWMDriverSTM32G4`** should **mirror Teensy at the `PWMDriver` virtual interface** and **delegate** to proven `_configure3PWM` / `_writeDutyCycle3PWM` logic unless there is a strong reason to fork.
- **Multi-timer 3PWM + `stm32_alignTimers`** is the correct mental model for **fixed, non-collapsible pin choices** on STM32G4.
- `**InlineCurrentSenseSTM32G4`** should prioritize a **clean ACS712 + ADC** path with honest scaling; **PWM-sync** is optional polish.
- `**FOCController`** should adopt currents in **stages** to manage noise and retuning risk.

This document supersedes the earlier chat-only proposal and incorporates the optimizations in Section 1.