# Lecture 7 — Hardware PWM for 3-Phase BLDC Motors

### *A Tale of Two Microcontrollers: Teensy 4.1 vs STM32G4*

---

## Learning objectives

By the end of this lecture you will be able to:

1. Describe the common abstraction this codebase uses to drive 3-phase BLDC motors with center-aligned PWM.
2. Explain how the **Teensy 4.1**'s NXP i.MX RT1062 implements that abstraction using its **FlexPWM** peripheral.
3. Explain how the **STM32G4** implements the same abstraction using its general-purpose **TIMx** peripherals plus the HAL and LL APIs.
4. Contrast the two approaches and reason about the trade-offs of each.

---

## Table of contents

- [I. The common abstraction](#i-the-common-abstraction)
- [II. Why center-aligned 3-phase PWM?](#ii-why-center-aligned-3-phase-pwm)
- [III. Case study 1 — Teensy 4.1](#iii-case-study-1--teensy-41)
- [IV. Case study 2 — STM32G4](#iv-case-study-2--stm32g4)
- [V. Side-by-side comparison](#v-side-by-side-comparison)
- [VI. Trade-offs and discussion](#vi-trade-offs-and-discussion)
- [VII. Recap](#vii-recap)

---

## I. The common abstraction

> *Source: [`src/pwm_driver/PWMDriver.hpp`](src/pwm_driver/PWMDriver.hpp)*

Every motor in this project is driven by an object of type `PWMDriver` — an abstract base class. The class hides the MCU-specific details behind **four pure-virtual hooks**:

```cpp
virtual void* configure3PWM(long freq, int pinA, int pinB, int pinC) = 0;
virtual void  write3PWM    (float dA, float dB, float dC, void* params) = 0;
virtual void  stopPwmHardware(void* params) = 0;
virtual void  cleanupParams(void* params) = 0;
```

The public API the rest of the firmware calls is just three methods:

| Method | What it does |
|--------|--------------|
| `init()` | configure the hardware + assert the enable pin |
| `setPwm(Va, Vb, Vc)` | voltages → duty cycles → write |
| `deinit()` | stop, tear down hardware, free params |

> **Why this matters.** The FOC controller, the scan path, the interface layer — none of them care whether the motor is on a Teensy or an STM32. They just call `setPwm()` with three voltage commands. The opaque `void* params` returned from `configure3PWM()` carries the MCU-specific hardware-handle bookkeeping (timer handles, channel indexes, etc.).

Two concrete subclasses provide the implementation:

- **`PWMDriverTeensy41`** — [`src/pwm_driver/teensy41/`](src/pwm_driver/teensy41/)
- **`PWMDriverSTM32G4`** — [`src/pwm_driver/stm32g4/`](src/pwm_driver/stm32g4/)

---

## II. Why center-aligned 3-phase PWM?

A 3-phase BLDC inverter has three half-bridges; we modulate each half-bridge's duty cycle and rely on the motor's inductance to filter the result into a smoothed voltage. Two reasons we want **center-aligned** (vs edge-aligned) PWM:

- **Symmetric switching** reduces the harmonic spectrum (good for EMC).
- All three phases share a common **midpoint sample instant**, which is the natural time to sample phase currents for FOC.

In both implementations the counter is configured to **count up to a half-period, then back down** — so the duty cycle is centered on the period midpoint, and all three phase edges move outward symmetrically from that midpoint.

For multi-motor systems we additionally want the timers driving each phase to be **phase-locked**: if their counters drift relative to each other, the "midpoint sampling" trick falls apart. Both subclasses go to some trouble to synchronise their underlying timers.

---

## III. Case study 1 — Teensy 4.1

> *NXP i.MX RT1062 · ARM Cortex-M7 @ 600 MHz*
> *Source: [`src/pwm_driver/teensy41/`](src/pwm_driver/teensy41/)*

### 3.1 Hardware: FlexPWM

The i.MX RT1062 has **four** FlexPWM peripherals (`FlexPWM1..4`), each containing **four submodules** (`0..3`). Each submodule has three output channels: **A**, **B**, and **X**.

A submodule is essentially its own complete PWM unit:

- its own counter (`INIT..VAL1`),
- its own compare registers (`VAL2/VAL3` for channel A, `VAL4/VAL5` for channel B, `VAL0` for channel X),
- its own load-OK / run bits in `MCTRL`.

### 3.2 Pin → hardware mapping

The mapping is **fixed in silicon**: pin 2 is always `FlexPWM4_2_A`, pin 3 is always `FlexPWM4_2_B`, etc. The driver hard-codes this table verbatim from Paul Stoffregen's `cores/teensy4/pwm.c`:

```cpp
const struct pwm_pin_info_struct pwm_pin_info[] = { ... };
```

`get_flexpwm(pin)`, `get_submodule(pin)`, `get_channel(pin)` just index into that table. You can fail at well-defined points — e.g. pin 10 is a QuadTimer pin (`info->type == 2`), not a FlexPWM pin, so `get_flexpwm()` returns `nullptr`.

### 3.3 Configuring center-aligned PWM

`setup_pwm_timer_submodule()` programs one submodule:

```cpp
SM[submodule].INIT  = -half_cycle;   // count from −HALF
SM[submodule].VAL1  = +half_cycle;   // … to +HALF
SM[submodule].VAL0  = 0;             // channel X compare
SM[submodule].VAL2  = -cval;         // channel A rising edge
SM[submodule].VAL3  = +cval;         // channel A falling edge
SM[submodule].VAL4  = -cval;         // channel B rising edge
SM[submodule].VAL5  = +cval;         // channel B falling edge
SM[submodule].CTRL  =  FULL | HALF | PRSC(prescale);
SM[submodule].CTRL2 =  INDEP | INIT_SEL(0 or 3) | …;
```

The counter literally counts from a **negative** number through zero to a **positive** one and back. To write a duty `d` (0..1) on channel A:

$$
\text{cval} = \tfrac{1}{2}\,\text{half\_cycle}\,(2d - 1) + \tfrac{1}{2}\,\text{half\_cycle}
$$

```cpp
VAL2 = -cval;
VAL3 = +cval;
```

so the channel-A output is high between `−cval` and `+cval`, i.e. symmetrically around the period midpoint — **that's center-aligned**.

### 3.4 Multi-submodule synchronisation — XBAR1

Three FlexPWM submodules (one per phase) won't naturally run in lockstep. The Teensy chip routes them via **XBARA1**, a programmable crossbar:

```cpp
xbar_connect(TRIG_master, EXT_SYNC_slaveA);
xbar_connect(TRIG_master, EXT_SYNC_slaveB);
```

In `setup_pwm_timer_submodule()`:

- the **master** submodule gets `TCTRL = SMTCTRL_OUT_TRIG_EN(…)` — it emits a TRIG pulse on `VAL1` match.
- the **slaves** get `CTRL2_INIT_SEL(3)` — they reinit their counter from `INIT` on each external sync edge.

> **Effectively:** pick whichever submodule is in `flexpwmC` as master, route its `TRIG` out through XBAR1 to the `EXT_SYNC` inputs of the submodules in `flexpwmA` and `flexpwmB`.

### 3.5 Cleanup

`stopPwmHardware()` walks the three `(flexpwm, submodule, channel)` triples in `Teensy41DriverParams` and:

1. disables the appropriate `PWMA_EN` / `PWMB_EN` / `PWMX_EN` bit in `OUTEN`;
2. clears the `RUN` bit in `MCTRL`.

`cleanupParams()` just `delete`s the params struct. The FlexPWM submodules themselves are stateless from C++'s point of view, so there's no allocator to track.

---

## IV. Case study 2 — STM32G4

> *ARM Cortex-M4 @ 170 MHz*
> *Source: [`src/pwm_driver/stm32g4/`](src/pwm_driver/stm32g4/)*

### 4.1 Hardware: TIMx

STM32G4 has the usual STM32 mix of timers:

| Timer  | Type | Channels |
|--------|------|----------|
| TIM1, TIM8, TIM20 | advanced control | 4 + complementary, MOE/break logic |
| TIM2…5            | general-purpose 32-bit | 4 |
| TIM6, TIM7        | basic | (no outputs) |
| TIM15, TIM16, TIM17 | limited-channel GP | 1–2 |

Each useful timer has up to 4 (sometimes 6) capture/compare channels routed to specific GPIO alternate-function pins. **Unlike FlexPWM, there is no concept of a "submodule"** — each TIM *is* one PWM unit, and you allocate channels within it.

### 4.2 Pin → hardware mapping

The mapping is **many-to-many**: a given GPIO pin can usually be routed to multiple `(timer, channel)` combinations via different alternate-function (AF) settings. stm32duino expresses this as the `PinMap_TIM[]` table from its `PeripheralPins.c`.

`PWMDriverSTM32G4::findBestTimerCombination()` picks one PinMap entry per pin. The current implementation is a naive "first match"; a smarter version would prefer combinations that share a master timer across pins.

### 4.3 Configuring center-aligned PWM

`PWMDriverSTM32G4::initTimerHandle()` initialises a `TIM_HandleTypeDef` in-place (no heap allocation — the storage lives in the params struct's `timers_owned[3]` array), configures it for center-aligned mode 2, 16-bit auto-reload preload, and calls `HAL_TIM_Base_Init()`. `setClockAndARR()` then computes prescaler + ARR for the requested PWM frequency.

`initChannelPWM()` configures one channel of one timer:

1. `HAL_TIM_PWM_ConfigChannel()` with `TIM_OCMODE_PWM1`, polarity active-high;
2. `pinmap_pinout()` to drive the GPIO into the right alternate function;
3. `LL_TIM_CC_EnableChannel()`;
4. If the timer has a break input (`TIM1` / `TIM8` / `TIM20`), `__HAL_TIM_MOE_ENABLE()` to unlock the outputs.

`configure3PWM()` walks the three phases, sharing a handle when consecutive phases land on the same peripheral, and then aligns the (possibly distinct) timers.

To write a duty `d` (0..1) on one channel:

```cpp
CCRx = d * (ARR + 1);
```

No sign trickery: the timer counts up to `ARR` then back down to `0`, and the channel is high whenever the counter is below `CCRx` (with `PWM1` mode).

### 4.4 Multi-timer synchronisation — TRGO / ITR

When more than one TIM is involved, `alignTimers()` elects a master:

- The candidate must be a master-capable instance (`IS_TIM_MASTER_INSTANCE` — on G4 this is `TIM1..5` and `TIM8`).
- For every other (slave) timer, `getInternalSourceTrigger()` must return a non-(−1) trigger ID. On G4 this is a one-line lookup table:

  | Master | Trigger output |
  |--------|----------------|
  | TIM1   | `LL_TIM_TS_ITR0` |
  | TIM2   | `LL_TIM_TS_ITR1` |
  | TIM3   | `LL_TIM_TS_ITR2` |
  | TIM4   | `LL_TIM_TS_ITR3` |
  | TIM5   | `LL_TIM_TS_ITR4` |
  | TIM8   | `LL_TIM_TS_ITR5` |

  i.e. each potential master broadcasts on its own ITR line.

Once a master is chosen:

- **Master:** `SlaveMode = DISABLED`, `TRGO output = ENABLE`.
- **Slaves:** `TriggerInput = master's ITR`; `SlaveMode = GATED`.
- All slaves are then resumed (counters paused, awaiting gate); the **master is resumed last** — when it starts, all slaves see the gate go high and begin in lockstep.

If no master can be elected (e.g. all candidates have a slave that they can't drive), each timer just starts independently and the alignment is best-effort.

### 4.5 Cleanup

`stopPwmHardware()` walks each `(timer, llchannel)` used by this motor and calls `pauseChannel()` (`LL_TIM_CC_DisableChannel`) to drop each output, then `pauseAll()` to disable the counter(s). `cleanupParams()` `delete`s the `STM32DriverParams`.

> **Note.** Each motor owns its `TIM_HandleTypeDef` storage inline in its `STM32DriverParams` (no shared pool), so `delete` frees both the params and the handle storage in one shot.

---

## V. Side-by-side comparison

| Topic                  | Teensy 4.1                                            | STM32G4                                                |
|------------------------|-------------------------------------------------------|--------------------------------------------------------|
| Core / clock           | Cortex-M7 @ 600 MHz                                   | Cortex-M4 @ 170 MHz                                    |
| PWM peripheral         | FlexPWM (4 × 4 submodules)                            | TIMx general-purpose                                   |
| Pin → HW mapping       | 1:1 fixed lookup table (`pwm_pin_info[]`)             | Many-to-many via `PinMap_TIM` (alternate functions)    |
| Center-aligned setup   | `INIT = −half … VAL1 = +half`; sign-symmetric VAL2..5 | `TIM_COUNTERMODE_CENTERALIGNED2` + plain `ARR`         |
| Duty-cycle write       | `VAL2/3` (or `VAL4/5`) = `±cval`, then `LDOK`         | `CCRx = d · (ARR + 1)`                                 |
| Counter width          | 16-bit                                                | 16-bit (32-bit on TIM2 / TIM5)                         |
| Prescaler              | 3-bit (8 levels)                                      | 16-bit (up to 65536)                                   |
| Multi-unit sync        | XBAR1 crossbar + `EXT_SYNC` inputs                    | `TRGO` → fixed `ITRx` mux + GATED slave mode           |
| Slave start condition  | Each `EXT_SYNC` pulse re-`INIT`s the counter          | Continuous gate-high enables slave counter             |
| Layer between us & HW  | None — direct register access via `imxrt.h`           | HAL + LL (under the stm32duino umbrella)               |
| Pin allocation step    | Compile-time-ish (just index the table)               | Run-time search through `PinMap_TIM`                   |
| Multi-motor sharing    | Each pin owns its `(flexpwm, submodule, channel)` outright | Each motor owns its TIM handles inline in `STM32DriverParams`; phases on the same peripheral share a handle within one motor |
| Init-failure signal    | `nullptr`                                             | `nullptr`                                              |
| Cleanup completeness   | Full — each pin's submodule torn down                 | Full — channels disabled, counters paused, all `TIM` storage freed with the params struct |
| Code organisation      | Two files (`.hpp` + `.cpp`)                           | Two files (`.hpp` + `.cpp`)                            |

---

## VI. Trade-offs and discussion

### 6.1 Why is the Teensy code simpler?

The Teensy has a richer PWM peripheral and a fixed pin map, so the driver collapses to *"look the pin up, twiddle the registers"*. The STM32 driver has to:

1. search the `PinMap`,
2. lazily allocate a `TIM` handle,
3. sync potentially-disparate timers across two motors,

so it grows into a small framework.

### 6.2 Why does the STM32 path use both HAL and LL?

- **HAL** is used for **one-off setup**: `HAL_TIM_Base_Init`, `HAL_TIM_PWM_ConfigChannel`, `__HAL_TIM_MOE_ENABLE` — big, slow, well-documented.
- **LL** is used for the **hot path**: `setPwm` (a single `CCRx` register write), enable/disable counter, enable/disable channel.

LL gets out of the way and lets you write to registers directly, which matters when `setPwm()` is being called from a 50–120 kHz FOC loop.

### 6.3 Why is the sync logic so different?

Both architectures support a *"one master, several slaves"* model, but the trigger-routing fabrics differ:

- On **Teensy**, XBARA1 is a fully programmable crossbar; any FlexPWM submodule's `TRIG` can be routed to any submodule's `EXT_SYNC`. Synchronisation is on a **per-edge** basis (each pulse restarts the slave counter from `INIT`).
- On **STM32**, ITR routing is **fixed in silicon** and is *per-timer* (not per-channel); you can't route `TIM1` onto a free-form destination. Instead we put the slave in **GATED slave-mode**, where the master's continuous `TRGO_ENABLE` signal acts as a "gate" letting the slave run.

### 6.4 Multi-motor implications

The Teensy code holds an entire `(FlexPWM, submodule)` pair for each phase of each motor — *2 motors × 3 phases* = up to **6** (potentially distinct) submodules, which is easy because each FlexPWM has 4 submodules and there are 4 FlexPWMs.

The STM32 code is more frugal: it routes through the same `TIM` handle if both motors land on the same peripheral, which it *has* to do because there are far fewer timers than there are FlexPWM submodules.

---

## VII. Recap

- The `PWMDriver` abstraction lets the rest of the firmware ignore the MCU. Two concrete subclasses fill it in.
- **Teensy 4.1** → FlexPWM submodules, hardcoded pin map, XBAR1 for timer sync, raw register pokes (no HAL).
- **STM32G4** → general-purpose TIMx, dynamic pin → timer search via stm32duino's `PinMap_TIM`, ITR + GATED slave-mode for sync, HAL for setup + LL for the fast path.
- Same job, profoundly different ways of getting there — **because the hardware is profoundly different**.

> *Next lecture: current sensing and the FOC loop that ultimately feeds these PWM duty cycles.*
