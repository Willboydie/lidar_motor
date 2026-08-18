# LiDAR Module — Host Integration Guide

Usage instructions and wire-protocol reference for integrating the LiDAR gimbal module
into a host system over its USB-serial link. The host is anything on the other end of
the cable: a PC, an SBC, or another MCU acting as system controller.

**Firmware sources of truth** — if this document and the code disagree, the code wins:

| Topic | File |
|-------|------|
| Command IDs, packet headers | [`src/utils/cmd_defs.h`](../src/utils/cmd_defs.h) |
| Frame parser (host → device) | [`src/parser/Parser.hpp`](../src/parser/Parser.hpp) / [`Parser.cpp`](../src/parser/Parser.cpp) |
| Command handling, telemetry structs | [`src/interface/Interface.hpp`](../src/interface/Interface.hpp) / [`Interface.cpp`](../src/interface/Interface.cpp) |
| Scan engine states / behaviour | [`src/scan_engine/ScanEngine.hpp`](../src/scan_engine/ScanEngine.hpp) / [`ScanEngine.cpp`](../src/scan_engine/ScanEngine.cpp) |
| Fault codes and error text | [`src/fault_manager/FaultManager.hpp`](../src/fault_manager/FaultManager.hpp) / [`FaultManager.cpp`](../src/fault_manager/FaultManager.cpp) |
| Enable / sleep / bootloader | [`src/system_manager/SystemManager.cpp`](../src/system_manager/SystemManager.cpp) |
| Production loop (Teensy) | [`src/_teensy_main.cpp`](../src/_teensy_main.cpp) |
| Baud, rates, limits, defaults | [`src/utils/constants.h`](../src/utils/constants.h), [`src/utils/tweakable_params.h`](../src/utils/tweakable_params.h) |

---

## 1. System overview

The module is a two-axis (X, Y) brushless-gimbal LiDAR scanner:

- Two FOC-driven gimbal motors position a Garmin LIDAR-Lite V3HP range finder.
- An onboard **ScanEngine** generates motion autonomously in two modes:
  - **Point-to-point** — S-curve (jerk-limited) moves to host-supplied targets.
  - **Coarse** — a continuous Lissajous scan pattern around a movable centre.
- Range samples are timestamp-correlated with rotor angles by the **AngleTracker**,
  so every distance reading has an associated (X, Y) pointing direction.

The host's job is: configure, enable, choose a scan mode, stream in targets or scan
parameters, and consume the telemetry stream (angles + distance) to build point clouds.

### Typical integration at a glance

```
host                                     device
 │  open serial 460800 8N1                 │
 │ ───────────────────────────────────────▶│  boots (disabled; ScanEngine off)
 │  [optional] RECONFIGURE_LDM (disabled)  │
 │  [optional] tune PID / LPF / scan params│
 │  DISABLE_ENABLE  (toggle → enabled)     │
 │  ENABLE_SCANENGINE                      │  moves to neutral pose
 │                                         │
 │  ── point-to-point ──                   │
 │  SET_POINT2POINT_TARGET (x, y)          │  S-curve move, then hold
 │        …repeat per waypoint…            │
 │                                         │
 │  ── or coarse scan ──                   │
 │  BEGIN_COARSE_SCANNING                  │  seeks neutral, then Lissajous
 │  UPDATE_COARSE_SCAN_CENTRE (x, y)       │  steer the pattern while scanning
 │  STOP_COARSE_SCANNING                   │  tapers, then seeks neutral (P2P)
 │                                         │
 │ ◀─────────────  telemetry  ──────────── │  0x55 control packets @ 1 kHz
 │ ◀────────────────────────────────────── │  0xAA status packets  @ 10 Hz
 │                                         │
 │  DISABLE_SCANENGINE                     │  returns to neutral
 │  DISABLE_ENABLE  (toggle → disabled)    │  motors released
```

---

## 2. Physical layer

| Parameter | Value |
|-----------|-------|
| Medium | USB CDC serial. Production host-protocol image is **Teensy 4.1** (`src/_teensy_main.cpp`). The STM32G4 PlatformIO env currently runs stage bench tests, not this protocol. |
| Baud | **460800** (`BAUD_RATE`; USB CDC largely ignores it, set it anyway) |
| Framing | 8 data bits, no parity, 1 stop bit (8N1) |
| Endianness | **Little-endian** for all multi-byte fields |
| Float format | IEEE 754 binary32 |

One port carries everything, in both directions:

- **Host → device:** length-prefixed binary commands (§3).
- **Device → host:** length-prefixed binary telemetry packets (§5, §6), debug text
  (§7), and fault text (§8) interleaved on the same stream. The host receiver must
  resync on packet headers and tolerate ASCII lines between packets (§9).

**Boot behaviour:** at power-up the firmware waits up to 3 s for the serial port
(`while (!Serial && millis() < 3000)`), delays 200 ms, then initialises subsystems.
`FOCController::calibrate()` is currently a no-op — there is no FOC alignment twitch.
PWM init does assert the driver EN pin, so the half-bridges are not fully inert before
the host enables. The system comes up **disabled**, in **ANGLE** control mode, with the
ScanEngine **off** and debug output **off**. Both telemetry streams start as soon as
`loop()` runs, whether or not the system is enabled.

---

## 3. Command framing (host → device)

Every command is one frame:

```
[cmd : uint8] [len_lo : uint8] [len_hi : uint8] [payload : len bytes]
```

- `len` is a little-endian `uint16` byte count of the payload only.
- Commands that take no arguments are sent with `len = 0` (3 bytes total).
- Commands that take arguments expect packed little-endian `float32`s in the payload.
- Maximum accepted payload is **16 bytes** (`CMD_PAYLOAD_MAX`). A longer frame is
  read and discarded in full, and `FAULT_PROTO_OVERFLOW` is latched.
- A payload shorter than the command requires is discarded and latches
  `FAULT_PROTO_OVERFLOW`. Extra trailing payload bytes (≤ 16 total) are ignored.
- An unknown `cmd` byte latches `FAULT_CMD_INVALID` (its declared payload is still
  consumed, so framing stays intact).

There are no checksums and **no acknowledgements**. Commands are silently applied;
confirm effects via the status packet (§6) and poll faults with `GET_ERROR_MSG` (§8).
Write each frame in a single transfer where possible — the parser is a byte-wise state
machine, and a partial frame followed by a disconnect leaves it mid-frame until the
remaining bytes arrive.

Example — one-float command `TARGET_X_SET` (`0x19`) with payload 3.6 (framing only;
this command is not a usable motion interface, §10.2):

```
19 04 00 66 66 66 40
```

Outgoing telemetry uses the same 3-byte prefix: `[header : uint8] [len : uint16 LE] …`.
The firmware writes the packed struct with `Serial.write(..., sizeof(packet))` and
stores the payload byte count (`sizeof − 3`) in `len`:

| Packet | Header | `len` field (`constants.h`) | Bytes on the wire (`sizeof`) |
|--------|--------|-----------------------------|------------------------------|
| Control | `0x55` | `CONTROL_PACKET_LEN` = **27** | **30** |
| Status | `0xAA` | `STATUS_PACKET_LEN` = **21** | **24** |

Both `len` values equal the payload after the 3-byte prefix, so `3 + len` matches
the packed size. Layouts are in §5 and §6.

---

## 4. Units, axes, and conventions

| Quantity | Unit / convention |
|----------|-------------------|
| Angles | radians, **absolute rotor mechanical angle** (not relative to neutral) |
| Angular velocity | rad/s |
| Coarse-scan frequency | rad/s angular frequency ω (pattern = A·sin(ω·t + φ)) |
| Coarse-scan phase | radians |
| LPF time constants | seconds |
| Distance | centimetres, `uint16` |

Key positions and limits (from `tweakable_params.h`):

| Constant | Value |
|----------|-------|
| Neutral pose (X, Y) | **3.54, 3.39 rad** (`DEFAULT_NEUTRAL_X_POS` / `_Y_POS`) |
| X travel limits | 2.5 – 4.2 rad |
| Y travel limits | 2.6 – 3.9 rad |
| Coarse amplitude limit (X and Y) | 0 – 0.6 rad |

All host-supplied positions (point-to-point targets, coarse centres) are validated
against these limits on the device; out-of-range requests are **silently ignored**
(a debug message is emitted if debug mode is on).

---

## 5. Control packet (`0x55`, 30 bytes, 1 kHz)

The high-rate closed-loop stream (`CONTROL_PACKET_PERIOD_US` = 1000 µs). Packed
`#pragma pack(1)` as written by `Interface::sendControlPacket()`. The device
writes all **30** bytes of `ControlPacket`. `len` equals the payload after the
prefix (27 = 30 − 3).

```
[header : uint8] [len : uint16 LE] [rest of packed struct]
```

`len` is written as `CONTROL_PACKET_LEN` (27). The packed fields are:

| Offset | Type | Field | Notes |
|--------|------|-------|-------|
| 0 | `uint8` | `header` | `0x55` |
| 1 | `uint16` | `len` | `CONTROL_PACKET_LEN` = 27 |
| 3 | `float32` | `target_angle_x` | current commanded X angle [rad] |
| 7 | `float32` | `target_angle_y` | current commanded Y angle [rad] |
| 11 | `uint16` | `distance` | latest LDM range [cm] |
| 13 | `float32` | `mechanical_angle_x` | live X rotor angle [rad] |
| 17 | `float32` | `mechanical_angle_y` | live Y rotor angle [rad] |
| 21 | `float32` | `associated_angle_x` | X angle at the last LDM sample time [rad] |
| 25 | `float32` | `associated_angle_y` | Y angle at the last LDM sample time [rad] |
| 29 | `uint8` | `reached_p2p_target` | packed `bool`: 1 when `ScanEngine::getReachedP2PTarget()` is true; 0 otherwise |

```python
CONTROL_STRUCT = struct.Struct("<BHffHffffB")   # 30 bytes, header 0x55
```

- **Point clouds:** use `(distance, associated_angle_x, associated_angle_y)`. The
  associated angles are the AngleTracker's estimate of where the gimbal was pointing
  when that range sample was taken: a windowed average of rotor-angle samples
  centred on the midpoint of the interval since the previous LDM reading — do
  **not** pair `distance` with the live `mechanical_angle_*` values.
- Between LDM samples, `distance` and the associated angles hold their last values;
  the LDM sample rate is well below 1 kHz, so consecutive control packets repeat them.
  De-duplicate on change if you only want fresh range samples.
- `target_angle_*` is `FOCController::getTargetAngle()`. The production loop calls
  `ScanEngine::updatePosition()` every angle-loop tick (~1 kHz) **regardless of
  `scan_mode`**, and that function always writes both FOC angle targets
  (`baseAngle + dx`). Direct `TARGET_X_SET` / `TARGET_Y_SET` commands are overwritten
  within one tick (§10.2).
- `reached_p2p_target` is a packed 1-byte boolean (`bool` in the struct). It starts
  **true** at boot. It is cleared (`0`) when a new point-to-point target is accepted
  (`SET_POINT2POINT_TARGET`, or any internal `setNewPoint2PointTarget`, including
  the move-to-neutral helpers). It becomes `1` when both axes' rolling angular
  error are within `TARGET_ANGLE_TOLERANCE` (0.01 rad). It is also forced `1` by
  `BEGIN_COARSE_SCANNING` (immediately) and by `establishAsPoint2PointBase()` (end
  of a coarse taper, or disable completing at neutral). Wait on this flag before
  issuing the next **waypoint**; it is not a coarse-pattern-complete flag (it sits
  at `1` for most of a coarse scan).

---

## 6. Status packet (`0xAA`, 24 bytes, 10 Hz)

Low-rate housekeeping (`STATUS_PACKET_PERIOD_MS` = 100 ms). This is the host's view
of the device state machine — since commands are unacknowledged toggles, always
confirm state transitions here.

Same length-prefixed prefix as the control packet. Packed `#pragma pack(1)` as
written by `Interface::sendStatusPacket()`. The device writes all **24** bytes of
`StatusPacket`. Here `len` equals the payload after the prefix (21 = 24 − 3).

```
[header : uint8] [len : uint16 LE] [rest of packed struct]
```

`len` is `STATUS_PACKET_LEN` (21). The packed fields are:

| Offset | Type | Field | Values |
|--------|------|-------|--------|
| 0 | `uint8` | `header` | `0xAA` |
| 1 | `uint16` | `len` | `STATUS_PACKET_LEN` = 21 |
| 3 | `uint8` | `enabled` | 0 = disabled, 1 = enabled |
| 4 | `uint8` | `scan_mode` | 0 = OFF, 1 = COARSE, 2 = POINT2POINT |
| 5 | `uint8` | `dynamic_routine` | see table below |
| 6 | `uint8` | `debug_mode` | 0 = off, 1 = on |
| 7 | `uint8` | `has_fault` | 1 if any fault is latched (§8) |
| 8 | `float32` | `coarse_centre_x` | current scan base / centre X [rad] |
| 12 | `float32` | `coarse_centre_y` | current scan base / centre Y [rad] |
| 16 | `float32` | `p2p_target_x` | current point-to-point target X [rad] |
| 20 | `float32` | `p2p_target_y` | current point-to-point target Y [rad] |

Control mode (ANGLE / VELOCITY) is **not** in this packet.

```python
STATUS_STRUCT = struct.Struct("<BHBBBBBffff")   # 24 bytes, header 0xAA
```

**`dynamic_routine`** reports the transition the ScanEngine is currently executing
(`DynamicRoutine` in `ScanEngine.hpp`). The engine is settled and ready for the next
command when this reads `NONE`. Every non-`NONE` routine that "seeks start" moves
to the **neutral pose**, not to the current position:

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `NONE` | steady state |
| 1 | `ENABLING` | moving to neutral after `ENABLE_SCANENGINE` |
| 2 | `DISABLING` | moving to neutral; `scan_mode` is POINT2POINT until arrival, then OFF |
| 3 | `SEEKING_COARSE_START` | moving to neutral; `scan_mode` stays at its previous value (usually POINT2POINT) until wind-up begins |
| 4 | `WINDING_UP_TO_COARSE` | ramping the Lissajous time-dilation toward 1.0; `scan_mode` is now COARSE. Completes in one tick if dilation is already 1.0 (typical on the first coarse scan) |
| 5 | `TAPERING_DOWN_FROM_COARSE` | ramping dilation down after `STOP_COARSE_SCANNING`; `scan_mode` still COARSE |
| 6 | `SEEKING_POINT2POINT_START` | moving to **neutral**, then idling in POINT2POINT (`scan_mode = 2`) |

---

## 7. Debug text (`0xFF` prefix)

When debug mode is on (`SET_DEBUG_MODE`, off by default), the firmware interleaves
human-readable progress messages into the stream. Each message is:

```
[0xFF] [0x04] [0x06] [ASCII text…] [\r\n]
```

The 3-byte prefix (`DEBUG_HEADER` in `Debug.hpp`) is fixed; on seeing `0xFF`, consume
the two following bytes and then read text up to the newline. Several soft rejections
(out-of-bounds targets, amplitude / centre limit violations, "enable ScanEngine first")
only emit a debug message — enable debug mode during bring-up. Wrong-mode
`STOP_COARSE_SCANNING` / `STOP_POINT2POINT_SCANNING` are silent even with debug on.

---

## 8. Faults and error text

Faults are sticky latches in the `FaultManager`; the status packet only exposes the
single `has_fault` flag. To find out *what* is latched, send `GET_ERROR_MSG` (`0x1F`):
the device writes one newline-terminated ASCII string per latched fault (see
`FaultManager::getMessage()` for the full set, e.g. `invalid command \n`,
`LDM timeout \n`) and then **clears all faults**. These strings have no binary
prefix — they arrive as bare text between telemetry packets.

Recommended host policy: when `has_fault` becomes 1, send `GET_ERROR_MSG`, log the
returned lines, and decide whether to continue.

Protocol-related fault codes you can provoke from the host side:

| Code | Name | Cause |
|------|------|-------|
| 128 | `FAULT_CMD_INVALID` | unknown command byte |
| 129 | `FAULT_CMD_REJECTED_STATE` | e.g. `RECONFIGURE_LDM` while enabled |
| 130 | `FAULT_PROTO_OVERFLOW` | payload too long (> 16 B) or shorter than the command requires |

Full fault map (codes are grouped by subsystem; strings in `FaultManager.cpp`):

| Range | Domain |
|-------|--------|
| 1–15 | System / lifecycle (init failed, not initialised, watchdog, loop overrun) |
| 16–31 / 32–47 | Encoder X / Y (init, update, SPI parity, COF, OCF, magnet, AGC) |
| 48–63 | PWM drivers (X/Y init, sync) |
| 64–95 | FOC / motion (init, NaN command, tracking error, velocity limit, e-stop) |
| 96–111 | Current sense (init, offset, overcurrent) |
| 112–127 | LDM (init, I2C, timeout, invalid range, config rejected) |
| 128–143 | Host protocol (see above) |
| 144–159 | Angle tracker overflow X / Y |

---

## 9. Receiver strategy (device → host)

The downstream byte stream interleaves four things: control packets, status packets,
debug messages, and bare fault text. Suggested parse loop:

1. Append received bytes to a buffer.
2. Inspect the first byte:
   - `0x55` or `0xAA` → wait until at least 3 bytes are present. Read `len` (uint16
     LE); it is the remaining payload. Wait for `len` more bytes (`3 + len` total).
     On this firmware that is **30 bytes** for control (`len` = 27) and **24 bytes**
     for status (`len` = 21). Optionally require those constants as a sanity check
     after unpacking. A later revision may extend the packed structs — `3 + len`
     stays correct; hardcoded 30 / 24 would not.
   - `0xFF` → skip the 3-byte debug prefix, then consume a text line to `\n`.
   - anything else → treat as bare text (fault/boot output): consume to `\n`; if no
     newline appears within a reasonable window, drop one byte and retry (resync).
3. Repeat until the buffer is exhausted.

The Python reference below unpacks the current 30 / 24 byte layouts (`3 + len`).

**Header collision caveat:** telemetry payload floats can legitimately contain the
bytes `0x55` / `0xAA` / `0xFF`, so after a desync a naive scanner can lock onto a
false header. There are no checksums; robustness comes from (a) checking `len`
against the known constants (27 / 21), (b) requiring the packed size to match
`3 + len`, (c) the steady 1 kHz / 10 Hz cadence, and (d) sanity checks on decoded
values (angles within 2.5–4.2 / 2.6–3.9 rad, plausible distances). Requiring two
consecutive size-consistent packets before trusting a resync works well in practice.

---

## 10. Command reference (host → device)

All IDs from `cmd_defs.h`. "—" payload means send with `len = 0`.
Payload floats are little-endian `float32`.

### 10.1 System state and lifecycle

| ID | Command | Payload | Behaviour |
|----|---------|---------|-----------|
| `0x01` | `DISABLE_ENABLE` | — | **Toggle** system enable. Enable powers FOC, encoders and LDM; requires successful boot init, else latches `FAULT_SYS_NOT_INITIALISED`. Disable releases the motors. |
| `0x02` | `SET_DEBUG_MODE` | — | **Toggle** debug text output (§7). |
| `0x03` | `TOGGLE_CONTROL_MODE` | — | **Toggle** ANGLE ↔ VELOCITY flags on both FOC controllers and the state machine. The system boots in ANGLE. Control mode is **not** reported in the status packet. The production Teensy loop always runs `angle_loop()`, which overwrites `target_velocity` every tick, so VELOCITY mode is not an independent motion interface on this firmware. The ScanEngine always writes angle targets (§10.2). |
| `0x1B` | `SLEEP` | — | If not initialised, latches `FAULT_SYS_NOT_INITIALISED`. Otherwise disables the drive (if enabled), then halts the CPU (WFI). |
| `0x1C` | `WAKE` | — | STM32: resumes SysTick. Teensy (production): no-op — WFI already returned with clocks intact. Does **not** re-enable — send `DISABLE_ENABLE` again. |
| `0x1D` | `RESTART` | — | `SLEEP` followed by `WAKE`. |
| `0x1E` | `ENTER_BOOTLOADER` | — | Disables the drive and jumps to the MCU bootloader (Teensy reboot / STM32 DFU). The serial link drops. |
| `0x1F` | `GET_ERROR_MSG` | — | Writes one ASCII line per latched fault, then clears all faults (§8). |

Because `DISABLE_ENABLE`, `SET_DEBUG_MODE` and `TOGGLE_CONTROL_MODE` are toggles with
no acknowledgement, drive them state-aware: check `enabled` / `debug_mode` in the
status packet and only send the toggle if the current state differs from the desired
one.

### 10.2 Direct motion targets

| ID | Command | Payload | Behaviour |
|----|---------|---------|-----------|
| `0x19` | `TARGET_X_SET` | 1 float | ANGLE mode: writes X `target_angle` [rad]. VELOCITY mode: writes X `target_velocity` [rad/s]. Not range-checked. |
| `0x1A` | `TARGET_Y_SET` | 1 float | Same for Y. |

These are **not** a usable motion interface on the production firmware. The Teensy
loop calls `ScanEngine::updatePosition()` every angle-loop tick even when
`scan_mode = OFF`, and that function always executes `setTargetAngle(baseAngle + dx)`
on both axes (initial `baseAngle` is the neutral pose, `dx = 0`). An ANGLE-mode
`TARGET_*_SET` is overwritten within ~1 ms. Prefer `SET_POINT2POINT_TARGET` with the
ScanEngine on.

### 10.3 ScanEngine

| ID | Command | Payload | Behaviour |
|----|---------|---------|-----------|
| `0x21` | `ENABLE_SCANENGINE` | — | If already on, debug "already on" and return. Otherwise sets `scan_mode = POINT2POINT` immediately and `dynamic_routine = ENABLING`, then S-curves to the neutral pose. |
| `0x22` | `DISABLE_SCANENGINE` | — | If already off, debug "already off" and return. Otherwise sets `scan_mode = POINT2POINT` immediately (no coarse taper) and `dynamic_routine = DISABLING`, S-curves to neutral, then `scan_mode = OFF`. |
| `0x2B` | `BEGIN_POINT2POINT_SCANNING` | — | Rejected (debug msg only) if the engine is off. Otherwise sets `scan_mode = POINT2POINT` immediately (no coarse taper) and `dynamic_routine = SEEKING_POINT2POINT_START`, which **moves to neutral**. |
| `0x2D` | `SET_POINT2POINT_TARGET` | 2 floats (X, Y) [rad] | If either coordinate is outside the travel limits (§4), ignored (debug msg). Otherwise rebases from the current mechanical angles, starts an S-curve to the absolute target, and clears `reached_p2p_target`. The S-curve is followed only while `scan_mode = POINT2POINT`; the rebase still updates `baseAngle` (the coarse centre) if issued in COARSE. No engine-on check. Issue the next waypoint after `reached_p2p_target` is `1`. |
| `0x2C` | `STOP_POINT2POINT_SCANNING` | — | No-op (silent) if not in POINT2POINT. Otherwise `SEEKING_POINT2POINT_START`: **moves to neutral**, then idles. |
| `0x23` | `BEGIN_COARSE_SCANNING` | — | Rejected (debug msg only) if the engine is off. Otherwise `SEEKING_COARSE_START`, resets Lissajous time, forces `reached_p2p_target = 1`. Moves to neutral, then `WINDING_UP_TO_COARSE`. Dilation is not reset: if it is already 1.0 (boot / no prior taper), wind-up completes on the first tick and status @ 10 Hz may skip routine 4. |
| `0x2E` | `UPDATE_COARSE_SCAN_CENTRE` | 2 floats (X, Y) [rad] | Only valid in COARSE mode (otherwise debug reject). Rejected if `centre ± amplitude` would leave the travel limits. |
| `0x2A` | `STOP_COARSE_SCANNING` | — | No-op (silent) if not in COARSE. Otherwise tapers dilation (`dynamic_routine` 5), then switches to POINT2POINT and **seeks neutral** (routine 6). |

### 10.4 Coarse-scan pattern parameters

The coarse pattern is a Lissajous figure: `offset = amplitude · sin(ω·t + phase)` per
axis, superimposed on the scan centre. Parameters may be set at any time; they take
effect on the next ScanEngine position tick (defaults: ω_x = 1.5, ω_y = 3.0 rad/s,
amplitude 0.2 rad, phase 0). Frequency and phase are not range-checked.

| ID | Command | Payload | Constraint |
|----|---------|---------|-----------|
| `0x24` | `X_AMPLITUDE_SET` | 1 float [rad] | 0 – 0.6; out-of-range ignored |
| `0x25` | `Y_AMPLITUDE_SET` | 1 float [rad] | 0 – 0.6; out-of-range ignored |
| `0x26` | `X_FREQUENCY_SET` | 1 float [rad/s] | angular frequency ω |
| `0x27` | `Y_FREQUENCY_SET` | 1 float [rad/s] | angular frequency ω |
| `0x28` | `X_PHASE_SET` | 1 float [rad] | |
| `0x29` | `Y_PHASE_SET` | 1 float [rad] | |

### 10.5 Controller tuning (usually not needed for integration)

Sensible defaults are compiled in (`tweakable_params.h`). Exposed for live tuning:

| ID | Command | ID | Command |
|----|---------|----|---------|
| `0x08` | `VELOCITY_P_X_SET` | `0x0E` | `ANGLE_P_X_SET` |
| `0x09` | `VELOCITY_I_X_SET` | `0x0F` | `ANGLE_I_X_SET` |
| `0x0A` | `VELOCITY_D_X_SET` | `0x10` | `ANGLE_D_X_SET` |
| `0x0B` | `VELOCITY_P_Y_SET` | `0x11` | `ANGLE_P_Y_SET` |
| `0x0C` | `VELOCITY_I_Y_SET` | `0x12` | `ANGLE_I_Y_SET` |
| `0x0D` | `VELOCITY_D_Y_SET` | `0x13` | `ANGLE_D_Y_SET` |

All take one float. The following apply to **both** axes at once (one float each):

| ID | Command | Unit |
|----|---------|------|
| `0x14` | `VELOCITY_LPF_TAU_SET` | s |
| `0x15` | `ANGLE_LPF_TAU_SET` | s |
| `0x16` | `FOC_VOLTAGE_LPF_TAU_SET` | s |
| `0x17` | `VELOCITY_FF_SET` | feed-forward gain |
| `0x18` | `ACCELERATION_FF_SET` | feed-forward gain |

### 10.6 LDM (rangefinder) configuration

| ID | Command | Payload | Behaviour |
|----|---------|---------|-----------|
| `0x20` | `RECONFIGURE_LDM` | 1 float (preset index) | Only while the system is **disabled**, else `FAULT_CMD_REJECTED_STATE`. The float is truncated to an integer preset index. |

Garmin V3HP presets (`GarminV3HP::configure()`):

| Index | Mode |
|-------|------|
| 0 | Default / balanced (boot default) |
| 1 | Short range, high speed |
| 2 | Default range, higher speed at short range (quick termination, less accuracy) |
| 3 | Maximum range |
| 4 | High sensitivity (more false positives) |
| 5 | Low sensitivity (fewer false positives) |
| 6 | Short range, high speed, higher error |

---

## 11. Integration recipes

### 11.1 Point-to-point survey (waypoint scanning)

1. Open the port; wait for boot text / first `0xAA` packet.
2. If `debug_mode = 0` and you want soft-rejection visibility: send `SET_DEBUG_MODE`.
3. If `enabled = 0`: send `DISABLE_ENABLE`; confirm `enabled = 1` in the next status
   packet. If it stays 0, poll `GET_ERROR_MSG`.
4. Send `ENABLE_SCANENGINE`; wait for `scan_mode = 2` and `dynamic_routine = 0`
   (the gimbal is now at neutral, 3.54 / 3.39 rad).
5. For each waypoint: send `SET_POINT2POINT_TARGET (x, y)` with both coordinates
   inside the travel limits; wait until `reached_p2p_target = 1` in the control
   packet; dwell as needed while logging
   `(distance, associated_angle_x, associated_angle_y)` tuples.
6. Shut down: `DISABLE_SCANENGINE`, wait for `scan_mode = 0`, then `DISABLE_ENABLE`.

### 11.2 Continuous coarse scan (area sweep)

1. Steps 1–4 as above.
2. Optionally set amplitudes / frequencies / phases (§10.4) — choose amplitudes so
   that `centre ± amplitude` stays inside the travel limits, or later centre updates
   will be rejected.
3. Send `BEGIN_COARSE_SCANNING`. Watch `dynamic_routine`: 3 (seeking start; `scan_mode`
   still POINT2POINT) → 4 (winding up; `scan_mode` = COARSE) → 0 (full pattern).
   Routine 4 may last one tick on the first coarse scan (dilation starts at 1.0) and
   can be missed at 10 Hz. Log point-cloud tuples throughout.
4. To steer the sweep, send `UPDATE_COARSE_SCAN_CENTRE (x, y)` at any time while in
   COARSE mode.
5. Send `STOP_COARSE_SCANNING`. Watch `dynamic_routine`: 5 (taper, still COARSE) →
   6 (seek neutral in POINT2POINT) → 0. The gimbal ends at the neutral pose, not at
   the last Lissajous point.

### 11.3 Python reference implementation

```python
import struct
import serial

CMD = {
    "DISABLE_ENABLE": 0x01, "SET_DEBUG_MODE": 0x02, "TOGGLE_CONTROL_MODE": 0x03,
    "TARGET_X_SET": 0x19, "TARGET_Y_SET": 0x1A,
    "SLEEP": 0x1B, "WAKE": 0x1C, "RESTART": 0x1D, "ENTER_BOOTLOADER": 0x1E,
    "GET_ERROR_MSG": 0x1F, "RECONFIGURE_LDM": 0x20,
    "ENABLE_SCANENGINE": 0x21, "DISABLE_SCANENGINE": 0x22,
    "BEGIN_COARSE_SCANNING": 0x23, "STOP_COARSE_SCANNING": 0x2A,
    "X_AMPLITUDE_SET": 0x24, "Y_AMPLITUDE_SET": 0x25,
    "X_FREQUENCY_SET": 0x26, "Y_FREQUENCY_SET": 0x27,
    "X_PHASE_SET": 0x28, "Y_PHASE_SET": 0x29,
    "BEGIN_POINT2POINT_SCANNING": 0x2B, "STOP_POINT2POINT_SCANNING": 0x2C,
    "SET_POINT2POINT_TARGET": 0x2D, "UPDATE_COARSE_SCAN_CENTRE": 0x2E,
}

CONTROL = struct.Struct("<BHffHffffB")  # 30 B packed ControlPacket (3 + len, len=27)
STATUS  = struct.Struct("<BHBBBBBffff") # 24 B packed StatusPacket (3 + len, len=21)
DEBUG_PREFIX = b"\xff\x04\x06"

def send(ser: serial.Serial, name: str, *args: float) -> None:
    payload = struct.pack(f"<{len(args)}f", *args) if args else b""
    ser.write(bytes([CMD[name]]) + struct.pack("<H", len(payload)) + payload)

def packets(ser: serial.Serial):
    """Yield ('control'|'status'|'debug'|'text', data) tuples."""
    buf = bytearray()
    while True:
        buf.extend(ser.read(ser.in_waiting or 1))
        while buf:
            b0 = buf[0]
            if b0 == 0x55:
                if len(buf) < CONTROL.size:
                    break
                yield ("control", CONTROL.unpack_from(buf)); del buf[:CONTROL.size]
            elif b0 == 0xAA:
                if len(buf) < STATUS.size:
                    break
                yield ("status", STATUS.unpack_from(buf)); del buf[:STATUS.size]
            elif b0 == 0xFF:
                nl = buf.find(b"\n", 3)
                if nl < 0:
                    break
                yield ("debug", buf[3:nl].decode(errors="replace").strip())
                del buf[:nl + 1]
            else:
                nl = buf.find(b"\n")
                if nl < 0:
                    if len(buf) > 256:   # no newline coming — resync
                        del buf[0]
                    break
                yield ("text", buf[:nl].decode(errors="replace").strip())
                del buf[:nl + 1]

# --- example session -------------------------------------------------------
ser = serial.Serial("COM3", 460800, timeout=0.05)

send(ser, "DISABLE_ENABLE")                       # toggle → enabled
send(ser, "ENABLE_SCANENGINE")                    # go to neutral
send(ser, "SET_POINT2POINT_TARGET", 3.7, 3.5)     # absolute rad

for kind, data in packets(ser):
    if kind == "control":
        _, _len, tx, ty, dist_cm, mx, my, ax, ay, reached_p2p = data
        point = (dist_cm, ax, ay)                 # point-cloud sample
    elif kind == "status":
        _, _len, enabled, scan_mode, routine, dbg, fault, cx, cy, px, py = data
        if fault:
            send(ser, "GET_ERROR_MSG")
    elif kind in ("debug", "text"):
        print(f"[{kind}] {data}")
```

---

## 12. Integration checklist / gotchas

- **Toggles, not setters.** `DISABLE_ENABLE`, `SET_DEBUG_MODE` and
  `TOGGLE_CONTROL_MODE` flip state. Always reconcile against the status packet
  (`enabled` / `debug_mode`) rather than assuming. Control mode is not in the
  status packet; on this firmware VELOCITY mode does not skip the angle loop.
- **No ACKs.** Verify every state-changing command through the `0xAA` stream; poll
  `GET_ERROR_MSG` when `has_fault` is set.
- **Silent rejection of bad targets.** Out-of-limit point-to-point targets, coarse
  centres and amplitudes are dropped without a fault. Enable debug mode during
  bring-up to see the rejection messages. Wrong-mode stop commands are silent even
  with debug on.
- **Angles are absolute rotor angles.** Work relative to the neutral pose
  (3.54, 3.39 rad) and stay inside X 2.5–4.2 / Y 2.6–3.9 rad.
- **PWM EN is asserted at boot** (driver `generic_init`); `calibrate()` is a no-op.
  FOC loops do not run until `DISABLE_ENABLE`. The half-bridges are not fully inert.
- **ScanEngine always writes FOC angle targets**, including when `scan_mode = OFF`.
  Do not use `TARGET_X_SET` / `TARGET_Y_SET` for motion on the production loop.
- **P2P start is always neutral.** `BEGIN_POINT2POINT_SCANNING`,
  `STOP_POINT2POINT_SCANNING`, and the end of `STOP_COARSE_SCANNING` all S-curve to
  the neutral pose. They do not hold the current position. `DISABLE_SCANENGINE`
  from COARSE also skips the taper and goes straight to a P2P move to neutral.
- **Wait for `dynamic_routine = 0`** before issuing the next scan-mode command; the
  engine overwrites `dynamic_routine` if you command mid-transition. For waypoints,
  wait for `reached_p2p_target = 1` in the control packet.
- **Telemetry `len` is the payload after the prefix.** Control is 30 bytes with
  `len = 27`; status is 24 bytes with `len = 21`. `3 + len` matches both packed
  sizes. A later revision may grow the structs — keep sizing from `3 + len`.
- **LDM reconfiguration requires the disabled state**, so do it before enabling.
- **Telemetry bandwidth:** 30 B × 1 kHz + 24 B × 10 Hz ≈ 30 kB/s — trivial for USB
  CDC, but poll your OS serial buffer frequently enough to avoid overflow drops,
  which will desync the parser (§9).
