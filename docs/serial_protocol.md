# Serial Protocol — Host Interface

This document describes the binary telemetry stream emitted by the firmware `Interface` class and the command format the host uses to control the scanner. It is the authoritative reference for host-side parsers (Python, C++, etc.).

Source of truth in firmware:

- Packet layouts: [`src/interface/Interface.hpp`](../src/interface/Interface.hpp)
- Rates and defaults: [`src/utils/tweakable_params.h`](../src/utils/tweakable_params.h)
- Production loop: [`src/main.cpp`](../src/main.cpp)

---

## 1. Physical layer

| Parameter | Value |
|-----------|-------|
| Port | USB serial (Teensy 4.1 / STM32G4) |
| Baud | **460800** (`BAUD_RATE` in `constants.h`) |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Byte order | **Little-endian** for all multi-byte fields |
| Float encoding | **IEEE 754 single precision** (4 bytes) |

The link carries **two kinds of outbound traffic** on the same port:

1. **Binary packets** — fixed-size structs written with `Serial.write()`.
2. **Text lines** — human-readable `Serial.println()` output (startup messages, command acknowledgements, errors). These are newline-terminated UTF-8 (ASCII subset in practice).

A robust host parser must treat the byte stream as opaque and **resync on packet headers** (see §5). Do not assume the port contains only binary data.

---

## 2. When packets are sent

Both telemetry packets are emitted only while the system is **enabled** (`stateMachine.enabled == true`). The host must send `DISABLE_ENABLE_CMD` (see §4) to enable the rig after calibration.

| Packet | Rate | Firmware constant | Size |
|--------|------|-------------------|------|
| Control | **500 Hz** (2 ms) | `CONTROL_PACKET_PERIOD_US` (= 2000 µs) | 43 bytes |
| Status | **10 Hz** (100 ms) | `STATUS_PACKET_PERIOD_MS` (= 100 ms) | 52 bytes |

At 500 Hz + 10 Hz the firmware transmits roughly **21.5 kB/s** of binary telemetry while enabled, before counting text lines.

**Distance field cadence:** the Garmin LDM returns measurements at roughly 300–400 Hz. The control packet is still sent at 500 Hz, but `distance`, `associated_angle_x`, and `associated_angle_y` are updated only when a new LDM sample is processed (`onLdmDistanceReady()`). Between LDM events those three fields retain their previous values.

---

## 3. Motor and coordinate conventions

| Firmware name | Axis | Role |
|---------------|------|------|
| `focController1` | **X** (theta) | First gimbal motor |
| `focController2` | **Y** (phi) | Second gimbal motor |

Angles in both packet types are in **radians**.

- **Mechanical angles** — rotor angle from the AS5048 encoder pipeline.
- **Target angles / velocities** — the values the FOC controller is currently commanded to track. In production the host streams angle setpoints via `TARGET_1_SET` / `TARGET_2_SET` (e.g. from a Lissajous path computed on the PC). Firmware `ScanPath` exists for bench tests but is not driven in the host-controlled configuration.
- **Associated angles** — best estimate of each motor's mechanical angle at the time the current `distance` sample was acquired. The firmware averages encoder samples collected during the LDM integration window, then applies a per-motor world→local transform (`worldToLocal()` in `quick_defs.h`).

Angular velocity fields are in **rad/s** (filtered estimate from the sensor pipeline).

`distance` is a **uint16** in **centimetres**, as returned by `LDM::getLatestDistance()`.

---

## 4. Host → device commands (reference)

Commands are polled by the firmware roughly every 200 ms. Each command is **5 bytes**:

```
[ cmd: uint8 ][ data: float32 ]
```

`data` is always present (4 bytes) even for toggle commands where the value is ignored.

Command IDs are defined in `Interface.hpp`. Common commands:

| Command | ID (hex) | `data` meaning |
|---------|----------|----------------|
| `DISABLE_ENABLE_CMD` | `0x01` | ignored — toggles enable |
| `SCAN_PLAY_PAUSE_CMD` | `0x02` | ignored — toggles scanning (ANGLE mode only) |
| `TOGGLE_CONTROL_MODE_CMD` | `0x03` | ignored — toggles ANGLE ↔ VELOCITY |
| `TARGET_1_SET_CMD` | `0x10` | angle or velocity target for motor X |
| `TARGET_2_SET_CMD` | `0x14` | angle or velocity target for motor Y |
| `VELOCITY_P_SET_CMD` … `ANGLE_RAMP_SET_CMD` | `0x08`–`0x0F` | PID gain / ramp (applied to both motors) |
| `FOC_VOLTAGE_LPF_TAU_SET_CMD` | `0x11` | FOC voltage LPF τ [s] |
| `VELOCITY_LPF_TAU_SET_CMD` | `0x12` | velocity LPF τ [s] |
| `ANGLE_LPF_TAU_SET_CMD` | `0x13` | angle LPF τ [s] |
| `VELOCITY_FF_SET_CMD` | `0x92` | velocity feed-forward gain [A·s/rad] |

Full command list: see `Interface.hpp`.

---

## 5. Parsing strategy

Because text and binary share the port, use a **byte buffer with header search**:

1. Append incoming bytes to a buffer.
2. Scan for `0x55` (control) or `0xAA` (status).
3. If fewer than `PACKET_SIZE` bytes remain from that offset, wait for more data.
4. Unpack the struct. On success, consume `offset + PACKET_SIZE` bytes.
5. If unpacking fails plausibility checks (optional), skip one byte and resync.

**Header collision note:** packet headers are single bytes with no checksum in V1. In theory a float payload can contain `0x55` or `0xAA`. If false sync is a problem in practice, validate timing (500 Hz / 10 Hz spacing) or wait for a second consistent packet before trusting fields.

**Distinguishing packet types:** only the first byte differs.

| Header (hex) | Packet | Size (bytes) |
|--------------|--------|--------------|
| `0x55` | Control | 43 |
| `0xAA` | Status | 52 |

No other outbound binary packet types are defined in V1.

---

## 6. Control packet (device → host)

**Purpose:** high-rate closed-loop state for motor control and point-cloud correlation.

**Header:** `0x55` (`CONTROL_PACKET_HEADER`)

**Layout** (packed, 43 bytes):

| Offset | Type | Field | Units / notes |
|--------|------|-------|---------------|
| 0 | `uint8` | `header` | `0x55` |
| 1 | `float32` | `target_angle_x` | rad |
| 5 | `float32` | `target_angle_y` | rad |
| 9 | `float32` | `target_velocity_x` | rad/s |
| 13 | `float32` | `target_velocity_y` | rad/s |
| 17 | `float32` | `mechanical_angle_x` | rad |
| 21 | `float32` | `mechanical_angle_y` | rad |
| 25 | `float32` | `mechanical_velocity_x` | rad/s |
| 29 | `float32` | `mechanical_velocity_y` | rad/s |
| 33 | `uint16` | `distance` | cm |
| 35 | `float32` | `associated_angle_x` | rad — angle at LDM sample time |
| 39 | `float32` | `associated_angle_y` | rad — angle at LDM sample time |

**C struct** (firmware):

```c
#pragma pack(push, 1)
struct ControlPacket {
    uint8_t header;
    float target_angle_x;
    float target_angle_y;
    float target_velocity_x;
    float target_velocity_y;
    float mechanical_angle_x;
    float mechanical_angle_y;
    float mechanical_velocity_x;
    float mechanical_velocity_y;
    uint16_t distance;
    float associated_angle_x;
    float associated_angle_y;
};
#pragma pack(pop)
```

**Python `struct` format:**

```python
CONTROL_HEADER = 0x55
CONTROL_SIZE = 43
CONTROL_STRUCT = struct.Struct("<BffffffffHff")
# fields = CONTROL_STRUCT.unpack_from(buf, offset)
```

**Building a point cloud:** when `distance` changes (or on every control packet if you prefer), record the tuple `(distance, associated_angle_x, associated_angle_y)`. Convert to Cartesian on the host as needed.

---

## 7. Status packet (device → host)

**Purpose:** lower-rate housekeeping — enable state, mode flags, and live tuning parameters.

**Header:** `0xAA` (`STATUS_PACKET_HEADER`)

**Layout** (packed, 52 bytes):

| Offset | Type | Field | Notes |
|--------|------|-------|-------|
| 0 | `uint8` | `header` | `0xAA` |
| 1 | `uint8` | `enabled` | `0` = disabled, non-zero = enabled |
| 2 | `uint8` | `control_mode` | see table below |
| 3 | `uint8` | `scanning` | `0` = paused, non-zero = scanning |
| 4 | `float32` | `velocity_p` | shared by both motors |
| 8 | `float32` | `velocity_i` | |
| 12 | `float32` | `velocity_d` | |
| 16 | `float32` | `velocity_ramp` | max output slew |
| 20 | `float32` | `angle_p` | |
| 24 | `float32` | `angle_i` | |
| 28 | `float32` | `angle_d` | |
| 32 | `float32` | `angle_ramp` | |
| 36 | `float32` | `velocity_lpf_tau` | sensor velocity LPF τ [s] |
| 40 | `float32` | `angle_lpf_tau` | sensor angle LPF τ [s] |
| 44 | `float32` | `foc_voltage_lpf_tau` | FOC voltage LPF τ [s] |
| 48 | `float32` | `velocity_feed_forward` | K_vff [A·s/rad] |

**`control_mode` values** (`ControlMode` enum in `StateMachine.hpp`):

| Value | Name | Meaning |
|-------|------|---------|
| `0` | `PATH` | reserved — not used in production loop |
| `1` | `ANGLE` | position control |
| `2` | `VELOCITY` | velocity control |

PID and filter fields reflect motor 1's controllers; both motors receive identical tuning when changed via serial commands.

**Python `struct` format:**

```python
STATUS_HEADER = 0xAA
STATUS_SIZE = 52
STATUS_STRUCT = struct.Struct("<BBBBffffffffffff")
```

---

## 8. Minimal Python receiver

```python
import struct
import serial

CONTROL = struct.Struct("<BffffffffHff")   # 43 bytes, header 0x55
STATUS  = struct.Struct("<BBBBffffffffffff")  # 52 bytes, header 0xAA

def parse_stream(ser: serial.Serial):
    buf = bytearray()
    while True:
        buf.extend(ser.read(ser.in_waiting or 1))
        while buf:
            if buf[0] == 0x55 and len(buf) >= CONTROL.size:
                pkt = CONTROL.unpack_from(buf)
                del buf[:CONTROL.size]
                yield ("control", pkt)
            elif buf[0] == 0xAA and len(buf) >= STATUS.size:
                pkt = STATUS.unpack_from(buf)
                del buf[:STATUS.size]
                yield ("status", pkt)
            elif buf[0] in (0x55, 0xAA):
                break  # wait for more bytes
            else:
                # text or garbage — skip until next plausible header
                try:
                    nl = buf.index(b"\n")
                    line = bytes(buf[: nl + 1])
                    del buf[: nl + 1]
                    yield ("text", line.decode("utf-8", errors="replace").strip())
                except ValueError:
                    idx55 = buf.find(b"\x55", 1)
                    idxaa = buf.find(b"\xaa", 1)
                    candidates = [i for i in (idx55, idxaa) if i != -1]
                    del buf[: min(candidates) if candidates else 1]
```

Open the port at 460800 baud. Expect text lines during boot and after commands even when binary streaming is active.

---

## 9. V1 limitations and planned fields

V1 intentionally omits fields that the firmware cannot yet populate reliably:

| Planned field | Packet | Status in V1 |
|---------------|--------|--------------|
| LDM acquisition rate | Control | **Not sent** — rate not exposed by driver |
| System health | Status | **Not sent** |
| Error codes / messages | Status | **Not sent** — only text `Serial.println` lines |

Future revisions may extend `StatusPacket` or add new headers. Host parsers should key off header byte and packet size rather than assuming a fixed stream length.

---

## 10. Migration from legacy `DataStream`

Older firmware (and `UI/motor_control_ui.py`) used a 19-byte `DataPacket` with header `0x55`:

```
uint8 cmd, uint16 distance, float angle1, float angle2, float target1, float target2
```

The V1 control packet reuses header `0x55` but is **43 bytes** with a different field layout. Parsers must use **packet size** (or field count after unpack) to distinguish the formats. New hosts should implement the 43-byte layout described in §6.

---

## 11. Quick reference

```
CONTROL:  0x55 | 8×float32 | uint16 distance | 2×float32     (43 B, 500 Hz)
STATUS:   0xAA | 4×uint8 flags | 12×float32 tuning/state       (52 B, 10 Hz)
COMMAND:  uint8 cmd | float32 data                             (5 B, host→device)
BAUD:     460800 8N1
```
