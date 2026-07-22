# LiDAR Module Serial Protocol

Host-facing reference for the USB-serial control and telemetry link used by this firmware.

**Firmware sources of truth**

| Topic | File |
|-------|------|
| Command IDs, packet structs | [`src/interface/Interface.hpp`](../src/interface/Interface.hpp) |
| Command handling / TX | [`src/interface/Interface.cpp`](../src/interface/Interface.cpp) |
| Fault codes / error text | [`src/fault_manager/FaultManager.hpp`](../src/fault_manager/FaultManager.hpp) |
| Enable / sleep / bootloader | [`src/system_manager/SystemManager.cpp`](../src/system_manager/SystemManager.cpp) |
| Baud, defaults, rates | [`src/utils/constants.h`](../src/utils/constants.h), [`src/utils/tweakable_params.h`](../src/utils/tweakable_params.h) |

---

## 1. Physical layer

| Parameter | Value |
|-----------|-------|
| Medium | USB CDC serial (Teensy 4.1 / STM32G4) |
| Baud | **460800** (`BAUD_RATE`) |
| Framing | 8 data bits, no parity, 1 stop bit (8N1) |
| Endianness | **Little-endian** for all multi-byte fields |
| Float format | IEEE 754 binary32 |

The same port carries:

1. **Binary telemetry** — packed structs via `Serial.write()`
2. **Text lines** — newline-terminated ASCII (boot / debug / fault messages)

Hosts must treat the RX stream as mixed binary + text and **resync on packet headers** (§5).

---

## 2. Frame formats

### 2.1 Host → device (command)

Every command is exactly **5 bytes**:

| Offset | Type | Field |
|--------|------|-------|
| 0 | `uint8` | `cmd` |
| 1–4 | `float32` | `data` |

`data` is always present. Toggle / lifecycle commands ignore its value; send `0.0`.

The firmware reads a command only when ≥ 5 bytes are available (`commandListener()`). Incomplete reads after the command byte raise `FAULT_PROTO_OVERFLOW`.

Unknown `cmd` values raise `FAULT_CMD_INVALID`.

### 2.2 Device → host (telemetry)

| Packet | Header | Size | Nominal rate | Constant |
|--------|--------|------|--------------|----------|
| Control | `0x55` | **27 bytes** | 500 Hz | `CONTROL_PACKET_PERIOD_US` = 2000 µs |
| Status | `0xAA` | **56 bytes** | 10 Hz | `STATUS_PACKET_PERIOD_MS` = 100 ms |

No checksums in this revision. Distinguish packet types by header byte and fixed size.

---

## 3. Coordinate and unit conventions

| Name | Axis | Role |
|------|------|------|
| X / theta | motor X | first gimbal axis |
| Y / phi | motor Y | second gimbal axis |

| Quantity | Unit |
|----------|------|
| Angles | radians |
| Velocities | rad/s |
| LPF time constants | seconds |
| `distance` | centimetres (`uint16`) |
| Velocity feed-forward | A·s/rad |

**Mechanical angles** (`mechanical_angle_x/y`) are the live AS5048 rotor angles.
**Associated angles** (`associated_angle_x/y`) are the AngleTracker estimate at the
last LDM sample time. The motor-control UI plots targets vs mechanical only.

While scanning under ANGLE mode, the onboard `ScanPath` may drive targets. Otherwise the host streams setpoints with `TARGET_X_SET` / `TARGET_Y_SET`.

---

## 4. Recommended host session

```
open port @ 460800
        │
        ▼
[optional] RECONFIGURE_LDM   ← only while disabled
[optional] set PID / LPF / scan-path params
        │
        ▼
DISABLE_ENABLE_CMD           ← toggle enable on
        │
        ├─ ANGLE + host path: stream TARGET_X / TARGET_Y
        ├─ ANGLE + onboard scan: SCAN_PLAY_PAUSE_CMD
        └─ VELOCITY: TOGGLE_CONTROL_MODE then stream targets as velocities
        │
        ▼
parse Control (0x55) + Status (0xAA) continuously
periodically GET_ERROR_MSG_CMD if faults are expected
        │
        ▼
DISABLE_ENABLE_CMD           ← disable before LDM reconfig / shutdown
[optional] SLEEP / RESTART / ENTER_BOOTLOADER
```

**State rules the host must respect**

| Action | Requirement |
|--------|-------------|
| `SCAN_PLAY_PAUSE_CMD` | `control_mode == ANGLE`; else `FAULT_CMD_REJECTED_STATE` |
| `RECONFIGURE_LDM_CMD` | system **disabled**; else `FAULT_CMD_REJECTED_STATE` |
| Telemetry rates | packets are emitted by the production loop while the system is running; enable before expecting useful closed-loop data |
| After `SLEEP_CMD` | actuators are disabled; host must `WAKE` then `DISABLE_ENABLE` again to run |
| `ENTER_BOOTLOADER_CMD` | disables drive, then jumps to the MCU bootloader (DFU / Teensy reboot) — link drops |

---

## 5. Parsing strategy (device → host)

1. Append RX bytes to a buffer.
2. Search for `0x55` (control) or `0xAA` (status).
3. If fewer than the packet size bytes remain, wait.
4. Unpack; on success consume exactly that many bytes.
5. Otherwise skip one byte (or drain a text line ending in `\n`) and resync.

**Header collision:** payload floats can contain `0x55` / `0xAA`. Prefer validating cadence (~2 ms / ~100 ms) or requiring two consecutive valid packets before trusting fields.

**Text vs binary:** non-header bytes are often debug or fault text. Prefer draining to `\n` when not at a known header.

---

## 6. Control packet (`0x55`, 27 bytes)

High-rate closed-loop stream for the motor-control UI (`UI/realtime_plotter.py`).

| Offset | Type | Field | Notes |
|--------|------|-------|-------|
| 0 | `uint8` | `header` | `0x55` |
| 1 | `float32` | `target_angle_x` | commanded X target (angle in ANGLE mode) |
| 5 | `float32` | `target_angle_y` | commanded Y target |
| 9 | `uint16` | `distance` | LDM range [cm] (0 on FOC bench stub) |
| 11 | `float32` | `mechanical_angle_x` | live X rotor angle [rad] |
| 15 | `float32` | `mechanical_angle_y` | live Y rotor angle [rad] |
| 19 | `float32` | `associated_angle_x` | X angle at last LDM sample [rad] |
| 23 | `float32` | `associated_angle_y` | Y angle at last LDM sample [rad] |

```c
#pragma pack(push, 1)
struct ControlPacket {
    uint8_t header;
    float target_angle_x;
    float target_angle_y;
    uint16_t distance;
    float mechanical_angle_x;
    float mechanical_angle_y;
    float associated_angle_x;
    float associated_angle_y;
};
#pragma pack(pop)
```

```python
CONTROL_HEADER = 0x55
CONTROL_STRUCT = struct.Struct("<BffHffff")  # 27 bytes
```

**Plotter:** overlays `target_*` vs `mechanical_*` for both axes; associated fields are consumed for framing only. Host streams setpoints with `TARGET_X_SET` / `TARGET_Y_SET` (onboard `ScanPath` is optional / unused by the plotter).

**Point cloud:** use `(distance_cm, associated_angle_x, associated_angle_y)`. Between LDM samples, `distance` and associated angles retain their last values.

---

## 7. Status packet (`0xAA`, 56 bytes)

Housekeeping and live tuning mirror (values from motor X; both motors share the same setpoints when changed over serial).

| Offset | Type | Field | Notes |
|--------|------|-------|-------|
| 0 | `uint8` | `header` | `0xAA` |
| 1 | `uint8` | `enabled` | `0` = disabled |
| 2 | `uint8` | `control_mode` | see below |
| 3 | `uint8` | `scanning` | `0` = paused |
| 4 | `float32` | `velocity_p` | |
| 8 | `float32` | `velocity_i` | |
| 12 | `float32` | `velocity_d` | |
| 16 | `float32` | `velocity_ramp` | |
| 20 | `float32` | `angle_p` | |
| 24 | `float32` | `angle_i` | |
| 28 | `float32` | `angle_d` | |
| 32 | `float32` | `angle_ramp` | |
| 36 | `float32` | `velocity_lpf_tau` | [s] |
| 40 | `float32` | `angle_lpf_tau` | [s] |
| 44 | `float32` | `foc_voltage_lpf_tau` | [s] |
| 48 | `float32` | `velocity_feed_forward` | [A·s/rad] |
| 52 | `float32` | `acceleration_feed_forward` | [A·s²/rad] |

**`control_mode`** (`StateMachine.hpp`):

| Value | Name | Meaning |
|-------|------|---------|
| `0` | `PATH` | reserved |
| `1` | `ANGLE` | position control (default) |
| `2` | `VELOCITY` | velocity control |

```python
STATUS_HEADER = 0xAA
STATUS_STRUCT = struct.Struct("<BBBBfffffffffffff")  # 56 bytes
```

---

## 8. Command reference (host → device)

### 8.1 State / motion

| Command | ID | Hex | `data` | Behaviour |
|---------|----|-----|--------|-----------|
| `DISABLE_ENABLE_CMD` | `0b00000001` | `0x01` | ignored | Toggle system enable (FOC, sensors, LDM) |
| `SCAN_PLAY_PAUSE_CMD` | `0b00000010` | `0x02` | ignored | Toggle onboard scan path (ANGLE mode only) |
| `TOGGLE_CONTROL_MODE_CMD` | `0b00000011` | `0x03` | ignored | Toggle ANGLE ↔ VELOCITY on both motors |
| `TARGET_X_SET_CMD` | `0b00010000` | `0x10` | float | ANGLE → X angle [rad]; VELOCITY → X velocity [rad/s] |
| `TARGET_Y_SET_CMD` | `0b00010100` | `0x14` | float | ANGLE → Y angle [rad]; VELOCITY → Y velocity [rad/s] |

### 8.2 Tuning (applied to both motors)

| Command | ID | Hex | `data` |
|---------|----|-----|--------|
| `VELOCITY_P_SET_CMD` | `0b00001000` | `0x08` | velocity P |
| `VELOCITY_I_SET_CMD` | `0b00001001` | `0x09` | velocity I |
| `VELOCITY_D_SET_CMD` | `0b00001010` | `0x0A` | velocity D |
| `VELOCITY_RAMP_SET_CMD` | `0b00001011` | `0x0B` | velocity ramp |
| `ANGLE_P_SET_CMD` | `0b00001100` | `0x0C` | angle P |
| `ANGLE_I_SET_CMD` | `0b00001101` | `0x0D` | angle I |
| `ANGLE_D_SET_CMD` | `0b00001110` | `0x0E` | angle D |
| `ANGLE_RAMP_SET_CMD` | `0b00001111` | `0x0F` | angle ramp |
| `FOC_VOLTAGE_LPF_TAU_SET_CMD` | `0b00010001` | `0x11` | FOC voltage LPF τ [s] |
| `VELOCITY_LPF_TAU_SET_CMD` | `0b00010010` | `0x12` | sensor velocity LPF τ [s] |
| `ANGLE_LPF_TAU_SET_CMD` | `0b00010011` | `0x13` | sensor angle LPF τ [s] |
| `VELOCITY_FF_SET_CMD` | `0b10010010` | `0x92` | velocity feed-forward [A·s/rad] |
| `ACCELERATION_FF_SET_CMD` | `0b10010011` | `0x93` | acceleration feed-forward [A·s²/rad] |

### 8.3 Scan path (onboard Lissajous / sinusoidal generator)

| Command | ID | Hex | `data` | Notes |
|---------|----|-----|--------|-------|
| `THETA_PHASE_INC_SET_CMD` | `0b00100001` | `0x21` | float → `uint32` | X frequency (phase increment) |
| `PHI_PHASE_INC_SET_CMD` | `0b00100010` | `0x22` | float → `uint32` | Y frequency |
| `THETA_AMPLITUDE_SET_CMD` | `0b00100011` | `0x23` | float [rad] | X amplitude |
| `PHI_AMPLITUDE_SET_CMD` | `0b00100100` | `0x24` | float [rad] | Y amplitude |
| `THETA_CENTRE_SET_CMD` | `0b00100101` | `0x25` | float [rad] | X centre |
| `PHI_CENTRE_SET_CMD` | `0b00100110` | `0x26` | float [rad] | Y centre |

Phase increments are truncated with `static_cast<uint32_t>(data)`. Send the integer value as a float (e.g. `1500007.0`).

Default amplitudes / centres / increments: see `tweakable_params.h`.

### 8.4 LDM

| Command | ID | Hex | `data` | Notes |
|---------|----|-----|--------|-------|
| `RECONFIGURE_LDM_CMD` | `0b00100000` | `0x20` | config index as float | Requires **disabled**. Index = `(uint16_t)data`. |

Garmin V3HP presets (`config_number`):

| Index | Mode |
|-------|------|
| 0 | Default / balanced (boot default) |
| 1 | Short range, high speed |
| 2 | Default range, higher speed short range |
| 3 | Maximum range |
| 4 | High sensitivity (more false positives) |
| 5 | Low sensitivity (fewer false positives) |
| 6 | Short range, high speed, higher error |

### 8.5 Power / lifecycle / faults

| Command | ID | Hex | `data` | Behaviour |
|---------|----|-----|--------|-----------|
| `SLEEP_CMD` | `0b00011000` | `0x18` | ignored | Disable drive, enter MCU sleep / WFI |
| `WAKE_CMD` | `0b00011001` | `0x19` | ignored | Restore clocks after sleep; **does not** re-enable |
| `RESTART_CMD` | `0b00011010` | `0x1A` | ignored | `sleep()` then `wake()` |
| `ENTER_BOOTLOADER_CMD` | `0b00011011` | `0x1B` | ignored | Disable, then MCU bootloader |
| `GET_ERROR_MSG_CMD` | `0b00011100` | `0x1C` | ignored | Dump and clear latched faults as text |

---

## 9. Faults and error text

Faults are sticky latches in `FaultManager`. They do **not** appear in the status packet.

**Host pull:** send `GET_ERROR_MSG_CMD` (`0x1C`). For each latched fault the device writes a newline-terminated ASCII string and clears that fault.

Example responses:

```
invalid command
command rejected in current state
serial protocol framing error
encoder X SPI parity
LDM timeout
```

### Protocol-related codes

| Code | Name | When |
|------|------|------|
| 128 | `FAULT_CMD_INVALID` | unknown command byte |
| 129 | `FAULT_CMD_REJECTED_STATE` | scan while not ANGLE; LDM reconfig while enabled |
| 130 | `FAULT_PROTO_OVERFLOW` | command framing short read |

### Full fault table (for host logging)

| Range | Domain | Codes (selected) |
|-------|--------|------------------|
| 1–15 | System | init failed (1), not initialised (2), watchdog (3), loop overrun (4) |
| 16–31 | Encoder X | init (16), update (17), SPI parity (18), COF (19), OCF (20), mag low/high (21/22), AGC (23) |
| 32–47 | Encoder Y | same pattern as X (+16) |
| 48–63 | PWM | X init (48), Y init (49), sync (50) |
| 64–95 | FOC / motion | X/Y init (64/65), NaN cmd (66/67), tracking (68/69), vel limit (70/71), e-stop (72) |
| 96–111 | Current sense | X/Y init (96/97), offset (98/99), overcurrent (100/101) |
| 112–127 | LDM | init (112), I2C (113), timeout (114), invalid range (115), config (116) |
| 128–143 | Protocol | see above |
| 144–159 | Angle tracker | X overflow (144), Y overflow (145) |

Message strings: `FaultManager::getMessage()`.

---

## 10. Sending a command (Python example)

```python
import struct
import serial

CMD = struct.Struct("<Bf")  # cmd + float32 LE

def send_cmd(ser: serial.Serial, cmd: int, data: float = 0.0) -> None:
    ser.write(CMD.pack(cmd & 0xFF, float(data)))

ser = serial.Serial(port="COM3", baudrate=460800, timeout=0.05)

send_cmd(ser, 0x01)          # enable / disable toggle
send_cmd(ser, 0x10, 0.25)    # TARGET_X = 0.25 rad
send_cmd(ser, 0x14, -0.10)   # TARGET_Y
send_cmd(ser, 0x1C)          # pull fault text
```

---

## 11. Minimal Python receiver

```python
import struct
import serial

CONTROL = struct.Struct("<BffHffff")           # 27 B, header 0x55
STATUS  = struct.Struct("<BBBBfffffffffffff") # 56 B, header 0xAA

def parse_stream(ser: serial.Serial):
    buf = bytearray()
    while True:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf.extend(chunk)
        while buf:
            if buf[0] == 0x55 and len(buf) >= CONTROL.size:
                yield ("control", CONTROL.unpack_from(buf))
                del buf[: CONTROL.size]
            elif buf[0] == 0xAA and len(buf) >= STATUS.size:
                yield ("status", STATUS.unpack_from(buf))
                del buf[: STATUS.size]
            elif buf[0] in (0x55, 0xAA):
                break
            else:
                try:
                    nl = buf.index(b"\n")
                    line = bytes(buf[: nl + 1])
                    del buf[: nl + 1]
                    yield ("text", line.decode("utf-8", errors="replace").strip())
                except ValueError:
                    i55 = buf.find(b"\x55", 1)
                    iaa = buf.find(b"\xaa", 1)
                    cand = [i for i in (i55, iaa) if i != -1]
                    del buf[: (min(cand) if cand else 1)]
```

---

## 12. Migration notes

| Revision | Control packet | Notes |
|----------|----------------|-------|
| Legacy `DataStream` | 19 B, different fields | `cmd, distance, angle1, angle2, target1, target2` |
| Prior V1 doc | **43 B** `0x55` | included velocities + mechanical angles |
| **Current** | **27 B** `0x55` | `target_x/y`, `distance`, `mechanical_x/y`, `associated_x/y` |

Status packet is **56 bytes** / `0xAA` (includes velocity + acceleration feed-forward).

Hosts must key off **header + size**. Do not assume the old 43-byte control layout.

Command renames: `TARGET_1_SET` / `TARGET_2_SET` → `TARGET_X_SET` (`0x10`) / `TARGET_Y_SET` (`0x14`).

New since the prior doc: sleep / wake / restart / bootloader, `GET_ERROR_MSG`, LDM reconfigure, scan-path parameter commands, and the fault manager.

---

## 13. Quick reference

```
COMMAND:  [uint8 cmd][float32 data]                         5 B   host → device
CONTROL:  0x55 | target_x/y | dist u16 | mech_x/y | assoc_x/y  27 B  ~500 Hz
STATUS:   0xAA | en | mode | scan | 12×float tuning         52 B  ~10 Hz
FAULTS:   GET_ERROR_MSG (0x1C) → ASCII lines, then clear
BAUD:     460800 8N1, little-endian IEEE754 floats
```
