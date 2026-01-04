# Robust Binary I/O Protocol v2

A reliable binary framing protocol with byte stuffing, hardware-accelerated CRC-32, and consolidated command structure for STM32-to-STM32 UART communication.

## Version History

| Version | Changes |
|---------|---------|
| v1 | Initial robust framing with byte stuffing and CRC-32 |
| v2 | Consolidated command packets under single 'C'/'c' type |

## Overview

This protocol provides reliable UART communication between the STM32H7 controller and STM32F103 motor drivers.

**Key Features:**
- SLIP-style byte stuffing prevents false frame synchronization
- CRC-32 error detection using STM32 hardware CRC peripheral
- Consolidated command structure reduces packet type proliferation
- Compatible with Arduino-FOC PacketCommander/Telemetry APIs

---

## Frame Format

```
┌────────┬─────┬──────┬─────────────┬────────┐
│ MARKER │ LEN │ TYPE │ PAYLOAD     │ CRC32  │
│  0xA5  │ 1B  │  1B  │  0-N bytes  │  4B    │
└────────┴─────┴──────┴─────────────┴────────┘
```

### Field Descriptions

| Field | Size | Description |
|-------|------|-------------|
| MARKER | 1 byte | `0xA5` - Frame synchronization marker |
| LEN | 1 byte | Length of TYPE + PAYLOAD + CRC32 (minimum 5) |
| TYPE | 1 byte | Packet type identifier |
| PAYLOAD | 0-N bytes | Packet data |
| CRC32 | 4 bytes | CRC-32 checksum, little-endian |

---

## Byte Stuffing

SLIP-style escaping prevents the marker byte from appearing in frame data:

| Original Byte | Escaped Sequence |
|---------------|------------------|
| 0xA5 (marker) | 0xDB 0xDC |
| 0xDB (escape) | 0xDB 0xDD |

**Constants:**
```c
#define FRAME_MARKER      0xA5
#define FRAME_ESC         0xDB
#define FRAME_ESC_MARKER  0xDC  // 0xDB 0xDC = 0xA5
#define FRAME_ESC_ESC     0xDD  // 0xDB 0xDD = 0xDB
```

---

## CRC-32

**Polynomial:** `0x04C11DB7` (CRC-32/MPEG-2, STM32 hardware compatible)

**Scope:** CRC is calculated over unescaped `LEN + TYPE + PAYLOAD`

**Byte Order:** Little-endian (LSB first)

---

## Packet Types

### Core Packet Types

| Type | Char | Direction | Description |
|------|------|-----------|-------------|
| 0x52 | 'R' | TX | Register read/write request |
| 0x72 | 'r' | RX | Register response |
| 0x54 | 'T' | RX | Telemetry data |
| 0x48 | 'H' | RX | Telemetry header |
| 0x43 | 'C' | TX | **Command request (v2)** |
| 0x63 | 'c' | RX | **Command response (v2)** |
| 0x4C | 'L' | RX | Log message |

### Deprecated Packet Types (v1 only)

| Type | Char | Replaced By |
|------|------|-------------|
| 0x42 | 'B' | 'C' with CMD_BOOTLOADER |
| 0x57 | 'W' | 'C' with CMD_WRITE |
| 0x62 | 'b' | 'c' response |
| 0x77 | 'w' | 'c' response |

---

## Command Packet Structure (v2)

### Command Request ('C' = 0x43)

```
┌──────┬─────────┬─────────────┐
│ TYPE │ CMD_ID  │ CMD_PAYLOAD │
│ 'C'  │  1 byte │  0-N bytes  │
└──────┴─────────┴─────────────┘
```

### Command Response ('c' = 0x63)

```
┌──────┬─────────┬────────┬──────────────┐
│ TYPE │ CMD_ID  │ STATUS │ RESP_PAYLOAD │
│ 'c'  │  1 byte │ 1 byte │  0-N bytes   │
└──────┴─────────┴────────┴──────────────┘
```

**STATUS values:**
| Value | Meaning |
|-------|---------|
| 0x00 | Success |
| 0x01 | Error |
| 0x02 | Busy |
| 0xFF | Unknown command |

---

## Command Definitions

### CMD_WRITE (0x01) - Save Settings to Flash

Saves current motor configuration to flash memory.

**Request:**
```
[C][0x01]
```

**Response:**
```
[c][0x01][status]
```

### CMD_CALIBRATE (0x02) - Sensor Calibration

Runs sensor calibration routine and stores results.

**Request:**
```
[C][0x02]
```

**Response:**
```
[c][0x02][status]
```

**Note:** Calibration takes several seconds. Motor will be temporarily controlled during calibration.

### CMD_BOOTLOADER (0x03) - Enter Bootloader

Resets device into bootloader mode for firmware update.

**Request:**
```
[C][0x03]
```

**Response:** None (device resets immediately)

### Reserved Command IDs

| CMD_ID | Reserved For |
|--------|--------------|
| 0x00 | Reserved |
| 0x04-0x0F | Future system commands |
| 0x10-0x1F | Diagnostic commands |
| 0x20-0xFF | Application-specific |

---

## Register Packet Structure

### Register Request ('R' = 0x52)

```
┌──────┬─────────┬─────────────┐
│ TYPE │ REG_ID  │ VALUE       │
│ 'R'  │  1 byte │  0-N bytes  │
└──────┴─────────┴─────────────┘
```

- **Read:** REG_ID only (no VALUE)
- **Write:** REG_ID + VALUE bytes

### Register Response ('r' = 0x72)

```
┌──────┬─────────┬─────────────┐
│ TYPE │ REG_ID  │ VALUE       │
│ 'r'  │  1 byte │  0-N bytes  │
└──────┴─────────┴─────────────┘
```

---

## Telemetry Packet Structure

### Telemetry Data ('T' = 0x54)

```
┌──────┬──────────┬─────────────────────┐
│ TYPE │ MOTOR_ID │ REGISTER_VALUES     │
│ 'T'  │  1 byte  │  variable           │
└──────┴──────────┴─────────────────────┘
```

Register values are concatenated in the order configured via REG_TELEMETRY_REG.

---

## Common Registers

| Register | ID | Size | Description |
|----------|------|------|-------------|
| STATUS | 0x00 | 1 | Motor status flags |
| TARGET | 0x01 | 4 | Target setpoint (float) |
| ENABLE | 0x04 | 1 | Motor enable (0/1) |
| CONTROL_MODE | 0x05 | 1 | Control mode |
| TORQUE_MODE | 0x06 | 1 | Torque control mode |
| MODULATION_MODE | 0x07 | 1 | PWM modulation mode |
| VELOCITY | 0x11 | 4 | Velocity (float, rad/s) |
| TELEMETRY_REG | 0x1A | var | Telemetry register config |
| TELEMETRY_DOWNSAMPLE | 0x1C | 4 | Telemetry rate divisor, keep at 1, prefer TELEMETRY_MIN_ELAPSED |
| TELEMETRY_MIN_ELAPSED | 0x1E | 4 | Telemetry timer, elapsed time in microseconds between two telemetry packets|

---

## Implementation Notes

### Hardware CRC Setup

Both MCUs require CRC peripheral initialization:

```c
CRC_HandleTypeDef hcrc;

void MX_CRC_Init(void) {
    __HAL_RCC_CRC_CLK_ENABLE();
    hcrc.Instance = CRC;
    HAL_CRC_Init(&hcrc);
}
```

### Buffer Sizes

| Buffer | Recommended Size |
|--------|------------------|
| TX | 72 bytes (worst case: 32 payload * 2 + overhead) |
| RX | 128 bytes |
| Parser | 64 bytes (unescaped) |

### Timing

| Parameter | Value |
|-----------|-------|
| Baud rate | 460800 |
| Telemetry rate | 500 Hz default |
| Command timeout | 200 ms |
| Calibration timeout | 10 s |

---

## Error Handling

### Sync Loss Recovery

On invalid escape sequence, unexpected marker, or invalid length:
1. Discard partial frame
2. Increment `sync_losses` counter
3. Search for next 0xA5 marker

### CRC Mismatch

On CRC verification failure:
1. Discard frame
2. Increment `crc_errors` counter
3. Return to IDLE state

### Command Timeout

If no response received within timeout:
1. Increment `ack_timeouts` counter
2. Optionally retry command

---

## Diagnostic Counters

| Counter | Description |
|---------|-------------|
| `sync_losses` | Parser re-synchronization events |
| `crc_errors` | CRC verification failures |
| `ack_timeouts` | Command response timeouts |
| `tx_drops` | TX buffer overflow events |

---

## Example Frames

### Enable Motor (Write ENABLE=1)

```
Request:  [A5][06][52][04][01][CRC32...]
           │   │   │   │   │
           │   │   │   │   └── Value: 1 (enable)
           │   │   │   └────── Register: ENABLE (0x04)
           │   │   └────────── Type: 'R' (register request)
           │   └────────────── Length: 6 (type + reg + value + crc)
           └────────────────── Marker

Response: [A5][06][72][04][01][CRC32...]
                   │
                   └── Type: 'r' (register response)
```

### Save Settings (Command)

```
Request:  [A5][06][43][01][CRC32...]
                   │   │
                   │   └── CMD_WRITE
                   └────── Type: 'C' (command)

Response: [A5][07][63][01][00][CRC32...]
                   │   │   │
                   │   │   └── Status: 0x00 (success)
                   │   └────── CMD_WRITE
                   └────────── Type: 'c' (command response)
```

### Enter Bootloader

```
Request:  [A5][06][43][03][CRC32...]
                   │   │
                   │   └── CMD_BOOTLOADER
                   └────── Type: 'C' (command)

(No response - device resets)
```

---

## Files

### Controller (STM32H7)

| File | Description |
|------|-------------|
| `motor_link_framing.h` | Frame encode/decode API |
| `motor_link_framing.c` | Implementation with HW CRC |
| `motor_link.c` | Motor communication logic |

### Driver (STM32F103)

| File | Description |
|------|-------------|
| `robust_binary_io.h` | RobustBinaryIO class |
| `robust_binary_io.cpp` | Implementation with HW CRC |
| `comms_streams.cpp` | Command handling |
