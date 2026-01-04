# Robust Binary I/O Protocol

A reliable binary framing protocol with byte stuffing and hardware-accelerated CRC-32 for STM32-to-STM32 UART communication.

## Overview

This protocol replaces the standard Arduino-FOC BinaryIO framing to solve synchronization issues caused by the 0xA5 marker byte appearing within float telemetry data.

**Key Features:**
- SLIP-style byte stuffing prevents false frame synchronization
- CRC-32 error detection using STM32 hardware CRC peripheral
- Compatible with existing PacketCommander/Telemetry APIs

## Frame Format

```
┌────────┬─────┬──────┬─────────────┬────────┐
│ MARKER │ LEN │ TYPE │ PAYLOAD     │ CRC32  │
│  0xA5  │ 1B  │  1B  │  0-N bytes  │  4B    │
└────────┴─────┴──────┴─────────────┴────────┘
     │      │      │        │           │
     │      │      │        │           └── CRC-32 of (LEN + TYPE + PAYLOAD)
     │      │      │        └── Variable length data
     │      │      └── Packet type ('R', 'r', 'T', etc.)
     │      └── Length of (TYPE + PAYLOAD + CRC32)
     └── Frame start marker (never appears in stuffed data)
```

### Field Descriptions

| Field | Size | Description |
|-------|------|-------------|
| MARKER | 1 byte | `0xA5` - Frame synchronization marker |
| LEN | 1 byte | Length of TYPE + PAYLOAD + CRC32 (minimum 5) |
| TYPE | 1 byte | Packet type identifier |
| PAYLOAD | 0-N bytes | Packet data (register values, telemetry, etc.) |
| CRC32 | 4 bytes | CRC-32 checksum, little-endian |

### Packet Types (inherited from BinaryIO)

| Type | Char | Description |
|------|------|-------------|
| 0x52 | 'R' | Register read/write request |
| 0x72 | 'r' | Register response |
| 0x54 | 'T' | Telemetry data |
| 0x48 | 'H' | Telemetry header |
| 0x42 | 'B' | Bootloader command |
| 0x57 | 'W' | Write settings |
| 0x77 | 'w' | Write settings response |
| 0x43 | 'C' | Calibration command |
| 0x63 | 'c' | Calibration response |
| 0x4C | 'L' | Log message |

## Byte Stuffing

To prevent the marker byte (0xA5) from appearing in frame data, SLIP-style escaping is used:

| Original Byte | Escaped Sequence |
|---------------|------------------|
| 0xA5 (marker) | 0xDB 0xDC |
| 0xDB (escape) | 0xDB 0xDD |

**Escape Characters:**
- `FRAME_ESC` = 0xDB
- `FRAME_ESC_MARKER` = 0xDC (represents 0xA5)
- `FRAME_ESC_ESC` = 0xDD (represents 0xDB)

### Example

Sending a register response with value `0xA5 0x00 0x00 0x00` (float):

```
Unescaped: [A5][06][72][00][A5][00][00][00][xx][xx][xx][xx]
                              ^^
                          Needs escaping

Escaped:   [A5][06][72][00][DB DC][00][00][00][xx][xx][xx][xx]
            ^               ^^^^^
         Marker          Escaped 0xA5
       (not escaped)
```

## CRC-32

### Polynomial

```
0x04C11DB7 (CRC-32/MPEG-2, same as STM32 hardware CRC)
```

This polynomial is used by the STM32 hardware CRC peripheral, enabling hardware acceleration on both the H7 controller and F103 driver.

### Calculation Scope

CRC is calculated over the **unescaped** bytes: `LEN + TYPE + PAYLOAD`

```
CRC32 = crc32([LEN][TYPE][PAYLOAD...])
```

### Byte Order

CRC-32 is transmitted **little-endian** (LSB first).

## Implementation

### Controller Side (STM32H7)

**Files:**
- `Drivers/motors/motor_link_framing.h`
- `Drivers/motors/motor_link_framing.c`

**Key Functions:**

```c
// Initialize parser state
void frame_parser_init(frame_parser_t *parser);

// Feed raw UART bytes to parser
void frame_parser_feed(frame_parser_t *parser, const uint8_t *data, size_t len);

// Extract complete frame (returns true if valid frame available)
bool frame_parser_pop(frame_parser_t *parser,
                      uint8_t *out_type,
                      const uint8_t **out_payload,
                      uint8_t *out_len);

// Encode a frame for transmission
size_t frame_encode(uint8_t type,
                    const uint8_t *payload,
                    size_t payload_len,
                    uint8_t *out,
                    size_t out_size);

// CRC-32 calculation (uses hardware when available)
uint32_t frame_crc32(const uint8_t *data, size_t len);
```

**Parser State Machine:**

```
         ┌──────────┐
         │   IDLE   │◄────────────────┐
         └────┬─────┘                 │
              │ 0xA5                  │
              ▼                       │
         ┌──────────┐                 │
         │   LEN    │─── invalid ────►│
         └────┬─────┘                 │
              │ valid                 │
              ▼                       │
         ┌──────────┐                 │
         │   DATA   │─── complete ───►│
         └────┬─────┘                 │
              │ 0xA5 (unexpected)     │
              └───────────────────────┘
```

### Driver Side (STM32F103)

**Files:**
- `src/robust_binary_io.h`
- `src/robust_binary_io.cpp`

**Usage:**

```cpp
#include "robust_binary_io.h"

// Replace FlushingBinaryIO with FlushingRobustBinaryIO
static FlushingRobustBinaryIO dma_io(uart_dma_stream());

// Use with PacketCommander and Telemetry as before
packet_commander.init(dma_io);
telemetry.init(dma_io);
```

**Statistics:**

```cpp
// Monitor for debugging
uint32_t sync_losses = dma_io.sync_losses();  // Re-sync events
uint32_t crc_errors = dma_io.crc_errors();    // CRC mismatches
```

## Hardware CRC Setup

Both MCUs require the CRC peripheral to be initialized.

### STM32CubeMX Configuration

1. Enable CRC peripheral in Pinout & Configuration
2. No additional settings required (uses default polynomial)

### Manual Initialization

```c
CRC_HandleTypeDef hcrc;

void MX_CRC_Init(void) {
    __HAL_RCC_CRC_CLK_ENABLE();
    hcrc.Instance = CRC;
    HAL_CRC_Init(&hcrc);
}
```

**Note:** The `hcrc` handle must be declared as `extern` and accessible to the framing code.

## Error Handling

### Sync Loss Recovery

When the parser encounters:
- Invalid escape sequence
- Unexpected marker in mid-frame
- Invalid length field

It discards the partial frame and searches for the next 0xA5 marker.

### CRC Mismatch

If the computed CRC doesn't match the received CRC:
1. Frame is discarded
2. `crc_errors` counter increments
3. Parser returns to IDLE state

### Diagnostics

Monitor these counters for link health:

| Counter | Meaning | Action |
|---------|---------|--------|
| `sync_losses` | Parser re-synchronized | Check for noise, buffer overflows |
| `crc_errors` | Data corruption detected | Check wiring, EMI shielding |

## Performance

### Overhead

| Component | Bytes |
|-----------|-------|
| Marker | 1 |
| Length | 1 |
| CRC-32 | 4 |
| **Total overhead** | **6 bytes** |

With byte stuffing, worst case doubles payload size (if every byte is 0xA5 or 0xDB).

### Timing (Hardware CRC)

| MCU | CRC-32 of 16 bytes |
|-----|-------------------|
| STM32F103 @ 72MHz | ~1 us |
| STM32H7 @ 480MHz | <0.5 us |

Hardware CRC has negligible impact on throughput.

## Migration from BinaryIO

### Compatibility

The robust protocol is **not backward compatible** with standard BinaryIO due to:
1. Different CRC (CRC-32 vs none)
2. Byte stuffing changes frame structure

Both ends must be updated simultaneously.

### Step-by-Step Migration

1. **Update driver firmware:**
   ```cpp
   // In comms_streams.cpp
   #include "robust_binary_io.h"
   static FlushingRobustBinaryIO dma_io(uart_dma_stream());
   ```

2. **Update controller firmware:**
   - Replace `motor_link_parser_*` calls with `frame_parser_*`
   - Use `frame_encode()` for transmission

3. **Initialize CRC peripheral** on both sides

4. **Test with telemetry disabled** first, then re-enable

## Troubleshooting

### No Communication

1. Verify CRC peripheral is initialized (`hcrc.Instance != NULL`)
2. Check UART baud rate matches on both sides
3. Confirm byte order (little-endian CRC)

### High CRC Errors

1. Check UART wiring and ground connections
2. Reduce baud rate to test
3. Add ferrite beads for EMI filtering

### High Sync Losses

1. Check for buffer overflows (increase `ROBUST_RX_BUFFER_SIZE`)
2. Verify no interrupt priority issues affecting UART DMA
3. Monitor `HAL_UART_ErrorCallback` for UART errors

## References

- [SLIP Protocol (RFC 1055)](https://datatracker.ietf.org/doc/html/rfc1055)
- [STM32 CRC Peripheral Application Note (AN4187)](https://www.st.com/resource/en/application_note/an4187-using-the-crc-peripheral-on-stm32-microcontrollers-stmicroelectronics.pdf)
- [Arduino-FOC-drivers BinaryIO](https://github.com/simplefoc/Arduino-FOC-drivers/tree/master/src/comms/streams)
