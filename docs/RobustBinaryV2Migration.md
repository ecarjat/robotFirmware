# Robust Binary I/O v2 Migration Guide

This document outlines the migration steps to implement the Robust Binary I/O v2 protocol on both the STM32H7 controller and STM32F103 motor driver.

## Overview

The v2 protocol introduces:
1. **Robust framing** with SLIP-style byte stuffing
2. **CRC-32 verification** using hardware CRC peripheral
3. **Consolidated command structure** ('C'/'c' packet type)

## Migration Checklist

### Phase 1: Core Framing (Both Sides)

- [ ] Controller: Implement `motor_link_framing.c/h`
- [ ] Controller: Initialize CRC peripheral in `main.c`
- [ ] Driver: Implement `robust_binary_io.cpp/h`
- [ ] Driver: Verify CRC peripheral initialization

### Phase 2: Command Consolidation (Both Sides)

- [ ] Controller: Update `motor_link.c` to use new command structure
- [ ] Driver: Update `comms_streams.cpp` to handle consolidated commands
- [ ] Define command IDs in shared header

### Phase 3: Integration & Testing

- [ ] Disable telemetry for initial testing
- [ ] Test register read/write
- [ ] Test each command (Write, Calibrate, Bootloader)
- [ ] Re-enable telemetry
- [ ] Verify no regressions

---

## Controller Side (STM32H7)

### Step 1: Add Framing Files

Ensure these files exist:
- `Drivers/motors/motor_link_framing.h`
- `Drivers/motors/motor_link_framing.c`

### Step 2: Initialize CRC Peripheral

In `main.c`, add CRC initialization:

```c
/* In MX_CRC_Init() or similar */
CRC_HandleTypeDef hcrc;

void MX_CRC_Init(void) {
    __HAL_RCC_CRC_CLK_ENABLE();
    hcrc.Instance = CRC;
    hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
    hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
    HAL_CRC_Init(&hcrc);
}
```

Declare `hcrc` as extern in `motor_link_framing.c`:
```c
extern CRC_HandleTypeDef hcrc;
```

### Step 3: Update motor_link.c - Command Definitions

Add command ID definitions:

```c
/* Command IDs (v2 protocol) */
#define CMD_WRITE       0x01
#define CMD_CALIBRATE   0x02
#define CMD_BOOTLOADER  0x03

/* Command response status */
#define CMD_STATUS_OK      0x00
#define CMD_STATUS_ERROR   0x01
#define CMD_STATUS_BUSY    0x02
#define CMD_STATUS_UNKNOWN 0xFF
```

### Step 4: Update motor_link.c - TX Functions

Replace direct UART writes with framed encoding.

**Before (v1):**
```c
void motor_link_send_bootloader(motor_link_t *link) {
    uint8_t buf[8];
    buf[0] = 0xA5;
    buf[1] = 1;
    buf[2] = 'B';
    HAL_UART_Transmit_DMA(link->huart, buf, 3);
}
```

**After (v2):**
```c
void motor_link_send_command(motor_link_t *link, uint8_t cmd_id) {
    uint8_t payload[1] = { cmd_id };
    uint8_t buf[16];
    size_t len = frame_encode('C', payload, 1, buf, sizeof(buf));
    if (len > 0) {
        HAL_UART_Transmit_DMA(link->huart, buf, len);
    }
}

void motor_link_send_bootloader(motor_link_t *link) {
    motor_link_send_command(link, CMD_BOOTLOADER);
}

void motor_link_send_write(motor_link_t *link) {
    motor_link_send_command(link, CMD_WRITE);
}

void motor_link_send_calibrate(motor_link_t *link) {
    motor_link_send_command(link, CMD_CALIBRATE);
}
```

### Step 5: Update motor_link.c - RX Parser

Replace the existing parser with the frame parser.

**Before (v1):**
```c
void motor_link_rx_callback(motor_link_t *link, uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        // Old state machine...
    }
}
```

**After (v2):**
```c
void motor_link_rx_callback(motor_link_t *link, uint8_t *data, size_t len) {
    frame_parser_feed(&link->parser, data, len);

    uint8_t type;
    const uint8_t *payload;
    uint8_t payload_len;

    while (frame_parser_pop(&link->parser, &type, &payload, &payload_len)) {
        motor_link_handle_packet(link, type, payload, payload_len);
    }
}

void motor_link_handle_packet(motor_link_t *link, uint8_t type,
                               const uint8_t *payload, uint8_t len) {
    switch (type) {
    case 'r':  /* Register response */
        if (len >= 1) {
            uint8_t reg_id = payload[0];
            motor_link_handle_register(link, reg_id, &payload[1], len - 1);
        }
        break;

    case 'c':  /* Command response (v2) */
        if (len >= 2) {
            uint8_t cmd_id = payload[0];
            uint8_t status = payload[1];
            motor_link_handle_command_response(link, cmd_id, status,
                                                &payload[2], len - 2);
        }
        break;

    case 'T':  /* Telemetry */
        motor_link_handle_telemetry(link, payload, len);
        break;

    case 'L':  /* Log message */
        motor_link_handle_log(link, payload, len);
        break;

    default:
        break;
    }
}
```

### Step 6: Update motor_link.h

Add frame parser to the link structure:

```c
#include "motor_link_framing.h"

typedef struct {
    UART_HandleTypeDef *huart;
    frame_parser_t parser;  /* Add this */

    /* ... existing fields ... */

    /* Statistics */
    uint32_t sync_losses;
    uint32_t crc_errors;
} motor_link_t;
```

### Step 7: Update motor_link_init()

Initialize the frame parser:

```c
void motor_link_init(motor_link_t *link, UART_HandleTypeDef *huart) {
    memset(link, 0, sizeof(*link));
    link->huart = huart;
    frame_parser_init(&link->parser);
}
```

---

## Driver Side (STM32F103)

### Step 1: Add Robust Binary IO Files

Ensure these files exist in `src/`:
- `robust_binary_io.h`
- `robust_binary_io.cpp`

### Step 2: Verify CRC Peripheral

The CRC peripheral should already be initialized by STM32CubeIDE/CubeMX. Verify in `main.c`:

```c
CRC_HandleTypeDef hcrc;

void MX_CRC_Init(void) {
    __HAL_RCC_CRC_CLK_ENABLE();
    hcrc.Instance = CRC;
    HAL_CRC_Init(&hcrc);
}
```

Declare `hcrc` as extern in `robust_binary_io.cpp`:
```cpp
extern CRC_HandleTypeDef hcrc;
```

### Step 3: Update comms_streams.cpp - IO Class

Replace `FlushingBinaryIO` with `FlushingRobustBinaryIO`:

**Before:**
```cpp
#include <comms/BinaryIO.h>
static FlushingBinaryIO dma_io(uart_dma_stream());
```

**After:**
```cpp
#include "robust_binary_io.h"
static FlushingRobustBinaryIO dma_io(uart_dma_stream());
```

### Step 4: Update comms_streams.cpp - Command Handler

Update the command handler to use consolidated 'C'/'c' format:

**Before (v1 - multiple handlers):**
```cpp
// In receiveDownsampleDelay callback or similar
case 'B':  // Bootloader
    system_reset_to_bootloader();
    break;
case 'W':  // Write
    motor->saveToFlash();
    send_write_ack();
    break;
case 'C':  // Calibrate
    start_calibration();
    break;
```

**After (v2 - consolidated handler):**
```cpp
void handle_command(uint8_t cmd_id, const uint8_t* payload, uint8_t len) {
    uint8_t status = CMD_STATUS_OK;

    switch (cmd_id) {
    case CMD_WRITE:  // 0x01
        if (motor->saveToFlash()) {
            status = CMD_STATUS_OK;
        } else {
            status = CMD_STATUS_ERROR;
        }
        send_command_response(cmd_id, status);
        break;

    case CMD_CALIBRATE:  // 0x02
        if (motor->isCalibrating()) {
            status = CMD_STATUS_BUSY;
            send_command_response(cmd_id, status);
        } else {
            start_calibration();
            // Response sent after calibration completes
        }
        break;

    case CMD_BOOTLOADER:  // 0x03
        // No response - device resets immediately
        system_reset_to_bootloader();
        break;

    default:
        send_command_response(cmd_id, CMD_STATUS_UNKNOWN);
        break;
    }
}

void send_command_response(uint8_t cmd_id, uint8_t status) {
    dma_io << Packet{'c', 2}
           << cmd_id
           << status
           << Packet{0, 0};  // END_PACKET
}
```

### Step 5: Update PacketCommander Integration

If using PacketCommander, register the new command handler:

```cpp
void setup_comms() {
    packet_commander.init(dma_io);

    // Register 'C' packet handler for consolidated commands
    packet_commander.addTarget('C', [](PacketIO& io) {
        uint8_t cmd_id;
        io >> cmd_id;

        // Read any additional payload if present
        uint8_t payload[16];
        uint8_t payload_len = 0;
        while (!io.is_complete() && payload_len < sizeof(payload)) {
            io >> payload[payload_len++];
        }

        handle_command(cmd_id, payload, payload_len);
    });
}
```

### Step 6: Add Command Response for Calibration Complete

Update the calibration complete callback:

```cpp
void on_calibration_complete(bool success) {
    send_command_response(CMD_CALIBRATE,
                          success ? CMD_STATUS_OK : CMD_STATUS_ERROR);
}
```

---

## Shared Definitions

Create a shared header or ensure both sides define:

```c
/* Command IDs */
#define CMD_WRITE       0x01
#define CMD_CALIBRATE   0x02
#define CMD_BOOTLOADER  0x03

/* Status codes */
#define CMD_STATUS_OK      0x00
#define CMD_STATUS_ERROR   0x01
#define CMD_STATUS_BUSY    0x02
#define CMD_STATUS_UNKNOWN 0xFF
```

---

## Testing Procedure

### Phase 1: Basic Communication

1. **Flash both devices** with updated firmware
2. **Disable telemetry** temporarily:
   ```c
   // Controller: comment out telemetry enable
   // motor_link_enable_telemetry(link, false);
   ```
3. **Test register read:**
   - Send register read request
   - Verify response received with correct CRC
4. **Test register write:**
   - Send register write request
   - Verify acknowledgment

### Phase 2: Command Testing

Test each command in sequence:

1. **CMD_WRITE (0x01):**
   ```
   TX: [A5][06][43][01][CRC32...]
   RX: [A5][07][63][01][00][CRC32...]  // status=OK
   ```

2. **CMD_CALIBRATE (0x02):**
   ```
   TX: [A5][06][43][02][CRC32...]
   RX: [A5][07][63][02][00][CRC32...]  // after calibration
   ```

3. **CMD_BOOTLOADER (0x03):**
   ```
   TX: [A5][06][43][03][CRC32...]
   // No response - device resets
   ```

### Phase 3: Telemetry Verification

1. **Re-enable telemetry**
2. **Monitor sync_losses and crc_errors counters**
3. **Run for extended period** (>1 hour)
4. **Verify zero CRC errors** under normal operation

### Phase 4: Stress Testing

1. **Interleave commands with telemetry**
2. **Send rapid register requests**
3. **Monitor for any synchronization issues**

---

## Rollback Plan

If issues are encountered:

1. **Controller:** Revert `motor_link.c` to use direct parsing
2. **Driver:** Revert to `FlushingBinaryIO`
3. **Both:** Disable CRC verification (not recommended for production)

---

## Diagnostic Commands

Add debug commands to monitor link health:

**Controller:**
```c
void motor_link_print_stats(motor_link_t *link) {
    printf("sync_losses: %lu\n", link->parser.sync_losses);
    printf("crc_errors:  %lu\n", link->parser.crc_errors);
}
```

**Driver:**
```cpp
void print_io_stats() {
    Serial.printf("sync_losses: %lu\n", dma_io.sync_losses());
    Serial.printf("crc_errors:  %lu\n", dma_io.crc_errors());
}
```

---

## Troubleshooting

### No Communication After Migration

1. Verify CRC peripheral is initialized on both sides
2. Check baud rate matches (460800)
3. Use logic analyzer to capture raw bytes
4. Verify frame marker (0xA5) appears at frame start

### High CRC Errors

1. Check UART wiring integrity
2. Verify both sides use same CRC polynomial (0x04C11DB7)
3. Verify little-endian CRC byte order
4. Check for buffer overflows

### High Sync Losses

1. Increase RX buffer sizes
2. Check for interrupt priority issues
3. Verify DMA is functioning correctly

### Command Responses Not Received

1. Verify driver is sending 'c' (0x63) type, not old types
2. Check controller is handling 'c' packet type
3. Verify command ID matches in request and response

---

## Version Compatibility

| Controller | Driver | Compatible |
|------------|--------|------------|
| v1 | v1 | Yes |
| v2 | v2 | Yes |
| v1 | v2 | **No** |
| v2 | v1 | **No** |

**Both sides must be updated simultaneously.**
