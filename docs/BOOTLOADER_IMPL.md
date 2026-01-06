# STM32H723 Bootloader Implementation

This document describes the current implementation of the USB CDC bootloader for the STM32H723VGT robot controller, including the complete flash memory layout.

---

## Flash Memory Map

The STM32H723VGT has **1024 KB** of internal flash organized into 8 sectors of 128 KB each.

| Sector | Address Range | Size | Usage |
|--------|---------------|------|-------|
| 0 | `0x08000000` - `0x0801FFFF` | 128 KB | **Bootloader** |
| 1 | `0x08020000` - `0x0803FFFF` | 128 KB | Application |
| 2 | `0x08040000` - `0x0805FFFF` | 128 KB | Application |
| 3 | `0x08060000` - `0x0807FFFF` | 128 KB | Application |
| 4 | `0x08080000` - `0x0809FFFF` | 128 KB | Application |
| 5 | `0x080A0000` - `0x080BFFFF` | 128 KB | Application |
| 6 | `0x080C0000` - `0x080DFFFF` | 128 KB | **Parameters** (reserved) |
| 7 | `0x080E0000` - `0x080FFFFF` | 128 KB | **Bootloader Metadata** |

### Summary

| Region | Base Address | End Address | Size | Description |
|--------|-------------|-------------|------|-------------|
| Bootloader | `0x08000000` | `0x0801FFFF` | 128 KB | USB CDC bootloader (write-protected) |
| Application | `0x08020000` | `0x080BFFFF` | 640 KB | User firmware |
| **Parameters** | `0x080C0000` | `0x080DFFFF` | 128 KB | Robot configuration storage |
| BL Metadata | `0x080E0000` | `0x080FFFFF` | 128 KB | Append-only app metadata sector |

---

## Bootloader Region (Sector 0)

**Address:** `0x08000000` - `0x0801FFFF` (128 KB)

The bootloader is a standalone STM32CubeIDE project located in `bootloader/bl_h723_cdc/`. It provides:

- USB CDC (VCP) interface on USB_OTG_HS in FS mode (PA11/PA12)
- KVBL protocol: COBS-framed messages with CRC32 verification
- Flash programming with 32-byte flashword alignment (STM32H7 requirement)
- Boot button on PC13 (active-high, internal pull-down)
- LED status patterns on PE3

### Boot Decision Logic

At reset, the bootloader:
1. Samples the boot button (PC13) for 50 ms with debouncing
2. Loads the latest valid metadata from the append-only sector
3. Enters bootloader mode if:
   - Boot button is held, OR
   - Metadata indicates `UPDATE_IN_PROGRESS`, OR
   - No valid application metadata exists
4. Otherwise, jumps to the application at `0x08020000`

### Key Configuration (bl_config.h)

```c
#define BL_BOOT_BASE           0x08000000UL
#define BL_BOOT_SIZE           0x00020000UL   // 128 KB
#define BL_APP_BASE            0x08020000UL
#define BL_APP_VECTOR_BASE     BL_APP_BASE    // No separate metadata header
#define BL_META_SECTOR_BASE    0x080E0000UL   // Append-only metadata sector
#define BL_META_SECTOR_SIZE    0x00020000UL   // 128 KB
#define BL_APP_MAX_SIZE        (BL_META_SECTOR_BASE - BL_APP_VECTOR_BASE)
#define BL_FLASHWORD_SIZE      32U            // H7 flash programming unit
#define BL_MAX_WRITE_CHUNK     2048U          // Max bytes per WRITE_REQ
```

---

## Application Region (Sectors 1-5)

**Address:** `0x08020000` - `0x080BFFFF` (640 KB)

The application firmware is linked to start at `0x08020000`. The vector table resides at this address (no separate metadata header in the app region).

### Linker Script Configuration

The application linker script ([STM32H723XG_FLASH.ld](../STM32H723XG_FLASH.ld)) defines the memory layout:

```ld
/* Flash layout:
 *   Bootloader:  0x08000000 - 0x0801FFFF (128K) - not mapped here
 *   Application: 0x08020000 - 0x080BFFFF (640K) - FLASH region
 *   Parameters:  0x080C0000 - 0x080DFFFF (128K) - PARAM_FLASH region
 *   BL Metadata: 0x080E0000 - 0x080FFFFF (128K) - reserved for bootloader
 */
MEMORY
{
  FLASH (rx)        : ORIGIN = 0x08020000, LENGTH = 0xA0000   /* 640K application */
  PARAM_FLASH (r)   : ORIGIN = 0x080C0000, LENGTH = 0x20000   /* 128K parameters */
  ...
}

/* Parameter storage flash region symbols (accessible from C code) */
_param_flash_start = ORIGIN(PARAM_FLASH);
_param_flash_end   = ORIGIN(PARAM_FLASH) + LENGTH(PARAM_FLASH);
_param_flash_size  = LENGTH(PARAM_FLASH);
```

### Application Requirements

- Linked to `APP_VECTOR_BASE` = `0x08020000`
- Vector table at offset 0 (standard ARM Cortex-M layout)
- VTOR is set by the bootloader before jumping
- Stack pointer (first vector word) must be in valid SRAM range

---

## Parameters Region (Sector 6)

**Address:** `0x080C0000` - `0x080DFFFF` (128 KB)

**Implementation:** [param_storage.c](../Drivers/param_storage.c) / [param_storage.h](../Drivers/param_storage.h)

This sector stores persistent robot configuration parameters:
- Motor PID gains (per wheel)
- Wheel geometry calibration
- IMU bias/calibration data
- Motion limits
- Communication settings

### Design

1. **Wear Leveling:** Append-only writes until the sector fills, then erase and restart. With 128 KB sector and ~256-byte records, this allows ~500 writes per erase cycle (~5 million parameter saves over flash lifetime).

2. **Atomic Updates:** Parameters are stored as versioned records with CRC32 validation. On boot, the module scans for the latest valid record.

3. **Default Values:** If no valid parameters are found, compile-time defaults are used.

4. **Bootloader Interaction:** The bootloader does NOT touch this sector. Firmware updates preserve stored parameters.

### Record Format

```c
typedef struct {
    uint32_t magic;           // 0x524F424F = 'ROBO'
    uint32_t version;         // Schema version (PARAM_VERSION)
    uint32_t sequence;        // Monotonic counter for wear leveling
    uint32_t data_length;     // sizeof(robot_params_t)
} param_header_t;             // 16 bytes

// Full record layout (32-byte aligned for H7 flash):
// [param_header_t][robot_params_t][padding][CRC32]
```

### Robot Parameters Structure

```c
typedef struct {
    /* Motor PID gains (left/right) */
    float motor_kp[2];
    float motor_ki[2];
    float motor_kd[2];
    float motor_max_output[2];

    /* Wheel geometry */
    float wheel_radius_m;
    float wheel_base_m;

    /* IMU calibration */
    int16_t gyro_bias[3];       /* mdps offset */
    int16_t accel_bias[3];      /* mg offset */
    int16_t mag_hard_iron[3];   /* uT offset */
    float   mag_soft_iron[9];   /* 3x3 correction matrix */

    /* Motion limits */
    float max_linear_vel_mps;
    float max_angular_vel_rps;
    float max_linear_accel_mps2;
    float max_angular_accel_rps2;

    /* Communication */
    uint32_t uart_baudrate;
    uint8_t  robot_id;

    uint8_t  reserved[64];
} robot_params_t;
```

### API Usage

```c
#include "param_storage.h"

// At startup
param_storage_init();

// Load parameters (uses defaults if none saved)
robot_params_t params;
param_storage_load(&params);

// Modify and save
params.motor_kp[0] = 2.0f;
param_storage_save(&params);

// Check storage usage
uint32_t used, free, count;
param_storage_stats(&used, &free, &count);
```

---

## Bootloader Metadata Region (Sector 7)

**Address:** `0x080E0000` - `0x080FFFFF` (128 KB)

This sector stores application metadata in an **append-only** fashion to avoid erasing the app vector table during updates.

### Why Append-Only?

On STM32H7, the minimum erase unit is a 128 KB sector. If metadata were stored at `0x08020000` (app base), every metadata update would require erasing the entire app sector, destroying the vector table.

The append-only approach:
1. Writes new metadata records sequentially
2. On boot, scans from the end to find the latest valid record
3. When the sector fills up, erases and wraps to the beginning

### Metadata Record Structure (256 bytes)

```c
typedef struct {
    uint32_t magic;           // 0x4B56424C = 'KVBL'
    uint32_t header_version;  // 1
    uint32_t image_length;    // Application size in bytes
    uint32_t image_crc32;     // CRC32 of application
    uint32_t flags;           // See below
    uint32_t reserved[56];    // Future use
    uint32_t header_crc32;    // CRC32 of header fields
    uint8_t  padding[];       // Pad to 256 bytes
} bl_app_meta_t;
```

### Metadata Flags

| Bit | Name | Description |
|-----|------|-------------|
| 0 | `VALID` | Application has been verified |
| 1 | `CONFIRMED` | Application confirmed working (for A/B schemes) |
| 2 | `UPDATE_IN_PROGRESS` | Firmware update started but not completed |

### Capacity

- Record size: 256 bytes
- Sector size: 128 KB (131,072 bytes)
- **Slots available:** 512 records before wrap

---

## KVBL Protocol

The bootloader communicates over USB CDC using COBS-framed messages.

### Frame Format

```
+----------+--------+---------+-------------+----------+----------+
| version  | msg_type | seq   | payload_len | payload  | crc32    |
| (1 byte) | (1 byte) | (2B)  | (2 bytes)   | (N bytes)| (4 bytes)|
+----------+--------+---------+-------------+----------+----------+
```

CRC32: Standard Ethernet polynomial (0x04C11DB7)

### Message Types

| Type | Code | Direction | Description |
|------|------|-----------|-------------|
| INFO_REQ | 0x01 | Host→BL | Query bootloader info |
| INFO_RESP | 0x81 | BL→Host | Bootloader info response |
| ERASE_REQ | 0x02 | Host→BL | Erase flash region |
| WRITE_REQ | 0x03 | Host→BL | Program flash chunk |
| CRC_REQ | 0x04 | Host→BL | Compute CRC over region |
| CRC_RESP | 0x84 | BL→Host | CRC result |
| BOOT_REQ | 0x05 | Host→BL | Finalize and jump to app |
| RESET_REQ | 0x06 | Host→BL | Software reset |
| ABORT_REQ | 0x07 | Host→BL | Abort update |
| ACK | 0xF0 | BL→Host | Command succeeded |
| NACK | 0xF1 | BL→Host | Command failed |

---

## LED Status Patterns (PE3)

| Pattern | Meaning |
|---------|---------|
| OFF | Early boot / decision in progress |
| 2 Hz blink | Waiting for USB enumeration |
| 1 pulse/sec | USB CDC connected, idle |
| 10 Hz blink | Flash operation active |
| Solid ON (1s) | Update verified, about to jump |
| Triple blink | Fatal error |

---

## Firmware Update Flow

### Host Side (kvbl_tool.py)

```bash
# 1. Query bootloader info
python3 kvbl_tool.py --port /dev/ttyACM0 info

# 2. Erase application region
python3 kvbl_tool.py --port /dev/ttyACM0 erase

# 3. Write firmware (2 KB chunks)
python3 kvbl_tool.py --port /dev/ttyACM0 write build/firmware.bin

# 4. Verify CRC
python3 kvbl_tool.py --port /dev/ttyACM0 verify build/firmware.bin

# 5. Commit metadata and boot
python3 kvbl_tool.py --port /dev/ttyACM0 boot --image build/firmware.bin
```

### Bootloader Side

1. **ERASE_REQ:** Erases sectors covering the specified range. Sets `UPDATE_IN_PROGRESS` flag in metadata.

2. **WRITE_REQ:** Programs data in 32-byte flashword units. Validates alignment and bounds.

3. **CRC_REQ:** Computes CRC32 over the specified flash region.

4. **BOOT_REQ:**
   - Verifies CRC matches expected value
   - Writes commit record with `VALID` flag
   - Clears `UPDATE_IN_PROGRESS`
   - Jumps to application

---

## Recovery Options

### Boot Button (PC13)

Hold PC13 during reset to force bootloader mode, even if a valid app exists.

### ROM DFU (BOOT0)

If the custom bootloader is corrupted:
1. Pull BOOT0 high (jumper or button)
2. Reset the MCU
3. Use STM32CubeProgrammer with USB DFU to reflash

---

## Source Files

### Bootloader (bootloader/bl_h723_cdc/)

| File | Description |
|------|-------------|
| `bl_config.h` | Memory map constants, protocol settings |
| `bl_main.c` | Boot decision logic, message dispatch |
| `bl_flash.c` | Flash erase/program with H7 flashword handling |
| `bl_appmeta.c` | Append-only metadata sector management |
| `bl_protocol.c` | KVBL message parsing and encoding |
| `bl_cobs.c` | COBS framing encoder/decoder |
| `bl_crc32.c` | CRC32 implementation |
| `bl_jump.c` | VTOR setup and jump-to-app sequence |
| `bl_usb_cdc.c` | USB CDC wrapper |
| `bl_console.c` | LED patterns, boot button |

### Host Tool (bootloader/host/)

| File | Description |
|------|-------------|
| `kvbl_tool.py` | Python CLI for firmware updates |

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2024-12 | Initial bootloader implementation |
| 1.1 | 2024-12 | Reserved Sector 6 for parameter storage |
| 1.2 | 2024-12 | Implemented param_storage module with wear-leveling |
