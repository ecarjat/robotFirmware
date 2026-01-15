# STM32H723 Memory Architecture & DMA Buffer Requirements

This document describes the memory layout, DMA buffer placement requirements, and allocation strategy for the self-balancing robot firmware running on the STM32H723VGT6 microcontroller.

---

## Table of Contents

1. [Memory Architecture Overview](#memory-architecture-overview)
2. [Memory Regions](#memory-regions)
3. [DMA Access Restrictions](#dma-access-restrictions)
4. [Buffer Placement Rules](#buffer-placement-rules)
5. [Current Buffer Allocations](#current-buffer-allocations)
6. [Cache Coherency Requirements](#cache-coherency-requirements)
7. [Flash Memory Layout](#flash-memory-layout)
8. [Adding New DMA Buffers](#adding-new-dma-buffers)

---

## Memory Architecture Overview

The STM32H7 series has a complex multi-bus architecture with multiple RAM domains. Each RAM region is connected to different bus masters, and **not all DMA controllers can access all memory regions**.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        STM32H723 Memory Map                             │
├─────────────────────────────────────────────────────────────────────────┤
│  0x0000_0000 ┌────────────────┐                                         │
│              │    ITCMRAM     │  64 KB  - Instruction TCM (CPU only)    │
│  0x0001_0000 └────────────────┘                                         │
│              :                :                                         │
│  0x2000_0000 ┌────────────────┐                                         │
│              │    DTCMRAM     │  128 KB - Data TCM (CPU only, fastest)  │
│  0x2002_0000 └────────────────┘                                         │
│              :                :                                         │
│  0x2400_0000 ┌────────────────┐                                         │
│              │    RAM_D1      │  320 KB - AXI SRAM (DMA1/DMA2/MDMA)     │
│  0x2405_0000 └────────────────┘                                         │
│              :                :                                         │
│  0x3000_0000 ┌────────────────┐                                         │
│              │    RAM_D2      │  32 KB  - AHB SRAM (DMA1/DMA2)          │
│  0x3000_8000 └────────────────┘                                         │
│              :                :                                         │
│  0x3800_0000 ┌────────────────┐                                         │
│              │    RAM_D3      │  16 KB  - AHB SRAM (BDMA only)          │
│  0x3800_4000 └────────────────┘                                         │
│              :                :                                         │
│  0x0802_0000 ┌────────────────┐                                         │
│              │     FLASH      │  640 KB - Application code              │
│  0x080C_0000 ├────────────────┤                                         │
│              │  PARAM_FLASH   │  128 KB - Parameter storage             │
│  0x080E_0000 └────────────────┘                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Memory Regions

### DTCMRAM (0x2000_0000 - 128 KB)

| Property | Value |
|----------|-------|
| **Base Address** | `0x20000000` |
| **Size** | 128 KB |
| **Access** | CPU only (fastest) |
| **DMA Access** | **NO** - Not accessible by any DMA |
| **Cache** | Not cached (TCM is always coherent) |
| **Use For** | Stack, heap, general BSS, fast variables |

**Linker Section:** `.data`, `.bss`, `._user_heap_stack`

### RAM_D1 / AXI SRAM (0x2400_0000 - 320 KB)

| Property | Value |
|----------|-------|
| **Base Address** | `0x24000000` |
| **Size** | 320 KB |
| **Access** | CPU, DMA1, DMA2, MDMA |
| **DMA Access** | **YES** - Standard DMA peripherals |
| **Cache** | D-Cache enabled (requires maintenance) |
| **Use For** | DMA buffers for UART, SPI, SDMMC, QSPI |

**Linker Section:** `.dma_buffer`

### RAM_D2 / AHB SRAM (0x3000_0000 - 32 KB)

| Property | Value |
|----------|-------|
| **Base Address** | `0x30000000` |
| **Size** | 32 KB |
| **Access** | CPU, DMA1, DMA2 |
| **DMA Access** | **YES** - Standard DMA peripherals |
| **Cache** | D-Cache enabled |
| **Use For** | Alternative DMA buffer location |

**Linker Section:** Not currently used in this project.

### RAM_D3 / SRAM4 (0x3800_0000 - 16 KB)

| Property | Value |
|----------|-------|
| **Base Address** | `0x38000000` |
| **Size** | 16 KB |
| **Access** | CPU, BDMA |
| **DMA Access** | **BDMA ONLY** - SPI6, I2C4, etc. |
| **Cache** | Not cached |
| **Use For** | BDMA buffers (SPI6 IMU sensors) |

**Linker Section:** `.bdma_buffer`

---

## DMA Access Restrictions

### Critical Restriction: SDMMC Internal DMA (IDMA)

**The STM32H7 SDMMC peripheral uses an internal DMA (IDMA) that CANNOT access DTCMRAM (0x2000_0000).**

This means:
- FatFS buffers must NOT be in DTCMRAM
- SD card scratch buffers must be in RAM_D1 or RAM_D2
- The default CubeMX-generated code places buffers in BSS (DTCMRAM) - this WILL NOT WORK

### DMA Controller to Memory Region Mapping

| DMA Controller | DTCMRAM | RAM_D1 | RAM_D2 | RAM_D3 |
|----------------|---------|--------|--------|--------|
| DMA1           | NO      | YES    | YES    | NO     |
| DMA2           | NO      | YES    | YES    | NO     |
| MDMA           | NO      | YES    | NO     | NO     |
| BDMA           | NO      | NO     | NO     | YES    |
| SDMMC IDMA     | **NO**  | YES    | YES    | NO     |
| OCTOSPI DMA    | NO      | YES    | YES    | NO     |

### Peripheral to DMA Mapping

| Peripheral | DMA Controller | Required Memory Region |
|------------|----------------|------------------------|
| USART1, USART2, USART6 | DMA1/DMA2 | RAM_D1 or RAM_D2 |
| SPI1, SPI2, SPI3, SPI4, SPI5 | DMA1/DMA2 | RAM_D1 or RAM_D2 |
| **SPI6** | **BDMA** | **RAM_D3 only** |
| I2C1, I2C2, I2C3 | DMA1/DMA2 | RAM_D1 or RAM_D2 |
| **I2C4** | **BDMA** | **RAM_D3 only** |
| SDMMC1 | Internal IDMA | RAM_D1 or RAM_D2 |
| OCTOSPI1 | DMA1/DMA2/MDMA | RAM_D1 |

---

## Buffer Placement Rules

### Rule 1: DMA Buffers in `.dma_buffer` Section

All buffers used with DMA1, DMA2, MDMA, SDMMC, or QSPI must be placed in the `.dma_buffer` section:

```c
static uint8_t my_dma_buffer[SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));
```

### Rule 2: BDMA Buffers in `.bdma_buffer` Section

All buffers used with BDMA (SPI6, I2C4) must be placed in the `.bdma_buffer` section:

```c
static uint8_t my_bdma_buffer[SIZE]
    __attribute__((section(".bdma_buffer"), aligned(32)));
```

### Rule 3: 32-Byte Alignment

All DMA buffers must be 32-byte aligned for D-Cache line operations:

```c
__attribute__((aligned(32)))
```

### Rule 4: Cache Maintenance Before/After DMA

- **Before DMA TX:** `SCB_CleanDCache_by_Addr()` to flush CPU writes to RAM
- **After DMA RX:** `SCB_InvalidateDCache_by_Addr()` to discard stale cache

---

## Current Buffer Allocations

### RAM_D1 (.dma_buffer) Allocations

| Module | Buffer | Size | Purpose |
|--------|--------|------|---------|
| `blackbox.c` | `s_log_queue` | 256 KB | Blackbox RAM queue (~3s @ 400Hz) |
| `blackbox.c` | `s_write_chunk` | 4 KB | QSPI write chunk buffer |
| `blackbox_dump.c` | `s_dump_buffer` | 8 KB | SD card dump read buffer |
| `blackbox_dump.c` | `s_dump_ctx` | ~100 B | Dump state context |
| `blackbox_dump.c` | `s_dump_meta` | ~68 B | Dump metadata buffer |
| `blackbox_dump.c` | `s_dump_trailer` | 16 B | Dump trailer buffer |
| `fatfs.c` | `SDFatFS` | ~600 B | FatFS filesystem structure |
| `fatfs.c` | `SDFile` | ~600 B | FatFS file handle |
| `sd_diskio.c` | `scratch` | 512 B | SD unaligned access buffer |
| `motor_link.c` | `s_left_rx_buffer` | 1 KB | Left motor UART RX |
| `motor_link.c` | `s_right_rx_buffer` | 1 KB | Right motor UART RX |
| `motor_link.c` | `s_left_tx_buffer` | 64 B | Left motor UART TX |
| `motor_link.c` | `s_right_tx_buffer` | 64 B | Right motor UART TX |
| `app_link.c` | `s_uart_rx_buffer` | 512 B | ESP32 UART RX (DMA circular) |
| `app_link.c` | `s_link_tx_queue` | ~2 KB | ESP32 UART TX queue (4 slots) |
| `app_file.c` | `s_file_handle` | ~600 B | File transfer FIL handle |
| `app_file.c` | `s_file_buffer` | 1 KB | File transfer data buffer |
| `log.c` | `s_log_ring` | 2 KB | Debug log ring buffer |

**Total RAM_D1 Usage:** ~277 KB (86.5% of 320 KB)

### RAM_D3 (.bdma_buffer) Allocations

| Module | Buffer | Size | Purpose |
|--------|--------|------|---------|
| `imu_bmi270.c` | `s_data_tx` | ~16 B | BMI270 SPI6 TX buffer |
| `imu_bmi270.c` | `s_data_rx` | ~16 B | BMI270 SPI6 RX buffer |
| `imu_icm42688.c` | `s_data_tx` | ~16 B | ICM42688 SPI6 TX buffer |
| `imu_icm42688.c` | `s_data_rx` | ~16 B | ICM42688 SPI6 RX buffer |
| `imu_bmm150.c` | `s_data_tx` | ~16 B | BMM150 SPI6 TX buffer |
| `imu_bmm150.c` | `s_data_rx` | ~16 B | BMM150 SPI6 RX buffer |

**Total RAM_D3 Usage:** ~100 B (0.6% of 16 KB)

### DTCMRAM (.bss) Allocations

All non-DMA variables, stack, and heap reside here by default.

**Total DTCMRAM Usage:** ~19 KB (14.8% of 128 KB)

---

## Cache Coherency Requirements

### D-Cache Operations

The STM32H723 has a 16 KB D-Cache with 32-byte cache lines. When using DMA:

#### Before Transmit (CPU → DMA → Peripheral)

```c
// Align addresses to cache line boundaries
uintptr_t start = (uintptr_t)buffer & ~0x1F;
uintptr_t end = ((uintptr_t)buffer + size + 31) & ~0x1F;
SCB_CleanDCache_by_Addr((uint32_t*)start, end - start);

// Now start DMA TX
HAL_UART_Transmit_DMA(...);
```

#### After Receive (Peripheral → DMA → CPU)

```c
// Wait for DMA completion...

// Invalidate cache to see new data
uintptr_t start = (uintptr_t)buffer & ~0x1F;
uintptr_t end = ((uintptr_t)buffer + size + 31) & ~0x1F;
SCB_InvalidateDCache_by_Addr((uint32_t*)start, end - start);

// Now read buffer
```

### Helper Functions (see `qspi_w25q64.c`)

```c
static void cache_clean(const void *addr, size_t len) {
    if (len == 0) return;
    uintptr_t start = (uintptr_t)addr & ~(uintptr_t)(32U - 1U);
    uintptr_t end = ((uintptr_t)addr + len + 31U) & ~(uintptr_t)(32U - 1U);
    SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}

static void cache_invalidate(void *addr, size_t len) {
    if (len == 0) return;
    uintptr_t start = (uintptr_t)addr & ~(uintptr_t)(32U - 1U);
    uintptr_t end = ((uintptr_t)addr + len + 31U) & ~(uintptr_t)(32U - 1U);
    SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}
```

---

## Flash Memory Layout

```
┌──────────────────────────────────────────────────────────────┐
│                    1 MB Internal Flash                       │
├──────────────────────────────────────────────────────────────┤
│  0x0800_0000  │  Bootloader        │  128 KB  │  (Separate)  │
│  0x0802_0000  │  Application       │  640 KB  │  FLASH       │
│  0x080C_0000  │  Parameters        │  128 KB  │  PARAM_FLASH │
│  0x080E_0000  │  BL Metadata       │  128 KB  │  (Reserved)  │
└──────────────────────────────────────────────────────────────┘
```

### QSPI External Flash (W25Q64 - 8 MB)

```
┌──────────────────────────────────────────────────────────────┐
│                    8 MB External QSPI Flash                  │
├──────────────────────────────────────────────────────────────┤
│  0x00_0000    │  Metadata (Slot 0) │   2 KB   │              │
│  0x00_0800    │  Metadata (Slot 1) │   2 KB   │              │
│  0x00_1000    │  Ring Buffer Start │          │              │
│               │         ...        │ ~8188 KB │  Blackbox    │
│  0x80_0000    │  Ring Buffer End   │          │              │
└──────────────────────────────────────────────────────────────┘
```

---

## Adding New DMA Buffers

### Checklist

1. **Identify the DMA controller** used by your peripheral
2. **Choose the correct memory region:**
   - UART, SPI1-5, I2C1-3, SDMMC, QSPI → RAM_D1 (`.dma_buffer`)
   - SPI6, I2C4 → RAM_D3 (`.bdma_buffer`)
3. **Declare with correct attributes:**

```c
// For DMA1/DMA2/MDMA/SDMMC/QSPI peripherals
static uint8_t my_buffer[SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));

// For BDMA peripherals (SPI6, I2C4)
static uint8_t my_buffer[SIZE]
    __attribute__((section(".bdma_buffer"), aligned(32)));
```

4. **Add cache maintenance** before TX and after RX
5. **Update this document** with the new allocation

### Example: Adding a New UART DMA Buffer

```c
// In your_driver.c

#define MY_UART_RX_SIZE 256U

// Place in .dma_buffer section for DMA access
static uint8_t s_my_uart_rx[MY_UART_RX_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));

void my_uart_init(void) {
    // Start circular DMA reception
    HAL_UART_Receive_DMA(&huart_x, s_my_uart_rx, MY_UART_RX_SIZE);
}

void my_uart_process(void) {
    // Invalidate cache before reading DMA-written data
    SCB_InvalidateDCache_by_Addr((uint32_t*)s_my_uart_rx, MY_UART_RX_SIZE);

    // Process received data...
}
```

---

## Common Pitfalls

### 1. Placing SDMMC Buffers in DTCMRAM

**Problem:** CubeMX generates FatFS buffers in BSS (DTCMRAM) by default.

**Symptom:** `f_mount()` fails with `FR_DISK_ERR` or hangs.

**Solution:** Override buffer placement in USER CODE sections:

```c
// In fatfs.c
FATFS SDFatFS __attribute__((section(".dma_buffer"), aligned(32)));
FIL SDFile __attribute__((section(".dma_buffer"), aligned(32)));
```

### 2. Forgetting Cache Maintenance

**Problem:** DMA writes data, but CPU reads stale cached values.

**Symptom:** Random data corruption, works sometimes.

**Solution:** Always invalidate cache after DMA RX completion.

### 3. Using SPI6/I2C4 with Wrong Memory Region

**Problem:** BDMA can only access RAM_D3.

**Symptom:** DMA transfer hangs or returns errors.

**Solution:** Place SPI6/I2C4 buffers in `.bdma_buffer` section.

### 4. Misaligned Buffers

**Problem:** Cache operations work on 32-byte lines.

**Symptom:** Adjacent data gets corrupted during cache operations.

**Solution:** Always use `aligned(32)` for DMA buffers.

---

## References

- [STM32H723 Reference Manual (RM0468)](https://www.st.com/resource/en/reference_manual/rm0468.pdf)
- [AN4839: Level 1 cache on STM32F7 Series and STM32H7 Series](https://www.st.com/resource/en/application_note/an4839.pdf)
- [AN5293: Managing memory protection unit in STM32 MCUs](https://www.st.com/resource/en/application_note/an5293.pdf)

---

*Last Updated: January 11, 2026*
