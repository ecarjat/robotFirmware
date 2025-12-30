# BRINGUP_TEST_PLAN.md — Codex-ready STM32H723 Bring-up Plan (WeAct H723 + USB CDC + SDMMC + QSPI + I2C + SPI + UART)
Version: 1.0
Target: WeAct STM32H723VGT6 board
Clock: HSE 25MHz (PH0/PH1) → PLL1 (SYSCLK), PLL3Q (USB 48MHz), PLL2R (SDMMC kernel)
Goal: Produce a deterministic, stepwise bring-up firmware that validates each subsystem in isolation, then in combination, with clear PASS/FAIL criteria and minimal coupling.

---

## 0) Principles
- Bring up **one subsystem at a time** with minimal dependencies.
- Each test:
  - has a single entrypoint function `test_<name>()`
  - prints structured logs via a known-good console (UART2 or USB CDC)
  - returns PASS/FAIL code
- Never block the control loop: use short bounded waits; prefer state machines.
- Only enable DMA/interrupts when needed for that test.

---

## 1) Required Hardware Setup
- ST-Link connected for SWD debugging (see `Pinmap.md` for reserved pins).
- USB cable from STM32 USB port to host (PC or Raspberry Pi).
- microSD card inserted (FAT32 formatted recommended for early tests).
- QSPI flash is on-board (W25Q64).
- IMUs connected:
  - I2C1: BMI270 (see `Pinmap.md` for wiring)
  - I2C2: BMM150 (see `Pinmap.md` for wiring)
  - SPI6: ICM-42688 (see `Pinmap.md` for wiring)
- ESP32 may be disconnected for early phases.
- Motors and LiDARs disconnected for early phases (reduce noise, simplify).

---

## 2) Build Targets / Firmware Modes
Create separate build-time modes to reduce complexity:
- BUILD_MODE_SMOKE: clocks + LED + console
- BUILD_MODE_USB: USB CDC console
- BUILD_MODE_SD: SDMMC + FATFS
- BUILD_MODE_QSPI: QSPI JEDEC + read/write
- BUILD_MODE_I2C: I2C scan + BMI270/BMM150 ID reads
- BUILD_MODE_SPI_IMU: SPI6 ICM-42688 WHOAMI + data read
- BUILD_MODE_UART: UART loopbacks / basic framing
- BUILD_MODE_INTEGRATION: combined sanity suite

Implementation:
- Use a compile-time define `BRINGUP_MODE` selecting which tests run from `main()`.

---

## 3) Standard Console Output Format
All tests must log:
- timestamp (ms)
- test name
- step name
- result

Example:
- [001234] TEST=QSPI STEP=JEDEC_ID RESULT=PASS mf=0xEF id=0x4017
- [001567] TEST=SDMMC STEP=MOUNT RESULT=FAIL err=-13

Console priority:
1) UART2 @ 921600 (most reliable at start)
2) USB CDC once validated

---

## 4) Test Order (Do Not Skip)
Run tests in this order and only proceed when PASS.

### Phase A — Core + Clock + Basic GPIO
A1. Clock Sanity + LED Blink
A2. SysTick + Timer (1kHz) sanity

### Phase B — Console Transport
B1. UART2 console
B2. USB CDC enumerate + TX/RX

### Phase C — Storage
C1. QSPI JEDEC ID
C2. QSPI read/write/erase
C3. SDMMC init
C4. FATFS mount + create/write/read file

### Phase D — Sensors
D1. I2C1 scan + BMI270 WHOAMI
D2. I2C2 scan + BMM150 ID
D3. SPI6 ICM-42688 WHOAMI
D4. IMU DRDY EXTI line toggling (optional)

### Phase E — Serial Links
E1. ESP32 spine UART (framing sanity)
E2. Motor UART links (per-node ping)
E3. LiDAR UART parsing (per sensor)

### Phase F — Integration
F1. All enabled simultaneously (USB + SD + QSPI + I2C + SPI + UART2)
F2. Background logging while reading sensors
F3. File download primitive over USB (read chunk test)

---

## 5) Test Specifications (Detailed)

### A1) Clock Sanity + LED Blink
Purpose: verify clocks, flash latency, and main loop health.
Setup:
- Configure clocks as per CubeMX (HSE 25MHz).
Procedure:
1) initialize HAL
2) toggle LED at 2Hz for 10 seconds
3) print system clock frequencies via HAL (if available)
PASS:
- LED toggles at correct rate
- no HardFault
- frequency readouts plausible

### A2) 1kHz Timer Tick
Purpose: validate TIMx configuration and interrupt priorities.
Procedure:
1) start TIMx at 1kHz interrupt
2) increment `tick1k_count` in ISR
3) in main loop, every 1000 ticks print "1kHz OK"
PASS:
- stable ~1Hz prints without drift >1% over 30s
- no missed ISR storms (count monotonic)

---

### B1) UART2 Console (USART2 or dedicated console UART)
Purpose: ensure stable prints at chosen baud (921600 recommended).
Procedure:
1) init UART console
2) print "UART OK" once per second for 10 seconds
PASS:
- no garbled characters
- host receives all lines

Optional:
- loopback TX->RX and echo characters for 5 seconds.

---

### B2) USB CDC Enumerate + Loopback
Purpose: confirm USB device stack and CLK48 source.
Procedure:
1) enable USB device CDC
2) wait up to 5 seconds for `usbConfigured == true`
3) when configured:
   - send "USB CDC OK"
   - implement RX callback to echo received bytes
PASS:
- host sees `/dev/ttyACM0`
- echo works reliably for 30 seconds
FAIL:
- never configured
- stalls after some transfers

---

### C1) QSPI JEDEC ID (W25Q64)
Purpose: confirm QSPI wiring + mode + clock.
Procedure:
1) init QSPI (indirect mode)
2) issue JEDEC ID (0x9F), read 3 bytes
Expected:
- Manufacturer = 0xEF (Winbond)
- Memory type/capacity indicates W25Q64 (commonly 0x40 0x17)
PASS:
- bytes match expected and stable across 10 reads
Notes:
- if mismatch: reduce prescaler, confirm mode0, confirm sample shifting half-cycle

---

### C2) QSPI Erase/Program/Verify
Purpose: validate write/erase pipeline and QE bit handling.
Procedure:
1) read SR1/SR2; ensure QE set; set if needed
2) choose test sector at a safe offset (e.g. 0x000000 or 0x10000)
3) sector erase (4KB or 64KB based on command used)
4) program 256-byte pattern page
5) read back and verify
PASS:
- verify exact match
- erase yields 0xFF
FAIL:
- timeout, verify mismatch, stuck WIP

---

### C3) SDMMC Init
Purpose: ensure SDMMC kernel clock source and signal integrity.
Procedure:
1) init SDMMC1
2) query card state + capacity
PASS:
- init returns OK
- CID/CSD read OK
FAIL:
- init errors, timeouts
Mitigation:
- confirm SDMMC clock source PLL2R <= 200MHz
- confirm 4-bit mode and external pullups

---

### C4) FATFS Mount + File R/W
Purpose: confirm filesystem integration.
Procedure:
1) f_mount
2) create file `/bringup.txt`
3) write known text + timestamp
4) close
5) reopen and read back
PASS:
- read back matches written
- file persists after reset

---

### D1) I2C1 Scan + BMI270 ID
Purpose: validate primary IMU bus.
Procedure:
1) I2C scan 0x08..0x77 (non-blocking or bounded)
2) report found addresses
3) read BMI270 chip ID register (per driver)
PASS:
- BMI270 address present
- chip ID correct and stable
FAIL:
- NACKs, bus errors
Mitigation:
- drop to 100kHz
- verify pullups on I2C1
- check wiring and address strap

---

### D2) I2C2 Scan + BMM150 ID
Procedure:
1) scan I2C2
2) read BMM150 chip ID
PASS:
- device found, ID correct
Mitigation:
- ensure each bus has its own pullups

---

### D3) SPI6 ICM-42688 WHOAMI
Purpose: validate SPI6 wiring + IMU ID.
Procedure:
1) read ICM-42688 WHO_AM_I over SPI6
2) optionally read TEMP/ACC/GYRO burst once
PASS:
- WHO_AM_I = 0x47
FAIL:
- 0x00/0xFF or timeout
Mitigation:
- verify CS/MISO/MOSI/SCK, SPI mode 3, and power/reset

---

### D4) IMU DRDY EXTI Toggle (Optional)
Purpose: validate GPIO EXTI wiring and edge trigger.
Procedure:
1) configure EXTI for BMI270 INT pin
2) enable data-ready interrupt at low rate (e.g. 100Hz)
3) count EXTI triggers for 5 seconds
PASS:
- count ~500 ± 5%
FAIL:
- 0 triggers or noisy triggers
Mitigation:
- invert edge, check pull config, ensure IMU INT configured push-pull

---

### E1) ESP32 Spine UART Sanity (no protocol yet)
Purpose: validate physical UART + framing readiness.
Procedure:
1) send periodic ping frame `0xC0 ... CRC` (or raw text) at 10Hz
2) require reply from ESP32 within 2s
PASS:
- round-trip stable for 30s
Notes:
- if RTS/CTS wired, enable and confirm no drops at 921600

---

### E2) Motor UART Link Ping (per node)
Purpose: validate each motor node connection without moving motors.
Procedure:
1) for each motor UART:
   - send `PING`
   - expect `PONG` + node info
PASS:
- all online nodes respond
FAIL:
- any node missing → show which UART/pins

---

### E3) LiDAR UART Parse
Purpose: validate TFmini frame parser and per-sensor timing.
Procedure:
1) for each LiDAR UART:
   - read for 2 seconds
   - count valid frames
   - compute mean distance if target present
PASS:
- frames/sec plausible
- CRC/checksum ok
Mitigation:
- confirm 3.3V UART logic
- confirm parser expects correct TFmini output format

---

### F1) Integration Suite
Enable simultaneously:
- UART2 console
- USB CDC
- QSPI periodic read
- SD periodic write (every 2s)
- I2C IMU read at 200–400Hz (cached)
- Ensure CPU load acceptable

PASS:
- no resets, no bus lockups
- SD file grows correctly
- USB remains enumerated

---

### F2) Background Logging Stress
Procedure:
1) write 1MB log to SD in chunks (e.g. 4KB)
2) while doing so:
   - keep USB responsive (echo)
   - keep IMU reads running
PASS:
- no missed IMU read deadlines beyond threshold
- no SD errors

---

### F3) FILE Channel Primitive over USB (minimal)
Purpose: validate your future FILE API path.
Procedure:
1) implement `LIST` and `READ_CHUNK` for a single test file
2) from host, request chunked reads
PASS:
- reconstructed file matches SD contents

---

## 6) Acceptance Criteria (Global)
System passes bring-up when:
- USB CDC enumerates and stays stable for 10 minutes
- SD FATFS R/W passes and survives reset
- QSPI JEDEC + R/W passes
- I2C1 BMI270 ID OK, I2C2 BMM150 ID OK, SPI6 ICM-42688 ID OK
- UART2 at 921600 is stable
- Timer 1kHz stable
- No HardFaults, no brownouts under integration stress

---

## 7) Deliverables to Generate
Codex must generate:
1) `bringup/bringup.h` + `bringup/bringup.c`
   - test runner, enum of tests, PASS/FAIL codes
2) `bringup/tests/test_clock.c`
3) `bringup/tests/test_uart_console.c`
4) `bringup/tests/test_usb_cdc.c`
5) `bringup/tests/test_qspi_w25q64.c`
6) `bringup/tests/test_sdmmc_fatfs.c`
7) `bringup/tests/test_i2c_scan.c` (I2C1 + I2C2)
8) `bringup/tests/test_spi_imu.c` (ICM-42688 WHO_AM_I)
9) `bringup/tests/test_exti_imu.c` (optional)
10) `bringup/tests/test_uart_links.c` (ESP32/motors/lidars skeletons)
11) `bringup/host_tools/` (optional)
   - small python script to open ttyACM0 and run LIST/READ_CHUNK tests

Each test must compile independently and be callable from `main()` via `BRINGUP_MODE`.

---

## 8) Notes for Codex
- Use HAL drivers generated by CubeMX; do not rewrite peripheral init.
- Keep ISRs minimal; use flags/queues.
- Provide clear, single-line log output for each PASS/FAIL.
- Avoid blocking calls > 50ms in the main bring-up loop unless specifically required.
- For QSPI, explicitly handle QE bit and WIP polling.

End of plan.
