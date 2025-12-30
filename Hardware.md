# Hardware.md — Robot Brain Wiring & Port Mapping (STM32H723VGT6 + Coprocessors)

## 0) Purpose
This document describes the **hardware wiring and MCU port mapping** for the robot “brain” based on:

- **Main controller board:** WeAct **STM32H723VGT6**
- **On-board peripherals already connected:** **microSD**, **TFT screen**, **USB**
- **External peripherals:**
  - 3× TFmini Plus LiDAR (Front / Back / Down)
  - 2× IMU + 1× magnetometer:
    - **ICM-42688** (IMU)
    - **BMI270** (IMU)
    - **BMM150** (magnetometer)
  - 4× motor driver nodes (each runs SimpleFOC; UART link to brain)
  - ESP32 BT coprocessor module (UART link)
  - Dual-color status LED (red/green)
  - Buttons (defined below)

> Note: This file defines **logical peripheral allocation** (UART/I2C/etc.). Exact pin/peripheral assignments are centralized in `Pinmap.md`; use that document as the authoritative reference when wiring or configuring CubeMX.

---

## 1) Power Domains
### 1.1 Rails
- **3.3V rail**
  - STM32H723 board logic
  - IMUs (ICM-42688, BMI270), magnetometer (BMM150)
  - ESP32 module (3.3V)
  - Status LED (through resistors)
  - Buttons (GPIO pull-ups)
- **5V rail**
  - 3× TFmini Plus LiDAR supply = **5V ±0.5V**
  - (Optional) any other 5V sensors

### 1.2 Grounding
- Single system ground star preferred.
- Keep IMU ground return clean:
  - Avoid sharing high-current motor return paths near the IMU ground reference.
- ESP32 RF module ground should be low-impedance; keep antenna area clear.

---

## 2) External Interfaces Summary
### 2.1 UART Consumers
- 4× Motor driver UART links
- 3× LiDAR UART links
- 1× ESP32 UART link
- (Optional) 1× debug UART (recommended if available)

### 2.2 I2C Buses
- I2C1: BMI270 IMU bus (high-rate, clean)
- I2C2: BMM150 magnetometer bus (lower-rate, isolated)

### 2.3 SPI
- On-board TFT and on-board SD card use SPI/SDMMC resources (already wired).
- SPI6 is allocated to the ICM42688 IMU (shared SPI bus layer for sensors).

---

## 3) Port Mapping (Logical Peripheral Allocation)

### 3.1 UART Mapping
All UART RX should use **DMA circular buffer + IDLE line interrupt** for frame parsing.

#### Motor Driver Nodes (SimpleFOC on STM32F103, 1 per motor)
| Motor Node | Function (suggested) | STM32 Peripheral | Direction | Notes |
|---|---|---|---|---|
| Motor Node 1 | Left wheel | USART1 | TX/RX | High-rate command/telemetry |
| Motor Node 2 | Right wheel | USART6 | TX/RX | High-rate command/telemetry (per Pinmap) |
| Motor Node 3 | Knee/aux motor A | USART3 | TX/RX | Reserve even if unused initially |
| Motor Node 4 | Knee/aux motor B | UART4  | TX/RX | Reserve even if unused initially |

**Baud target:** 460800 or 921600 (final depends on link stability/cabling)

#### LiDAR (TFmini Plus)
| LiDAR | Mount | STM32 Peripheral | Direction | Notes |
|---|---|---|---|---|
| LiDAR 1 | Front | UART5  | RX (TX optional) | Obstacle distance |
| LiDAR 2 | Back  | UART7  | RX (TX optional) | Reverse safety |
| LiDAR 3 | Down  | UART8  | RX (TX optional) | Height/cliff sensing |

**Baud:** 115200 typical (confirm TFmini Plus configuration)

#### Bluetooth Coprocessor (ESP32)
| Device | STM32 Peripheral | Direction | Notes |
|---|---|---|---|
| ESP32-WROOM-32E | USART2 | TX/RX | Runs Bluepad32 + UART bridge (see Pinmap for RTS/CTS) |

**Baud target:** 460800 or 921600  
**Optional:** RTS/CTS hardware flow control if routed and needed.

---

### 3.2 I2C Mapping
Use short wires, good pull-ups, and keep IMU bus isolated from noisy devices.

#### I2C1 — IMU Bus (High-rate)
| Device | Type | Bus | Notes |
|---|---|---|---|
| BMI270 | 6-axis IMU | I2C1 | Primary/high-rate IMU |

**Speed:** 400 kHz (safe) or 1 MHz (if wiring supports)  
**Recommended:** route each IMU **INT** pin to a dedicated EXTI GPIO.

#### I2C2 — Magnetometer Bus
| Device | Type | Bus | Notes |
|---|---|---|---|
| BMM150 | Magnetometer | I2C2 | Heading/yaw reference; lower rate |

**Speed:** 100–400 kHz

#### SPI6 — IMU Bus
| Device | Type | Bus | Notes |
|---|---|---|---|
| ICM-42688 | 6-axis IMU | SPI6 | SPI mode 3, INT1 via EXTI |

---

## 4) GPIO Mapping

### 4.1 Dual-color Status LED (Red/Green)
Assume bi-color LED with current-limiting resistors.

| Signal | GPIO Type | Notes |
|---|---|---|
| LED_GREEN | Output (PWM-capable preferred) | “OK / balanced / armed” |
| LED_RED | Output (PWM-capable preferred) | “Fault / disarmed / error codes” |

Suggested behavior:
- Solid green = ready/armed
- Blinking green = balancing
- Solid red = fault latched
- Red blink codes = fault ID

---

### 4.2 Buttons (Recommended Set = 4)
Buttons should be GPIO inputs with pull-up; pressed = GND.

| Button | Function | GPIO Type | Notes |
|---|---|---|---|
| BTN1 | ARM / Enable balancing | Input + EXTI | Long-press optional |
| BTN2 | Mode cycle (PID/LQR/CAL/DIAG) | Input + EXTI | Short press cycles |
| BTN3 | IMU Calibrate / Zero | Input + EXTI | Only allowed when disarmed |
| BTN4 | E-Stop / Disarm | Input + EXTI | Immediate torque = 0 |

If you want the minimum set, keep **BTN1 + BTN4**.

---

### 4.3 Reserved/Optional GPIOs (Strongly Recommended)
Reserve these signals even if not implemented on day 1.

| Signal | Purpose |
|---|---|
| ESP32_EN | Reset/control ESP32 enable |
| ESP32_IO0 | ESP32 boot mode (for flashing) |
| MOTOR_EN_GLOBAL | Global enable to motor nodes (if supported) |
| FAULT_IN | External fault latch input (e.g., motor node fault OR) |
| BUZZER (optional) | Audible feedback for state/fault |

---

## 5) Connectors & Cabling Recommendations

### 5.1 UART Cabling
- Prefer twisted pairs for TX/GND and RX/GND for longer runs.
- Keep LiDAR UART lines away from motor phase wires.
- Consider JST-GH / JST-XH connectors (locking) for vibration environments.

### 5.2 I2C Cabling
- Keep I2C runs short (<20–30 cm ideally).
- Use pull-ups near the master (STM32) or on the sensor breakout if present.
- Keep IMU bus physically separated from motor and LiDAR cabling.

### 5.3 ESP32 Antenna Clearance
- Keep metal and wiring away from the antenna end of the module.
- Prefer the N* variants with PCB antenna unless you need U.FL.

---

## 6) On-board Resources (WeAct Board)
The board already has:
- microSD (logging)
- TFT screen (UI/debug dashboard)
- USB port (power + programming + possibly USB-CDC logging)

Do not reassign the pins/peripherals consumed by these on-board devices.

---

## 7) Firmware/Timing Assumptions (Hardware-facing)
- **Main control loop:** timer-driven at 500 Hz–1 kHz
- **UART ingest:** DMA circular + IDLE interrupt for:
  - 4 motor node streams
  - 3 LiDAR streams
  - 1 ESP32 stream
- **Logging:** buffered writes to SD; must never block the control loop
- **Display:** low-rate refresh (10–20 Hz) via DMA where possible

---

## 8) Bring-up Checklist
1. Power rails stable: 3.3V clean; 5V LiDAR rail capable of peak current.
2. Verify BMI270 I2C1 communication; validate INT lines.
3. Verify ICM-42688 SPI6 communication.
4. Verify magnetometer I2C2 communication.
4. Verify each LiDAR UART receives valid frames (one at a time).
5. Verify motor node UART handshake (one at a time).
6. Bring up ESP32 UART link (basic packet echo) before Bluepad32 integration.
7. Validate buttons + LED patterns.
8. Enable SD logging last (after real-time loop is stable).

---

## 9) Open Items / To Decide
- Exact UART baud rates after cable-length testing.
- Whether to add RTS/CTS between STM32 and ESP32.
- Whether motor-node UART links need differential transceivers for long runs (unlikely on compact robot).
- Whether to add a dedicated debug UART if USB-CDC is not sufficient.

---
