# PIN_MAPPING.md — WeAct STM32H723VGT6 (WeAct STM32H7xx Board V1.2) Robot Wiring
Version: 1.1 (Primary IMU = BMI270)  
Audience: Codex + embedded firmware implementation (STM32CubeIDE/HAL or LL)

## 0) Goal
Concrete, conflict-free pin/port mapping for the robot architecture:

- **STM32H723VGT6 (WeAct board)** = robot brain (motors, IMUs, LiDARs, logging SD, TFT, real-time control)
- **ESP32** = Xbox controller (BT) + Wi-Fi portal (logs/telemetry/firmware)
- **Raspberry Pi (future autonomy)** = connects to STM32 **via USB** (CDC-ACM) for commands + telemetry + file API; Pi hosts 2D LiDAR + vision locally.

Constraints:
- Do not reuse pins consumed by **on-board TFT**, **microSD**, **USB FS**, **QSPI flash**, **SWD**.
- Use **two separate I2C buses** for IMUs/mag.
- Use **UART for all 3 TFmini Plus**.
- Use **4 dedicated UART links** for 4 motor driver nodes.
- Use **1 UART spine** between STM32 and ESP32 @ 921600 + optional RTS/CTS.
- USB FS port is reserved for STM32↔Pi (CDC + FILE API).

Source: Board schematic (WeAct STM32H7xx V1.2).  [oai_citation:0‡STM32H7xx SchDoc V12.pdf](sediment://file_00000000dcfc720a8372515e6a1473dc)

---

## 1) Reserved On-board Pins (DO NOT ASSIGN)
### 1.1 TFT (ST7735 on-board)
- PE14 = LCD_SDA (MOSI)
- PE13 = LCD_WR_RS (D/C)
- PE12 = LCD_SCL (SCK)
- PE11 = LCD_CS
- PE10  = LCD_LED

### 1.2 microSD (SDMMC1 on-board)
- PC8  = SDMMC1_D0
- PC9  = SDMMC1_D1
- PC10 = SDMMC1_D2
- PC11 = SDMMC1_D3
- PC12 = SDMMC1_CK
- PD2  = SDMMC1_CMD

### 1.3 USB FS (wired to USB port)
- PA11 = USB1_DN
- PA12 = USB1_DP  

### 1.4 QSPI Flash (on-board)
- PB2  = QSPI_CLK
- PB6  = QSPI_BK1_NCS
- PD11 = QSPI_BK1_IO0  
- PD12  = QSPI_BK1_IO1
- PE2 = QSPI_BK1_IO2
- PD13 = QSPI_BK1_IO3

### 1.5 SPI Flash (on-board)
- PB4  = SPIx_MISO
- PD7  = SPIx_MOSI
- PB3  = SPIx_CLK
- PD11 = SPIx_CS

### 1.6 SWD (debug)
- PA13 = SWDIO
- PA14 = SWCLK  

---

## 2) I2C Buses (split IMUs across two buses)
### 2.1 I2C1 — Primary IMU bus (cleanest, highest priority)
- PB8 = I2C1_SCL
- PB7 = I2C1_SDA  

Devices on I2C1:
- **BMI270 (IMU #1, PRIMARY)**

Suggested interrupts (EXTI-capable GPIOs; can be changed):
- PC3 = BMI270_INT1

### 2.2 I2C2 — Secondary IMU + Magnetometer bus
- PB10 = I2C2_SCL (set AF in CubeMX)
- PB11 = I2C2_SDA (set AF in CubeMX)

Devices on I2C2:
- **ICM-45686 (IMU #2, SECONDARY)**
- **BMM150  (Magnetometer)**

Suggested interrupts:
- PC0 = ICM45686_INT1
- PC2 = ICM45686_INT2

Electrical:
- Each I2C bus has its own pull-ups to 3.3V (start 2.2k–4.7k).
- Start at 400 kHz; drop to 100 kHz if errors.

---

## 3) STM32 ↔ ESP32 Spine UART (teleop + portal bridge)
### 3.1 Link UART (921600 baud)
- PA2 = USART2_TX  (STM32 → ESP32 RX)
- PA3 = USART2_RX  (ESP32 → STM32 TX)  

### 3.2 hardware flow control
- PA1 = USART2_RTS
- PA0 = USART2_CTS  

Notes:
- RX via DMA circular + IDLE interrupt.
- TX via DMA queue.
- Protocol: COBS(or SLIP)+CRC32 multiplex channels CMD/TELEM/FILE/RPC.

---

## 4) Raspberry Pi Link (future autonomy) — USB CDC
### 4.1 Physical pins (fixed)
- PA11/PA12 already wired to USB FS port (reserved).  

Firmware requirements:
- STM32 exposes **USB CDC-ACM** device.
- Over CDC, STM32 supports multiplex channels CMD/TELEM/FILE/RPC.
- FILE channel supports listing logs and reading SD file chunks.

---

## 5) Motor Drivers (4× UART links)
Motor protocol must remain compatible with:
https://github.com/ecarjat/T-Storm32NT-simpleFoc/blob/main/README.md

### 5.1 Motor #1 UART
- PB14  = USART1_TX
- PB15 = USART1_RX  
- DMA
### 5.2 Motor #2 UART (DCMI pins repurposed; camera unused)
- PC6 = USART6_TX
- PC7 = USART6_RX  
- DMA
### 5.3 Motor #3 UART (DCMI pins repurposed; camera unused)
- PD8 = USART3_TX
- PD9 = USART3_RX  
- DMA
### 5.4 Motor #4 UART (DCMI pins repurposed; camera unused)
- PD0 = UART4_RX
- PD1 = UART4_TX  
- DMA


---

## 6) LiDARs (3× TFmini Plus on UART)
### 6.1 LiDAR Front (UART)
- PB12 = UART5_RX
- PB13 = UART5_TX (optional)  
- RX DMA

### 6.2 LiDAR Back (UART)
- PE7 = UART7_RX
- PE8 = UART7_TX (optional)  
- RX DMA

### 6.3 LiDAR Down (UART)
- PE0 = UART8_RX
- PE1 = UART8_TX (optional)  

Firmware:
- RX via DMA circular + IDLE interrupt.
- Parse TFmini frames; update distance+timestamp.

---

## 7) LEDs
- PE3  = on-board BLUE_LED 
- PB1 = external LED_GREEN
- PB0  = external LED_RED  

---

## 8) Buttons 
- PA4 = BTN_ARM
- PE4  = BTN_MODE
- PE5  = BTN_CAL
- PE6  = BTN_ESTOP  

- BOOT0 = SW1
- PC13 = SW2
- NRST = SW3
---

## 9) Codex instructions (must follow)
1) Use this mapping as source-of-truth.
2) Generate CubeMX/HAL init for:
   - I2C1 
   - I2C2 
   - USART2 (ESP32 spine)  +  RTS/CTS
   - Motor UARTs 
   - LiDAR UARTs u
   - USB FS CDC-ACM 
   - Preserve SDMMC1 and TFT wiring; do not reconfigure their pins
