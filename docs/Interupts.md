# Interrupt Priorities

This document lists interrupt priorities configured in the firmware and the rationale
where it is known. When no rationale exists in code comments or design docs, it is
explicitly called out as undocumented.

## Priority Grouping

- NVIC grouping: `NVIC_PRIORITYGROUP_4` (configured in `Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c`).
  This means 4 bits of preempt priority and 0 bits of subpriority.

## Configured IRQ Priorities

| IRQ | Preempt Priority | Subpriority | Source | Rationale |
| --- | --- | --- | --- | --- |
| `OTG_HS_IRQn` | 0 | 0 | `USB_DEVICE/Target/usbd_conf.c` | USB device ISR; CubeMX default, no project-specific rationale documented. |
| `I2C1_EV_IRQn` | 0 | 0 | `Core/Src/stm32h7xx_hal_msp.c` | I2C event ISR; CubeMX default, no project-specific rationale documented. |
| `I2C1_ER_IRQn` | 0 | 0 | `Core/Src/stm32h7xx_hal_msp.c` | I2C error ISR; CubeMX default, no project-specific rationale documented. |
| `OCTOSPI1_IRQn` | 0 | 0 | `Core/Src/stm32h7xx_hal_msp.c` | QSPI/OSPI ISR; CubeMX default, no project-specific rationale documented. |
| `SDMMC1_IRQn` | 6 | 0 | `Core/Src/stm32h7xx_hal_msp.c` | Lower priority than most other ISRs; rationale for value 6 not documented. |
| `SPI6_IRQn` | 0 | 0 | `Core/Src/stm32h7xx_hal_msp.c` | SPI6 ISR (IMU bus); CubeMX default, no project-specific rationale documented. |
| `TIM2_IRQn` | 5 | 0 | `Core/Src/stm32h7xx_hal_msp.c` | Control timer ISR (TIM2); mid priority to keep control timing responsive while not using priority 0. |
| `UART4_IRQn` | 0 | 0 | `Core/Src/stm32h7xx_hal_msp.c` | UART4 ISR; CubeMX default, no project-specific rationale documented. |
| `USART1_IRQn` | 0 | 0 | `Core/Src/stm32h7xx_hal_msp.c` | USART1 ISR; CubeMX default, no project-specific rationale documented. |
| `USART2_IRQn` | 0 | 0 | `Core/Src/stm32h7xx_hal_msp.c` | USART2 ISR; CubeMX default, no project-specific rationale documented. |
| `USART3_IRQn` | 0 | 0 | `Core/Src/stm32h7xx_hal_msp.c` | USART3 ISR; CubeMX default, no project-specific rationale documented. |
| `USART6_IRQn` | 0 | 0 | `Core/Src/stm32h7xx_hal_msp.c` | USART6 ISR; CubeMX default, no project-specific rationale documented. |
| `BDMA_Channel0_IRQn` | 0 | 0 | `Core/Src/main.c` | BDMA interrupt; CubeMX default, no project-specific rationale documented. |
| `BDMA_Channel1_IRQn` | 0 | 0 | `Core/Src/main.c` | BDMA interrupt; CubeMX default, no project-specific rationale documented. |
| `DMA1_Stream4_IRQn` | 0 | 0 | `Core/Src/main.c` | DMA1 stream interrupt; CubeMX default, no project-specific rationale documented. |
| `DMA1_Stream5_IRQn` | 0 | 0 | `Core/Src/main.c` | DMA1 stream interrupt; CubeMX default, no project-specific rationale documented. |
| `DMA1_Stream6_IRQn` | 0 | 0 | `Core/Src/main.c` | DMA1 stream interrupt; CubeMX default, no project-specific rationale documented. |
| `DMA1_Stream7_IRQn` | 0 | 0 | `Core/Src/main.c` | DMA1 stream interrupt; CubeMX default, no project-specific rationale documented. |
| `DMA2_Stream0_IRQn` | 0 | 0 | `Core/Src/main.c` | DMA2 stream interrupt; CubeMX default, no project-specific rationale documented. |
| `DMA2_Stream1_IRQn` | 0 | 0 | `Core/Src/main.c` | DMA2 stream interrupt; CubeMX default, no project-specific rationale documented. |
| `DMA2_Stream2_IRQn` | 0 | 0 | `Core/Src/main.c` | DMA2 stream interrupt; CubeMX default, no project-specific rationale documented. |
| `DMA2_Stream3_IRQn` | 0 | 0 | `Core/Src/main.c` | DMA2 stream interrupt; CubeMX default, no project-specific rationale documented. |
| `DMA2_Stream4_IRQn` | 0 | 0 | `Core/Src/main.c` | DMA2 stream interrupt; CubeMX default, no project-specific rationale documented. |
| `DMA2_Stream5_IRQn` | 0 | 0 | `Core/Src/main.c` | DMA2 stream interrupt; CubeMX default, no project-specific rationale documented. |
| `DMA2_Stream6_IRQn` | 0 | 0 | `Core/Src/main.c` | DMA2 stream interrupt; CubeMX default, no project-specific rationale documented. |
| `DMA2_Stream7_IRQn` | 0 | 0 | `Core/Src/main.c` | DMA2 stream interrupt; CubeMX default, no project-specific rationale documented. |
| `MDMA_IRQn` | 0 | 0 | `Core/Src/main.c` | MDMA interrupt; CubeMX default, no project-specific rationale documented. |
| `ICM42688_INT1_EXTI_IRQn` | 0 | 0 | `Core/Src/main.c` | IMU data-ready EXTI; CubeMX default, no project-specific rationale documented. |
| `BMI270_INT1_EXTI_IRQn` | 0 | 0 | `Core/Src/main.c` | IMU data-ready EXTI; CubeMX default, no project-specific rationale documented. |
| `BMM150_INT1_EXTI_IRQn` | 0 | 0 | `Core/Src/main.c` | IMU data-ready EXTI; CubeMX default, no project-specific rationale documented. |

## Notes

- If additional interrupts are configured later (e.g., new peripherals or FreeRTOS),
  add them here with a short rationale or a TODO for rationale.
