#ifndef SPI_BUS_H
#define SPI_BUS_H

#include <stddef.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#define SPI_BUS_OK      0
#define SPI_BUS_ERR     -1
#define SPI_BUS_BUSY    -2
#define SPI_BUS_ARG     -3
#define SPI_BUS_TIMEOUT -4

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    uint32_t cpol;
    uint32_t cpha;
    uint32_t prescaler;
    uint32_t datasize;
} spi_bus_device_t;

typedef void (*spi_bus_done_cb_t)(void *ctx, int status);

int spi_bus_init(SPI_HandleTypeDef *hspi);
void spi_bus_device_init(spi_bus_device_t *dev,
                         GPIO_TypeDef *cs_port,
                         uint16_t cs_pin,
                         uint32_t cpol,
                         uint32_t cpha,
                         uint32_t prescaler,
                         uint32_t datasize);
int spi_bus_transfer_dma(spi_bus_device_t *dev,
                         const uint8_t *tx,
                         uint8_t *rx,
                         size_t len,
                         spi_bus_done_cb_t cb,
                         void *ctx);
int spi_bus_transfer(spi_bus_device_t *dev,
                     const uint8_t *tx,
                     uint8_t *rx,
                     size_t len,
                     uint32_t timeout_ms);
int spi_bus_transfer_blocking(spi_bus_device_t *dev,
                              const uint8_t *tx,
                              uint8_t *rx,
                              size_t len,
                              uint32_t timeout_ms);
void spi_bus_abort(void);
int spi_bus_is_busy(void);

#ifdef __cplusplus
}
#endif

#endif /* SPI_BUS_H */
