#include "spi_bus.h"

#include <limits.h>
#include <string.h>

#define SPI_BUS_CACHE_LINE_BYTES 32U
#define SPI_BUS_OK               0
#define SPI_BUS_ERR              -1
#define SPI_BUS_BUSY             -2
#define SPI_BUS_ARG              -3
#define SPI_BUS_TIMEOUT          -4
#define SPI_BUS_HAL_TIMEOUT_MS   50U

typedef struct {
    SPI_HandleTypeDef *hspi;
    spi_bus_device_t *active_dev;
    const uint8_t *tx;
    uint8_t *rx;
    size_t len;
    spi_bus_done_cb_t cb;
    void *cb_ctx;
    volatile uint8_t busy;
    uint32_t cpol;
    uint32_t cpha;
    uint32_t prescaler;
    uint32_t datasize;
} spi_bus_state_t;

static spi_bus_state_t s_bus;

typedef struct {
    volatile uint8_t done;
    int status;
} spi_bus_wait_t;

static void spi_bus_wait_cb(void *ctx, int status)
{
    spi_bus_wait_t *state = (spi_bus_wait_t *)ctx;
    if (state == NULL)
    {
        return;
    }
    state->status = status;
    state->done = 1U;
}

static uintptr_t spi_bus_align_down(uintptr_t addr)
{
    return addr & ~(uintptr_t)(SPI_BUS_CACHE_LINE_BYTES - 1U);
}

static uintptr_t spi_bus_align_up(uintptr_t addr)
{
    return (addr + (SPI_BUS_CACHE_LINE_BYTES - 1U)) & ~(uintptr_t)(SPI_BUS_CACHE_LINE_BYTES - 1U);
}

static void spi_bus_cache_clean(const void *buf, size_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (buf == NULL || len == 0U)
    {
        return;
    }
    uintptr_t start = spi_bus_align_down((uintptr_t)buf);
    uintptr_t end = spi_bus_align_up((uintptr_t)buf + len);
    SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
    (void)buf;
    (void)len;
#endif
}

static void spi_bus_cache_invalidate(const void *buf, size_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (buf == NULL || len == 0U)
    {
        return;
    }
    uintptr_t start = spi_bus_align_down((uintptr_t)buf);
    uintptr_t end = spi_bus_align_up((uintptr_t)buf + len);
    SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
    (void)buf;
    (void)len;
#endif
}

static void spi_bus_cache_clean_invalidate(const void *buf, size_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (buf == NULL || len == 0U)
    {
        return;
    }
    uintptr_t start = spi_bus_align_down((uintptr_t)buf);
    uintptr_t end = spi_bus_align_up((uintptr_t)buf + len);
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
    (void)buf;
    (void)len;
#endif
}

static void spi_bus_assert_cs(const spi_bus_device_t *dev)
{
    if (dev == NULL)
    {
        return;
    }
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void spi_bus_deassert_cs(const spi_bus_device_t *dev)
{
    if (dev == NULL)
    {
        return;
    }
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static int spi_bus_apply_config(const spi_bus_device_t *dev)
{
    if (s_bus.hspi == NULL || dev == NULL)
    {
        return SPI_BUS_ARG;
    }

    if (s_bus.cpol == dev->cpol &&
        s_bus.cpha == dev->cpha &&
        s_bus.prescaler == dev->prescaler &&
        s_bus.datasize == dev->datasize)
    {
        return SPI_BUS_OK;
    }

    s_bus.hspi->Init.CLKPolarity = dev->cpol;
    s_bus.hspi->Init.CLKPhase = dev->cpha;
    s_bus.hspi->Init.BaudRatePrescaler = dev->prescaler;
    s_bus.hspi->Init.DataSize = dev->datasize;

    if (HAL_SPI_Init(s_bus.hspi) != HAL_OK)
    {
        return SPI_BUS_ERR;
    }

    s_bus.cpol = dev->cpol;
    s_bus.cpha = dev->cpha;
    s_bus.prescaler = dev->prescaler;
    s_bus.datasize = dev->datasize;
    return SPI_BUS_OK;
}

static void spi_bus_finish(int status)
{
    spi_bus_done_cb_t cb = s_bus.cb;
    void *cb_ctx = s_bus.cb_ctx;
    spi_bus_device_t *dev = s_bus.active_dev;
    uint8_t *rx = s_bus.rx;
    size_t len = s_bus.len;

    s_bus.cb = NULL;
    s_bus.cb_ctx = NULL;
    s_bus.active_dev = NULL;
    s_bus.tx = NULL;
    s_bus.rx = NULL;
    s_bus.len = 0U;
    s_bus.busy = 0U;

    spi_bus_deassert_cs(dev);
    spi_bus_cache_invalidate(rx, len);

    if (cb != NULL)
    {
        cb(cb_ctx, status);
    }
}

int spi_bus_init(SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL)
    {
        return SPI_BUS_ARG;
    }
    memset(&s_bus, 0, sizeof(s_bus));
    s_bus.hspi = hspi;
    s_bus.cpol = hspi->Init.CLKPolarity;
    s_bus.cpha = hspi->Init.CLKPhase;
    s_bus.prescaler = hspi->Init.BaudRatePrescaler;
    s_bus.datasize = hspi->Init.DataSize;
    return SPI_BUS_OK;
}

void spi_bus_device_init(spi_bus_device_t *dev,
                         GPIO_TypeDef *cs_port,
                         uint16_t cs_pin,
                         uint32_t cpol,
                         uint32_t cpha,
                         uint32_t prescaler,
                         uint32_t datasize)
{
    if (dev == NULL)
    {
        return;
    }
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    dev->cpol = cpol;
    dev->cpha = cpha;
    dev->prescaler = prescaler;
    dev->datasize = datasize;

    if (cs_port != NULL)
    {
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    }
}

int spi_bus_transfer_dma(spi_bus_device_t *dev,
                         const uint8_t *tx,
                         uint8_t *rx,
                         size_t len,
                         spi_bus_done_cb_t cb,
                         void *ctx)
{
    if (dev == NULL || tx == NULL || rx == NULL || len == 0U)
    {
        return SPI_BUS_ARG;
    }
    if (len > UINT16_MAX)
    {
        return SPI_BUS_ARG;
    }
    if (s_bus.busy)
    {
        return SPI_BUS_BUSY;
    }

    if (spi_bus_apply_config(dev) != SPI_BUS_OK)
    {
        return SPI_BUS_ERR;
    }

    s_bus.busy = 1U;
    s_bus.active_dev = dev;
    s_bus.tx = tx;
    s_bus.rx = rx;
    s_bus.len = len;
    s_bus.cb = cb;
    s_bus.cb_ctx = ctx;

    spi_bus_cache_clean(tx, len);
    spi_bus_cache_clean_invalidate(rx, len);

    spi_bus_assert_cs(dev);
    if (HAL_SPI_TransmitReceive_DMA(s_bus.hspi, (uint8_t *)tx, rx, (uint16_t)len) != HAL_OK)
    {
        spi_bus_deassert_cs(dev);
        s_bus.busy = 0U;
        s_bus.active_dev = NULL;
        s_bus.tx = NULL;
        s_bus.rx = NULL;
        s_bus.len = 0U;
        s_bus.cb = NULL;
        s_bus.cb_ctx = NULL;
        return SPI_BUS_ERR;
    }

    return SPI_BUS_OK;
}

int spi_bus_transfer(spi_bus_device_t *dev,
                     const uint8_t *tx,
                     uint8_t *rx,
                     size_t len,
                     uint32_t timeout_ms)
{
    spi_bus_wait_t wait_state = {0U, SPI_BUS_ERR};
    int rc = spi_bus_transfer_dma(dev, tx, rx, len, spi_bus_wait_cb, &wait_state);

    if (rc != SPI_BUS_OK)
    {
        return rc;
    }

    uint32_t start = HAL_GetTick();
    while (!wait_state.done)
    {
        if (timeout_ms > 0U && (HAL_GetTick() - start) > timeout_ms)
        {
            (void)HAL_SPI_Abort(s_bus.hspi);
            if (s_bus.busy)
            {
                spi_bus_finish(SPI_BUS_TIMEOUT);
            }
            return SPI_BUS_TIMEOUT;
        }
    }

    return wait_state.status;
}

int spi_bus_transfer_blocking(spi_bus_device_t *dev,
                              const uint8_t *tx,
                              uint8_t *rx,
                              size_t len,
                              uint32_t timeout_ms)
{
    if (dev == NULL || tx == NULL || rx == NULL || len == 0U)
    {
        return SPI_BUS_ARG;
    }
    if (len > UINT16_MAX)
    {
        return SPI_BUS_ARG;
    }
    if (s_bus.busy)
    {
        return SPI_BUS_BUSY;
    }

    if (spi_bus_apply_config(dev) != SPI_BUS_OK)
    {
        return SPI_BUS_ERR;
    }

    s_bus.busy = 1U;
    s_bus.active_dev = dev;

    spi_bus_assert_cs(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(s_bus.hspi,
                                                   (uint8_t *)tx,
                                                   rx,
                                                   (uint16_t)len,
                                                   (timeout_ms > 0U) ? timeout_ms : SPI_BUS_HAL_TIMEOUT_MS);
    spi_bus_deassert_cs(dev);

    s_bus.busy = 0U;
    s_bus.active_dev = NULL;

    return (st == HAL_OK) ? SPI_BUS_OK : SPI_BUS_ERR;
}

int spi_bus_is_busy(void)
{
    return s_bus.busy ? 1 : 0;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == s_bus.hspi && s_bus.busy)
    {
        spi_bus_finish(SPI_BUS_OK);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == s_bus.hspi && s_bus.busy)
    {
        spi_bus_finish(SPI_BUS_ERR);
    }
}
