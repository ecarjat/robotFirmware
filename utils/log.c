#include "log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "usbd_cdc_if.h"

static uint8_t s_log_ring[APP_LOG_RING_BYTES]
    __attribute__((section(".dma_buffer"), aligned(32)));
static volatile uint16_t s_log_head = 0U;
static volatile uint16_t s_log_tail = 0U;
static volatile uint8_t s_log_tx_busy = 0U;
static volatile uint16_t s_log_tx_len = 0U;

static uint16_t app_log_ring_write(const uint8_t *data, uint16_t len);
static void app_log_usb_kick(void);

void app_log_printf(const char *fmt, ...)
{
    char buffer[APP_LOG_BUFFER_BYTES];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len < 0)
    {
        return;
    }

    size_t bounded_len = (size_t)len;
    if (bounded_len >= sizeof(buffer))
    {
        bounded_len = sizeof(buffer) - 1U;
    }

    /*
     * Always write to ring buffer. Early boot logs before USB enumeration
     * will be buffered and sent once USB is configured.
     */
    (void)app_log_ring_write((const uint8_t *)buffer, (uint16_t)bounded_len);
    app_log_usb_kick();
}

static uint16_t app_log_ring_used(uint16_t head, uint16_t tail)
{
    if (head >= tail)
    {
        return (uint16_t)(head - tail);
    }
    return (uint16_t)(APP_LOG_RING_BYTES - (tail - head));
}

static uint16_t app_log_ring_free(uint16_t head, uint16_t tail)
{
    uint16_t used = app_log_ring_used(head, tail);
    return (uint16_t)((APP_LOG_RING_BYTES - 1U) - used);
}

void app_log_flush_blocking(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (1)
    {
        app_log_usb_kick();
        __disable_irq();
        uint16_t head = s_log_head;
        uint16_t tail = s_log_tail;
        uint8_t busy = s_log_tx_busy;
        __enable_irq();
        if (!busy && head == tail)
        {
            return;
        }
        if ((HAL_GetTick() - start) > timeout_ms)
        {
            return;
        }
    }
}

static uint16_t app_log_ring_write(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U)
    {
        return 0U;
    }

    __disable_irq();
    uint16_t head = s_log_head;
    uint16_t tail = s_log_tail;
    uint16_t free = app_log_ring_free(head, tail);
    if (free == 0U)
    {
        __enable_irq();
        return 0U;
    }
    if (len > free)
    {
        len = free;
    }

    uint16_t first = (uint16_t)(APP_LOG_RING_BYTES - head);
    if (first > len)
    {
        first = len;
    }
    memcpy(&s_log_ring[head], data, first);
    head = (uint16_t)(head + first);
    if (head >= APP_LOG_RING_BYTES)
    {
        head = 0U;
    }

    uint16_t remaining = (uint16_t)(len - first);
    if (remaining > 0U)
    {
        memcpy(&s_log_ring[0], &data[first], remaining);
        head = remaining;
    }

    s_log_head = head;
    __enable_irq();
    return len;
}

static void app_log_usb_kick(void)
{
    /*
     * Only attempt USB transmit if device is configured.
     * Early boot logs remain buffered until USB enumeration completes.
     */
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED)
    {
        return;
    }

    if (s_log_tx_busy)
    {
        return;
    }

    __disable_irq();
    if (s_log_tx_busy)
    {
        __enable_irq();
        return;
    }

    uint16_t tail = s_log_tail;
    uint16_t head = s_log_head;
    if (tail == head)
    {
        __enable_irq();
        return;
    }

    uint16_t len = (head > tail) ? (uint16_t)(head - tail)
                                 : (uint16_t)(APP_LOG_RING_BYTES - tail);
    if (len > APP_LOG_USB_CHUNK_BYTES)
    {
        len = APP_LOG_USB_CHUNK_BYTES;
    }
    s_log_tx_busy = 1U;
    s_log_tx_len = len;
    __enable_irq();

    if (CDC_Transmit_HS(&s_log_ring[tail], len) != USBD_OK)
    {
        __disable_irq();
        s_log_tx_busy = 0U;
        s_log_tx_len = 0U;
        __enable_irq();
    }
}

void app_log_usb_tx_complete(uint16_t len)
{
    (void)len;
    __disable_irq();
    if (!s_log_tx_busy)
    {
        __enable_irq();
        return;
    }

    uint16_t tail = s_log_tail;
    uint16_t advance = s_log_tx_len;
    tail = (uint16_t)(tail + advance);
    if (tail >= APP_LOG_RING_BYTES)
    {
        tail = (uint16_t)(tail - APP_LOG_RING_BYTES);
    }

    s_log_tail = tail;
    s_log_tx_busy = 0U;
    s_log_tx_len = 0U;
    __enable_irq();

    app_log_usb_kick();
}
