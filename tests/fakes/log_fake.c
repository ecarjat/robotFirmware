#include "log.h"

#include <stdarg.h>
#include <stdio.h>

void app_log_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    (void)vprintf(fmt, args);
    va_end(args);
}

void app_log_usb_tx_complete(uint16_t len)
{
    (void)len;
}

void app_log_flush_blocking(uint32_t timeout_ms)
{
    (void)timeout_ms;
}
