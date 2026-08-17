#include "logging/blackbox_dump.h"
#include "blackbox_dump_fake.h"

static uint32_t s_capture_requests;
static uint32_t s_tick_calls;

void blackbox_dump_fake_reset(void)
{
    s_capture_requests = 0U;
    s_tick_calls = 0U;
}

uint32_t blackbox_dump_fake_capture_requests(void)
{
    return s_capture_requests;
}

uint32_t blackbox_dump_fake_tick_calls(void)
{
    return s_tick_calls;
}

bool log_dump_last_seconds(uint32_t seconds)
{
    (void)seconds;
    s_capture_requests++;
    return true;
}

bool log_is_dumping(void)
{
    return false;
}

void log_dump_init(void)
{
}

void log_dump_tick(bool export_allowed)
{
    (void)export_allowed;
    s_tick_calls++;
}

bool log_dump_blocks_logging(void)
{
    return false;
}

bool log_get_dump_stats(uint32_t *records_written, uint32_t *bytes_written)
{
    if (records_written) {
        *records_written = 0;
    }
    if (bytes_written) {
        *bytes_written = 0;
    }
    return true;
}
