#include "logging/blackbox_dump.h"

bool log_dump_last_seconds(uint32_t seconds)
{
    (void)seconds;
    return true;
}

bool log_is_dumping(void)
{
    return false;
}

void log_dump_init(void)
{
}

void log_dump_tick(void)
{
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
