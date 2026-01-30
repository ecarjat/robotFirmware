#include "logging/blackbox.h"

void log_push_record(const LogRecord *rec)
{
    (void)rec;
}

bool log_is_initialized(void)
{
    return true;
}

void log_get_stats(log_stats_t *out)
{
    if (out) {
        *out = (log_stats_t){0};
    }
}
