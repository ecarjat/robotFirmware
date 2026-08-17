#ifndef BLACKBOX_DUMP_H
#define BLACKBOX_DUMP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize the blackbox dump module
 *
 * Must be called at startup to ensure the dump context is in a known
 * clean state.
 */
void log_dump_init(void);

/**
 * @brief Capture the trailing blackbox window for deferred SD export.
 *
 * Dumps the last N seconds of logged data from the QSPI ring buffer
 * to an SD file (DUMP_0001.BIN, DUMP_0002.BIN, etc.)
 *
 * This function does no filesystem or QSPI read work. It records a capture
 * watermark and the background state machine commits the queued source data.
 * SD export begins only when log_dump_tick() is called with export_allowed.
 *
 * @param seconds Number of seconds to dump (e.g., 30)
 * @return true if capture was accepted, false if a capture is already active,
 *         SD is unavailable, or no log data exists
 */
bool log_dump_last_seconds(uint32_t seconds);

/**
 * @brief Check if a dump is currently in progress
 *
 * @return true if dump is active, false if idle
 */
bool log_is_dumping(void);

/**
 * @brief Background dump tick - call periodically from app_idle_tick()
 *
 * Advances capture processing on every call. Filesystem, QSPI-read, and SD
 * write states advance only when export_allowed is true.
 *
 * @param export_allowed true only when it is safe to run blocking SD export
 */
void log_dump_tick(bool export_allowed);

/**
 * @brief Return whether the active dump is in an export state.
 *
 * Internal logging users use this to pause QSPI producer/consumer work while
 * the exporter reads the protected ring snapshot. Capture-pending states do
 * not block logging.
 */
bool log_dump_blocks_logging(void);

/**
 * @brief Get dump statistics
 *
 * @param records_written Number of records written in current/last dump
 * @param bytes_written Number of bytes written in current/last dump
 * @return true if stats are valid, false if no dump has been performed
 */
bool log_get_dump_stats(uint32_t *records_written, uint32_t *bytes_written);

#ifdef __cplusplus
}
#endif

#endif /* BLACKBOX_DUMP_H */
