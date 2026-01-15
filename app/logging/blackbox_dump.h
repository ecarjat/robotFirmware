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
 * @brief Initiate a blackbox dump to SD card
 *
 * Dumps the last N seconds of logged data from the QSPI ring buffer
 * to an SD file (DUMP_0001.BIN, DUMP_0002.BIN, etc.)
 *
 * This function is non-blocking - it starts the dump process which
 * runs cooperatively in the background via log_dump_tick().
 *
 * @param seconds Number of seconds to dump (e.g., 30)
 * @return true if dump was initiated, false if already dumping or SD not ready
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
 * Advances the dump state machine. Should be called regularly
 * (e.g., every 50ms in the idle loop) to make progress on dumps.
 */
void log_dump_tick(void);

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
