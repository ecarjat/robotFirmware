#ifndef BLACKBOX_H
#define BLACKBOX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "blackbox_format.h"
#include "param_storage.h"
#include <stdbool.h>
#include <stdint.h>

/* Configuration */
/* Queue size MUST be a multiple of LOG_RECORD_SIZE (160) to avoid wrap issues */
#define LOG_RAM_QUEUE_RECORDS  1600U      /* ~4s @ 400Hz */
#define LOG_RAM_QUEUE_SIZE  (LOG_RAM_QUEUE_RECORDS * 160U)  /* 256,000 bytes */
#define LOG_WRITE_CHUNK_SIZE  4096U       /* Write in 4 KB chunks (page-aligned) */
#define LOG_PREERASE_AHEAD  8U            /* Sectors to pre-erase ahead (32 KB) */
#define LOG_META_UPDATE_PERIOD_MS  1000U  /* Update meta every 1 second */

/**
 * @brief Logging statistics
 */
typedef struct {
  uint32_t dropped_records;      /* Records dropped due to queue full */
  uint32_t qspi_write_errors;    /* QSPI write operation failures */
  uint32_t erase_lag_sectors;    /* How many sectors behind erase is */
  uint32_t fill_seconds;         /* Estimated seconds of data in ring */
  uint32_t total_records;        /* Total records written since boot */
  uint32_t wrap_count;           /* Number of ring buffer wraps */
} log_stats_t;

/**
 * @brief Result of polling a capture watermark.
 *
 * A capture watermark is used by the dump module to freeze an exact end point
 * in the QSPI ring without blocking the control loop while queued records are
 * committed.
 */
typedef enum {
  LOG_CAPTURE_PENDING = 0,
  LOG_CAPTURE_READY,
  LOG_CAPTURE_FAILED
} log_capture_state_t;

/**
 * @brief Immutable source boundary for a deferred dump.
 *
 * end_addr and end_seq are exclusive: they identify the location and record
 * sequence immediately after the last record accepted by the capture.
 */
typedef struct {
  uint32_t end_addr;
  uint32_t end_seq;
  uint32_t records_available;
} log_capture_snapshot_t;

/**
 * @brief Initialize blackbox logging system
 *
 * - Reads/validates meta from flash (dual-slot recovery)
 * - Initializes RAM queue
 * - Sets up background tasks
 *
 * Must be called after:
 *  - HAL init
 *  - QSPI init (MX_OCTOSPI1_Init)
 *  - SD/FatFS init (MX_FATFS_Init)
 *  - Parameter load (robot_params_t)
 *
 * @param params Pointer to robot parameters
 */
void log_init(const robot_params_t *params);

/**
 * @brief Check if blackbox logging is initialized
 * @return true if initialized, false otherwise
 */
bool log_is_initialized(void);

/**
 * @brief Push a log record to the RAM queue
 *
 * Called from control loop (400Hz). Must be fast and non-blocking.
 * If queue is full, increments dropped_records counter.
 *
 * @param rec Pointer to log record (will be copied)
 */
void log_push_record(const LogRecord *rec);

/**
 * @brief Background tick - write queued records to QSPI
 *
 * Should be called from app_idle_tick() (50ms period).
 * Coalesces records into 4KB chunks and writes to flash.
 * Updates meta periodically.
 */
void log_writer_tick(void);

/**
 * @brief Background tick - pre-erase sectors ahead of write pointer
 *
 * Should be called from app_idle_tick() (50ms period).
 * Keeps LOG_PREERASE_AHEAD sectors erased in advance.
 */
void log_erase_tick(void);

/**
 * @brief Flush queued records to QSPI (best-effort, bounded time)
 *
 * @param timeout_ms Maximum time to spend flushing
 * @return true if queue drained and no pending write, false otherwise
 */
bool log_flush_pending(uint32_t timeout_ms);

/**
 * @brief Get logging statistics
 *
 * @param out Pointer to stats structure
 */
void log_get_stats(log_stats_t *out);

/**
 * @brief Get current write position in ring buffer
 *
 * Used by dump module to determine latest record location.
 *
 * @return Current write address in flash
 */
uint32_t log_get_write_addr(void);

/**
 * @brief Get the write position in the ring buffer at boot time
 *
 * @return Write address at boot
 */
uint32_t log_get_boot_write_addr(void);

/**
 * @brief Get current sequence number
 *
 * @return Next sequence number to be assigned
 */
uint32_t log_get_seq(void);

/**
 * @brief Get active log fields mask
 *
 * @return Bitmask of LOGF_* fields
 */
uint32_t log_get_fields_mask(void);

/**
 * @brief Get configured log rate in Hz
 *
 * @return Log rate in Hz
 */
uint16_t log_get_rate_hz(void);

/**
 * @brief Start a non-blocking capture of all records accepted so far.
 *
 * The logger flushes its current partial chunk through the capture boundary,
 * but does not consume records accepted after it. Poll with
 * log_capture_poll() until the source boundary is ready.
 */
bool log_capture_begin(void);

/**
 * @brief Poll a capture watermark.
 *
 * @param[out] snapshot Filled when LOG_CAPTURE_READY is returned.
 */
log_capture_state_t log_capture_poll(log_capture_snapshot_t *snapshot);

/**
 * @brief Protect a captured trailing window from ring overwrite.
 *
 * Once normal logging reaches the retention boundary, later records are
 * dropped without blocking the control loop until log_capture_release().
 */
bool log_capture_protect(uint32_t records_to_protect);

/**
 * @brief Release a capture watermark and any retention hold.
 */
void log_capture_release(void);

#ifdef __cplusplus
}
#endif

#endif /* BLACKBOX_H */
