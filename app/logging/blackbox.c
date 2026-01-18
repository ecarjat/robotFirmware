#include "blackbox.h"

#include "app_config.h"
#include "crc32.h"
#include "main.h"
#include "blackbox_dump.h"
#include "qspi_w25q64.h"
#include "config_control.h"
#include <limits.h>
#include <string.h>

/*
 * Lock-free SPSC (Single Producer Single Consumer) Queue
 * -------------------------------------------------------
 * Producer: log_push_record() - called from control loop (may be ISR context)
 * Consumer: log_writer_tick() - called from main loop only
 *
 * Safety guarantees:
 * - Producer only writes s_queue_head, consumer only reads it
 * - Consumer only writes s_queue_tail, producer only reads it
 * - Memory barriers ensure data visibility before/after index updates
 * - No interrupt disabling needed for SPSC queues
 */

/* Memory barrier macros for ARM Cortex-M7 */
#define LOG_QUEUE_WRITE_BARRIER()  __DMB()  /* Ensure writes complete before index update */
#define LOG_QUEUE_READ_BARRIER()   __DMB()  /* Ensure index read before data read */

/* RAM queue for log records (in .dma_buffer section) */
static uint8_t s_log_queue[LOG_RAM_QUEUE_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));

/* SPSC queue indices - producer writes head, consumer writes tail */
static volatile uint32_t s_queue_head = 0;  /* Written by producer only */
static volatile uint32_t s_queue_tail = 0;  /* Written by consumer only */
static volatile uint32_t s_dropped_records = 0;

/* Write state */
static uint32_t s_write_addr = LOG_RING_START;
static uint32_t s_write_addr_at_boot = LOG_RING_START;
static uint32_t s_wrap_count = 0;
static volatile uint32_t s_next_seq = 0;
static uint32_t s_total_records = 0;

/* Erase state */
static uint32_t s_next_erase_addr = LOG_RING_START;

/* Meta update state */
static uint32_t s_last_meta_update_ms = 0;
static uint32_t s_boot_count = 0;
static uint32_t s_last_dump_id = 0;
static uint32_t s_meta_sequence = 0;

typedef enum {
  LOG_META_SAVE_IDLE = 0,
  LOG_META_SAVE_ERASE,
  LOG_META_SAVE_WRITE
} log_meta_save_state_t;

typedef enum {
  LOG_META_OP_NONE = 0,
  LOG_META_OP_ERASE,
  LOG_META_OP_WRITE
} log_meta_op_t;

static log_meta_save_state_t s_meta_save_state = LOG_META_SAVE_IDLE;
static log_meta_op_t s_meta_op_inflight = LOG_META_OP_NONE;
static uint32_t s_meta_slot_addr = LOG_META_SLOT0;
static LogMeta s_meta_pending
    __attribute__((section(".dma_buffer"), aligned(32)));

/* Statistics */
static uint32_t s_qspi_write_errors = 0;

/* Active log fields mask */
static uint32_t s_log_fields_mask = LOG_FIELDS_MASK_DEFAULT;
static uint16_t s_log_rate_hz = 0U;

/* Write chunk buffer (in .dma_buffer section) */
static uint8_t s_write_chunk[LOG_WRITE_CHUNK_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));
static size_t s_write_chunk_len = 0;

typedef enum {
  LOG_WRITE_IDLE = 0,
  LOG_WRITE_FIRST,
  LOG_WRITE_SECOND
} log_write_state_t;

static log_write_state_t s_write_state = LOG_WRITE_IDLE;
static uint32_t s_write_chunk_start_addr = LOG_RING_START;
static size_t s_write_chunk_first_len = 0;
static size_t s_write_chunk_second_len = 0;

/* Initialization flag */
static bool s_initialized = false;

/* Internal functions */
static bool log_load_meta(void);
static bool log_save_meta(void);
static bool log_validate_meta(const LogMeta *meta);
static bool log_validate_ring_tail(void);
static void log_format_meta(void);
static bool log_flush_write_chunk(void);
static void log_advance_write_addr_after_chunk(void);
static uint16_t log_rate_from_params(const robot_params_t *params);
static void log_meta_tick(qspi_w25q64_async_state_t async_state);
static bool log_erase_ahead_ready(void);

void log_init(const robot_params_t *params) {
  if (s_initialized) {
    return;
  }

  /* Initialize QSPI flash driver */
  qspi_w25q64_init();

  if (!qspi_w25q64_is_ready()) {
    /* Flash not responding - cannot initialize logging */
    return;
  }

  /* Load configuration from parameters */
  s_log_fields_mask = params->log_fields_mask;
  if (s_log_fields_mask == 0) {
    s_log_fields_mask = LOG_FIELDS_MASK_DEFAULT;
  }
  s_log_rate_hz = log_rate_from_params(params);

  /* Try to load existing metadata */
  if (!log_load_meta()) {
    /* No valid meta found - format flash */
    log_format_meta();
  }

  if (!log_validate_ring_tail()) {
    APP_LOG_WARN("Log ring invalid - resetting write pointer");
    log_format_meta();
  }

  /* Increment boot count and save */
  s_boot_count++;
  log_save_meta();

  /* Store initial write address for session-based dump calculations */
  s_write_addr_at_boot = s_write_addr;
  s_next_erase_addr = s_write_addr & ~(W25Q64_SECTOR_SIZE - 1);

  s_initialized = true;
}

bool log_is_initialized(void) {
  return s_initialized;
}

void log_push_record(const LogRecord *rec) {
  if (!s_initialized || rec == NULL) {
    return;
  }

  /*
   * SPSC Producer: Only this function writes s_queue_head.
   * Consumer (log_writer_tick) only reads s_queue_head.
   * We read s_queue_tail to check for full condition (safe - consumer writes it).
   */

  const size_t record_size = LOG_RECORD_SIZE;
  uint32_t head = s_queue_head;
  uint32_t next_head = (head + record_size) % LOG_RAM_QUEUE_SIZE;

  /* Read tail once to check if queue is full */
  uint32_t tail = s_queue_tail;
  if (next_head == tail) {
    s_dropped_records++;
    return;
  }

  /* Copy record into queue */
  if (head + record_size <= LOG_RAM_QUEUE_SIZE) {
    /* Contiguous copy */
    memcpy(&s_log_queue[head], rec, record_size);
  } else {
    /* Wrap-around copy */
    size_t first_part = LOG_RAM_QUEUE_SIZE - head;
    memcpy(&s_log_queue[head], rec, first_part);
    memcpy(&s_log_queue[0], (const uint8_t *)rec + first_part,
           record_size - first_part);
  }

  /* Update sequence number (for dumper) */
  s_next_seq = rec->seq + 1;

  /* Write barrier: ensure data is visible before head update */
  LOG_QUEUE_WRITE_BARRIER();

  /* Publish new head (atomic 32-bit write on ARM) */
  s_queue_head = next_head;
}

void log_writer_tick(void) {
  if (!s_initialized) {
    return;
  }

  qspi_w25q64_async_state_t async_state = qspi_w25q64_write_async_tick();

  if (s_write_state != LOG_WRITE_IDLE) {
    if (async_state == QSPI_W25Q64_ASYNC_ERROR) {
      s_qspi_write_errors++;
      s_write_chunk_len = 0;
      s_write_chunk_first_len = 0;
      s_write_chunk_second_len = 0;
      s_write_state = LOG_WRITE_IDLE;
    } else if (async_state == QSPI_W25Q64_ASYNC_DONE) {
      if (s_write_state == LOG_WRITE_FIRST && s_write_chunk_second_len > 0) {
        if (!qspi_w25q64_write_async_start(
                LOG_RING_START, s_write_chunk + s_write_chunk_first_len,
                s_write_chunk_second_len)) {
          s_qspi_write_errors++;
          s_write_chunk_len = 0;
          s_write_chunk_first_len = 0;
          s_write_chunk_second_len = 0;
          s_write_state = LOG_WRITE_IDLE;
        } else {
          s_write_state = LOG_WRITE_SECOND;
          return;
        }
      } else {
        /* Count records once the full chunk is successfully written. */
        s_total_records += (uint32_t)(s_write_chunk_len / LOG_RECORD_SIZE);
        log_advance_write_addr_after_chunk();
        s_write_chunk_len = 0;
        s_write_chunk_first_len = 0;
        s_write_chunk_second_len = 0;
        s_write_state = LOG_WRITE_IDLE;
      }
    }
  }

  log_meta_tick(async_state);
  if (s_meta_op_inflight != LOG_META_OP_NONE) {
    return;
  }

  if (log_is_dumping()) {
    return;
  }

  /* Check if flash is busy */
  if (qspi_w25q64_is_busy()) {
    return;
  }

  if (!log_erase_ahead_ready()) {
    log_erase_tick();
    if (!log_erase_ahead_ready()) {
      return;
    }
  }

  if (s_write_state != LOG_WRITE_IDLE) {
    return;
  }

  /*
   * SPSC Consumer: Only this function writes s_queue_tail.
   * Producer (log_push_record) only reads s_queue_tail.
   * We read s_queue_head to check for empty condition (safe - producer writes it).
   */
  uint32_t tail = s_queue_tail;

  while (1) {
    /* Read head with barrier to ensure we see producer's data */
    uint32_t head = s_queue_head;
    LOG_QUEUE_READ_BARRIER();

    if (tail == head) {
      break;  /* Queue empty */
    }

    /* Calculate available data (contiguous from tail) */
    size_t available;
    if (head >= tail) {
      available = head - tail;
    } else {
      available = LOG_RAM_QUEUE_SIZE - tail;
    }

    if (available < LOG_RECORD_SIZE) {
      break;  /* Not enough for a full record */
    }

    /* Copy record to write chunk */
    size_t space_in_chunk = LOG_WRITE_CHUNK_SIZE - s_write_chunk_len;

    if (space_in_chunk < LOG_RECORD_SIZE) {
      /* Chunk full - flush it */
      if (log_flush_write_chunk()) {
        /* Update tail before returning */
        s_queue_tail = tail;
        return;
      }
      space_in_chunk = LOG_WRITE_CHUNK_SIZE;
    }

    /* Copy one record (data is valid - producer completed write before updating head) */
    memcpy(&s_write_chunk[s_write_chunk_len], &s_log_queue[tail],
           LOG_RECORD_SIZE);
    s_write_chunk_len += LOG_RECORD_SIZE;

    /* Advance local tail */
    tail = (tail + LOG_RECORD_SIZE) % LOG_RAM_QUEUE_SIZE;

    /* If chunk is full, flush it */
    if (s_write_chunk_len >= LOG_WRITE_CHUNK_SIZE) {
      if (log_flush_write_chunk()) {
        /* Update tail before returning */
        s_queue_tail = tail;
        return;
      }
      break;  /* One chunk per tick to avoid blocking too long */
    }
  }

  /* Publish new tail (atomic 32-bit write on ARM) */
  s_queue_tail = tail;

  /* Periodic meta update */
  uint32_t now = HAL_GetTick();
  if (now - s_last_meta_update_ms >= LOG_META_UPDATE_PERIOD_MS) {
    if (log_save_meta()) {
      s_last_meta_update_ms = now;
    }
  }
}

void log_erase_tick(void) {
  if (!s_initialized) {
    return;
  }

  if (log_is_dumping()) {
    return;
  }

  /* Check if flash is busy */
  if (qspi_w25q64_is_busy()) {
    return;
  }

  /* Calculate sectors ahead */
  uint32_t erase_addr = s_next_erase_addr;
  uint32_t write_addr = s_write_addr;

  /* Handle wrap-around */
  if (erase_addr < write_addr) {
    erase_addr += LOG_RING_SIZE;
  }

  uint32_t sectors_ahead = (erase_addr - write_addr) / W25Q64_SECTOR_SIZE;

  /* Erase if we're falling behind */
  if (sectors_ahead < LOG_PREERASE_AHEAD) {
    if (qspi_w25q64_erase_sector_async_start(s_next_erase_addr)) {
      /* Advance to next sector */
      s_next_erase_addr += W25Q64_SECTOR_SIZE;
      if (s_next_erase_addr >= LOG_RING_END) {
        s_next_erase_addr = LOG_RING_START;
      }
    }
  }
}

void log_get_stats(log_stats_t *out) {
  if (out == NULL) {
    return;
  }

  /* These are diagnostic counters - volatile reads are sufficient */
  out->dropped_records = s_dropped_records;
  out->qspi_write_errors = s_qspi_write_errors;
  out->total_records = s_total_records;
  out->wrap_count = s_wrap_count;

  /* Calculate erase lag */
  uint32_t erase_addr = s_next_erase_addr;
  uint32_t write_addr = s_write_addr;
  if (erase_addr < write_addr) {
    erase_addr += LOG_RING_SIZE;
  }
  uint32_t sectors_ahead = (erase_addr - write_addr) / W25Q64_SECTOR_SIZE;
  out->erase_lag_sectors =
      (sectors_ahead < LOG_PREERASE_AHEAD) ? (LOG_PREERASE_AHEAD - sectors_ahead) : 0;

  /* Estimate fill */
  if (s_total_records > 0) {
    /* Simplified estimate based on wrap count */
    uint32_t capacity_records = LOG_RING_SIZE / LOG_RECORD_SIZE;
    uint32_t rate_hz = (s_log_rate_hz > 0U) ? s_log_rate_hz : 1U;
    out->fill_seconds = capacity_records / rate_hz;
  } else {
    out->fill_seconds = 0;
  }
}

bool log_flush_pending(uint32_t timeout_ms) {
  if (!s_initialized) {
    return false;
  }

  uint32_t start = HAL_GetTick();
  while (1) {
    log_writer_tick();
    log_erase_tick();

    if (s_write_state == LOG_WRITE_IDLE && s_queue_head == s_queue_tail) {
      return true;
    }

    if (timeout_ms == 0U) {
      return false;
    }

    uint32_t now = HAL_GetTick();
    if ((now - start) >= timeout_ms) {
      return false;
    }
    HAL_Delay(1U);
  }
}

uint32_t log_get_write_addr(void) { return s_write_addr; }

uint32_t log_get_boot_write_addr(void) { return s_write_addr_at_boot; }

uint32_t log_get_seq(void) { return s_next_seq; }

uint32_t log_get_fields_mask(void) { return s_log_fields_mask; }

uint16_t log_get_rate_hz(void) { return s_log_rate_hz; }

/* Internal functions */

static bool log_flush_write_chunk(void) {
  if (s_write_chunk_len == 0) {
    return false;
  }

  if (s_write_state != LOG_WRITE_IDLE) {
    return false;
  }

  s_write_chunk_start_addr = s_write_addr;
  s_write_chunk_first_len = s_write_chunk_len;
  s_write_chunk_second_len = 0;

  if (s_write_chunk_start_addr + s_write_chunk_len > LOG_RING_END) {
    s_write_chunk_first_len = LOG_RING_END - s_write_chunk_start_addr;
    s_write_chunk_second_len = s_write_chunk_len - s_write_chunk_first_len;
  }

  if (!qspi_w25q64_write_async_start(s_write_chunk_start_addr, s_write_chunk,
                                    s_write_chunk_first_len)) {
    s_qspi_write_errors++;
    s_write_chunk_len = 0;
    s_write_chunk_first_len = 0;
    s_write_chunk_second_len = 0;
    s_write_state = LOG_WRITE_IDLE;
    return false;
  }

  s_write_state = LOG_WRITE_FIRST;
  return true;
}

static void log_advance_write_addr_after_chunk(void) {
  if (s_write_chunk_len == 0) {
    return;
  }

  if (s_write_chunk_start_addr + s_write_chunk_len > LOG_RING_END) {
    s_write_addr = LOG_RING_START + s_write_chunk_second_len;
    s_wrap_count++;
    return;
  }

  s_write_addr = s_write_chunk_start_addr + s_write_chunk_len;
  if (s_write_addr >= LOG_RING_END) {
    s_write_addr = LOG_RING_START;
    s_wrap_count++;
  }
}

static bool log_load_meta(void) {
  LogMeta meta0, meta1;
  bool valid0 = false, valid1 = false;

  /* Read slot 0 */
  if (qspi_w25q64_read(LOG_META_SLOT0, (uint8_t *)&meta0, sizeof(LogMeta))) {
    valid0 = log_validate_meta(&meta0);
  }

  /* Read slot 1 */
  if (qspi_w25q64_read(LOG_META_SLOT1, (uint8_t *)&meta1, sizeof(LogMeta))) {
    valid1 = log_validate_meta(&meta1);
  }

  /* Choose newest valid slot by sequence number */
  const LogMeta *chosen = NULL;

  if (valid0 && valid1) {
    /* Both valid - choose newer */
    chosen = (meta1.sequence > meta0.sequence) ? &meta1 : &meta0;
  } else if (valid0) {
    chosen = &meta0;
  } else if (valid1) {
    chosen = &meta1;
  }

  if (chosen == NULL) {
    return false;
  }

  /* Restore state from metadata */
  s_write_addr = chosen->write_addr;
  s_wrap_count = chosen->wrap_count;
  s_boot_count = chosen->boot_count;
  s_last_dump_id = chosen->last_dump_id;
  s_meta_sequence = chosen->sequence;

  return true;
}

static bool log_save_meta(void) {
  if (s_meta_save_state != LOG_META_SAVE_IDLE ||
      s_meta_op_inflight != LOG_META_OP_NONE) {
    return false;
  }

  LogMeta *meta = &s_meta_pending;
  memset(meta, 0, sizeof(*meta));

  /* Populate metadata */
  memcpy(meta->magic, LOG_META_MAGIC, 8);
  meta->version = LOG_META_VERSION;
  meta->record_size = LOG_RECORD_SIZE;
  meta->rate_hz = s_log_rate_hz;
  meta->log_fields_mask = s_log_fields_mask;
  meta->ring_start = LOG_RING_START;
  meta->ring_size = LOG_RING_SIZE;
  meta->write_addr = s_write_addr;
  meta->wrap_count = s_wrap_count;
  meta->boot_count = s_boot_count;
  meta->last_dump_id = s_last_dump_id;
  meta->sequence = s_meta_sequence++;  /* Post-increment: use current, then increment */

  /* Compute CRC */
  meta->meta_crc32 =
      robot_crc32((const uint8_t *)meta, sizeof(LogMeta) - sizeof(uint32_t));

  /* Write to alternating slot */
  s_meta_slot_addr = (meta->sequence % 2 == 0) ? LOG_META_SLOT0 : LOG_META_SLOT1;
  s_meta_save_state =
      ((meta->sequence % 2) == 0) ? LOG_META_SAVE_ERASE : LOG_META_SAVE_WRITE;

  return true;
}

static void log_meta_tick(qspi_w25q64_async_state_t async_state) {
  if (s_meta_save_state == LOG_META_SAVE_IDLE) {
    return;
  }

  if (s_meta_op_inflight != LOG_META_OP_NONE) {
    if (async_state == QSPI_W25Q64_ASYNC_DONE) {
      if (s_meta_op_inflight == LOG_META_OP_ERASE) {
        s_meta_op_inflight = LOG_META_OP_NONE;
        s_meta_save_state = LOG_META_SAVE_WRITE;
      } else {
        s_meta_op_inflight = LOG_META_OP_NONE;
        s_meta_save_state = LOG_META_SAVE_IDLE;
      }
    } else if (async_state == QSPI_W25Q64_ASYNC_ERROR) {
      s_meta_op_inflight = LOG_META_OP_NONE;
      s_meta_save_state = LOG_META_SAVE_IDLE;
    }
    return;
  }

  if (s_write_state != LOG_WRITE_IDLE) {
    return;
  }

  if (qspi_w25q64_is_busy()) {
    return;
  }

  if (s_meta_save_state == LOG_META_SAVE_ERASE) {
    if (qspi_w25q64_erase_sector_async_start(LOG_META_START)) {
      s_meta_op_inflight = LOG_META_OP_ERASE;
    }
  } else if (s_meta_save_state == LOG_META_SAVE_WRITE) {
    if (qspi_w25q64_write_async_start(s_meta_slot_addr,
                                     (const uint8_t *)&s_meta_pending,
                                     sizeof(s_meta_pending))) {
      s_meta_op_inflight = LOG_META_OP_WRITE;
    }
  }
}

static bool log_validate_meta(const LogMeta *meta) {
  if (meta == NULL) {
    return false;
  }

  /* Check magic */
  if (memcmp(meta->magic, LOG_META_MAGIC, 7) != 0) {
    return false;
  }

  /* Check version */
  if (meta->version != LOG_META_VERSION) {
    return false;
  }

  /* Check record and ring layout */
  if (meta->record_size != LOG_RECORD_SIZE) {
    return false;
  }
  if (meta->ring_start != LOG_RING_START || meta->ring_size != LOG_RING_SIZE) {
    return false;
  }

  /* Check CRC */
  uint32_t calc_crc =
      robot_crc32((const uint8_t *)meta, sizeof(LogMeta) - sizeof(uint32_t));
  if (calc_crc != meta->meta_crc32) {
    return false;
  }

  /* Check bounds */
  if (meta->write_addr < LOG_RING_START || meta->write_addr >= LOG_RING_END) {
    return false;
  }

  /* Ensure write_addr is aligned to the record size relative to ring start */
  if (((meta->write_addr - LOG_RING_START) % LOG_RECORD_SIZE) != 0U) {
    return false;
  }

  return true;
}

static bool log_validate_ring_tail(void) {
  if (!s_initialized && !qspi_w25q64_is_ready()) {
    return false;
  }

  uint32_t start = HAL_GetTick();
  while (qspi_w25q64_is_busy()) {
    if ((HAL_GetTick() - start) >= 50U) {
      APP_LOG_WARN("QSPI busy during ring validation");
      return true;
    }
    HAL_Delay(1U);
  }

  if (s_write_addr == LOG_RING_START) {
    return true;
  }

  uint32_t addr = s_write_addr - LOG_RECORD_SIZE;
  if (s_write_addr <= LOG_RING_START) {
    addr = LOG_RING_END - LOG_RECORD_SIZE;
  }

  LogRecord rec;
  if (!qspi_w25q64_read(addr, (uint8_t *)&rec, sizeof(rec))) {
    return false;
  }

  if (rec.magic != LOG_RECORD_MAGIC || rec.version != LOG_RECORD_VERSION) {
    return false;
  }

  return true;
}

static void log_format_meta(void) {
  /* Initialize state */
  s_write_addr = LOG_RING_START;
  s_wrap_count = 0;
  s_boot_count = 0;
  s_last_dump_id = 0;
  s_meta_sequence = 0;
  s_meta_save_state = LOG_META_SAVE_IDLE;
  s_meta_op_inflight = LOG_META_OP_NONE;
  s_next_erase_addr = LOG_RING_START;

  /* Save initial metadata */
  log_save_meta();
}

static bool log_erase_ahead_ready(void) {
  uint32_t erase_addr = s_next_erase_addr;
  uint32_t write_addr = s_write_addr;

  if (erase_addr < write_addr) {
    erase_addr += LOG_RING_SIZE;
  }
  uint32_t sectors_ahead = (erase_addr - write_addr) / W25Q64_SECTOR_SIZE;
  return sectors_ahead >= LOG_PREERASE_AHEAD;
}

static uint16_t log_rate_from_params(const robot_params_t *params) {
  float rate_hz = CONTROL_DEFAULT_HZ;
  if (params != NULL && params->control_rate_hz > 1e-3f) {
    rate_hz = params->control_rate_hz;
  }
  if (rate_hz < 1.0f) {
    rate_hz = 1.0f;
  }
  if (rate_hz > (float)UINT16_MAX) {
    rate_hz = (float)UINT16_MAX;
  }
  return (uint16_t)(rate_hz + 0.5f);
}
