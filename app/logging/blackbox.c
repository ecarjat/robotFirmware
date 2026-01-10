#include "blackbox.h"

#include "crc32.h"
#include "main.h"
#include "qspi_w25q64.h"
#include <string.h>

/* RAM queue for log records (in .dma_buffer section) */
static uint8_t s_log_queue[LOG_RAM_QUEUE_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));
static volatile uint32_t s_queue_head = 0;
static volatile uint32_t s_queue_tail = 0;
static volatile uint32_t s_dropped_records = 0;

/* Write state */
static uint32_t s_write_addr = LOG_RING_START;
static uint32_t s_wrap_count = 0;
static uint32_t s_next_seq = 0;
static uint32_t s_total_records = 0;

/* Erase state */
static uint32_t s_next_erase_addr = LOG_RING_START;

/* Meta update state */
static uint32_t s_last_meta_update_ms = 0;
static uint32_t s_boot_count = 0;
static uint32_t s_last_dump_id = 0;
static uint32_t s_meta_sequence = 0;

/* Statistics */
static uint32_t s_qspi_write_errors = 0;

/* Active log fields mask */
static uint32_t s_log_fields_mask = LOG_FIELDS_MASK_DEFAULT;

/* Write chunk buffer (in .dma_buffer section) */
static uint8_t s_write_chunk[LOG_WRITE_CHUNK_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));
static size_t s_write_chunk_len = 0;

/* Initialization flag */
static bool s_initialized = false;

/* Internal functions */
static bool log_load_meta(void);
static bool log_save_meta(void);
static bool log_validate_meta(const LogMeta *meta);
static void log_format_meta(void);
static void log_flush_write_chunk(void);

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

  /* Try to load existing metadata */
  if (!log_load_meta()) {
    /* No valid meta found - format flash */
    log_format_meta();
  }

  /* Increment boot count and save */
  s_boot_count++;
  log_save_meta();

  s_initialized = true;
}

void log_push_record(const LogRecord *rec) {
  if (!s_initialized || rec == NULL) {
    return;
  }

  /* Calculate space needed */
  const size_t record_size = LOG_RECORD_SIZE;
  uint32_t next_head = (s_queue_head + record_size) % LOG_RAM_QUEUE_SIZE;

  /* Check if queue is full */
  if (next_head == s_queue_tail) {
    s_dropped_records++;
    return;
  }

  /* Copy record into queue */
  uint32_t head = s_queue_head;
  if (head + record_size <= LOG_RAM_QUEUE_SIZE) {
    /* Contiguous copy */
    memcpy(&s_log_queue[head], rec, record_size);
  } else {
    /* Wrap-around copy */
    size_t first_part = LOG_RAM_QUEUE_SIZE - head;
    memcpy(&s_log_queue[head], rec, first_part);
    memcpy(&s_log_queue[0], (uint8_t *)rec + first_part,
           record_size - first_part);
  }

  /* Update head pointer */
  __disable_irq();
  s_queue_head = next_head;
  __enable_irq();
}

void log_writer_tick(void) {
  if (!s_initialized) {
    return;
  }

  /* Check if flash is busy */
  if (qspi_w25q64_is_busy()) {
    return;
  }

  /* Process queued records */
  while (s_queue_tail != s_queue_head) {
    /* Calculate available data */
    uint32_t tail = s_queue_tail;
    uint32_t head = s_queue_head;
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
      log_flush_write_chunk();
      space_in_chunk = LOG_WRITE_CHUNK_SIZE;
    }

    /* Copy one record */
    memcpy(&s_write_chunk[s_write_chunk_len], &s_log_queue[tail],
           LOG_RECORD_SIZE);
    s_write_chunk_len += LOG_RECORD_SIZE;
    s_total_records++;

    /* Update tail */
    __disable_irq();
    s_queue_tail = (tail + LOG_RECORD_SIZE) % LOG_RAM_QUEUE_SIZE;
    __enable_irq();

    /* If chunk is full, flush it */
    if (s_write_chunk_len >= LOG_WRITE_CHUNK_SIZE) {
      log_flush_write_chunk();
      break;  /* One chunk per tick to avoid blocking too long */
    }
  }

  /* Periodic meta update */
  uint32_t now = HAL_GetTick();
  if (now - s_last_meta_update_ms >= LOG_META_UPDATE_PERIOD_MS) {
    log_save_meta();
    s_last_meta_update_ms = now;
  }
}

void log_erase_tick(void) {
  if (!s_initialized) {
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
    if (qspi_w25q64_erase_sector_4k(s_next_erase_addr)) {
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

  __disable_irq();
  out->dropped_records = s_dropped_records;
  out->qspi_write_errors = s_qspi_write_errors;
  out->total_records = s_total_records;
  out->wrap_count = s_wrap_count;
  __enable_irq();

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
    out->fill_seconds = capacity_records / 400;  /* Assume 400 Hz */
  } else {
    out->fill_seconds = 0;
  }
}

uint32_t log_get_write_addr(void) { return s_write_addr; }

uint32_t log_get_seq(void) { return s_next_seq; }

/* Internal functions */

static void log_flush_write_chunk(void) {
  if (s_write_chunk_len == 0) {
    return;
  }

  /* Write chunk to flash */
  uint32_t addr = s_write_addr;
  size_t len = s_write_chunk_len;

  /* Handle ring wrap */
  if (addr + len > LOG_RING_END) {
    /* Split write across wrap boundary */
    size_t first_part = LOG_RING_END - addr;
    if (!qspi_w25q64_write_page(addr, s_write_chunk, first_part)) {
      s_qspi_write_errors++;
    }
    if (!qspi_w25q64_write_page(LOG_RING_START, s_write_chunk + first_part,
                                  len - first_part)) {
      s_qspi_write_errors++;
    }
    s_write_addr = LOG_RING_START + (len - first_part);
    s_wrap_count++;
  } else {
    /* Single contiguous write */
    if (!qspi_w25q64_write_page(addr, s_write_chunk, len)) {
      s_qspi_write_errors++;
    }
    s_write_addr = addr + len;
    if (s_write_addr >= LOG_RING_END) {
      s_write_addr = LOG_RING_START;
      s_wrap_count++;
    }
  }

  /* Reset chunk */
  s_write_chunk_len = 0;
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
  LogMeta meta;
  memset(&meta, 0, sizeof(meta));

  /* Populate metadata */
  memcpy(meta.magic, LOG_META_MAGIC, 8);
  meta.version = LOG_META_VERSION;
  meta.record_size = LOG_RECORD_SIZE;
  meta.rate_hz = 400;  /* TODO: Get from params */
  meta.log_fields_mask = s_log_fields_mask;
  meta.ring_start = LOG_RING_START;
  meta.ring_size = LOG_RING_SIZE;
  meta.write_addr = s_write_addr;
  meta.wrap_count = s_wrap_count;
  meta.boot_count = s_boot_count;
  meta.last_dump_id = s_last_dump_id;
  meta.sequence = ++s_meta_sequence;

  /* Compute CRC */
  meta.meta_crc32 =
      robot_crc32((const uint8_t *)&meta, sizeof(LogMeta) - sizeof(uint32_t));

  /* Write to alternating slot */
  uint32_t slot_addr = (meta.sequence % 2 == 0) ? LOG_META_SLOT0 : LOG_META_SLOT1;

  /* Erase slot first if needed */
  if ((meta.sequence % 2) == 0) {
    /* Erase the entire meta sector before writing slot 0 again */
    qspi_w25q64_erase_sector_4k(LOG_META_START);
  }

  return qspi_w25q64_write_page(slot_addr, (const uint8_t *)&meta,
                                  sizeof(LogMeta));
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

  return true;
}

static void log_format_meta(void) {
  /* Erase metadata sector */
  qspi_w25q64_erase_sector_4k(LOG_META_START);

  /* Initialize state */
  s_write_addr = LOG_RING_START;
  s_wrap_count = 0;
  s_boot_count = 0;
  s_last_dump_id = 0;
  s_meta_sequence = 0;

  /* Save initial metadata */
  log_save_meta();
}
