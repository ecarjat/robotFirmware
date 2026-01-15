#include "blackbox_dump.h"

#include "app_config.h"
#include "blackbox.h"
#include "blackbox_format.h"
#include "crc32.h"
#include "fatfs.h"
#include "qspi_w25q64.h"
#include <stdio.h>
#include <string.h>

/* Dump chunk size (8 KB - balance between RAM usage and SD efficiency) */
#define DUMP_CHUNK_SIZE (8192U)

/* Dump buffer (in .dma_buffer section for SD DMA) */
static uint8_t s_dump_buffer[DUMP_CHUNK_SIZE]
    __attribute__((section(".dma_buffer"), aligned(32)));

/* Dump state machine */
typedef enum {
  DUMP_IDLE,
  DUMP_OPENING_FILE,
  DUMP_WRITING_META,
  DUMP_READING_QSPI,
  DUMP_WRITING_SD,
  DUMP_FINALIZING,
  DUMP_DONE
} dump_state_t;

/* Dump context */
static struct {
  dump_state_t state;
  FIL file;
  uint32_t start_addr;       /* Ring start address for this dump */
  uint32_t end_addr;         /* Ring end address for this dump */
  uint32_t current_addr;     /* Current read position */
  uint32_t bytes_remaining;  /* Bytes left to dump */
  uint32_t start_seq;        /* First record sequence number */
  uint32_t end_seq;          /* Last record sequence number */
  uint32_t record_count;     /* Number of records in dump */
  uint32_t bytes_written;    /* Total bytes written to SD */
  char filename[16];         /* DUMP_nnnn.BIN */
  uint32_t last_dump_id;     /* Dump file counter */
  size_t current_chunk_size; /* Size of chunk being transferred */
} s_dump_ctx __attribute__((section(".dma_buffer"), aligned(32)));

static LogMeta s_dump_meta __attribute__((section(".dma_buffer"), aligned(32)));
static LogDumpTrailer s_dump_trailer __attribute__((section(".dma_buffer"), aligned(32)));
static bool s_dump_ctx_initialized = false;

/* Internal functions */
static const char *dump_state_name(dump_state_t state);
static void dump_reset_context(void);
static bool dump_calculate_window(uint32_t seconds);
static bool dump_generate_filename(void);
static void dump_advance_state(dump_state_t next_state);
static void dump_clean_cache(const void *addr, size_t len);

void log_dump_init(void) {
  dump_reset_context();
  s_dump_ctx_initialized = true;
}

bool log_dump_last_seconds(uint32_t seconds) {
  /* Check if already dumping */
  if (s_dump_ctx.state != DUMP_IDLE) {
    return false;
  }

  /* Check if SD is mounted */
  if (retSD != 0) {
    return false;
  }

  /* Log current blackbox stats for debugging */
  log_stats_t stats;
  log_get_stats(&stats);
  APP_LOG_INFO("Dump requested: %lu sec, blackbox stats: total=%lu dropped=%lu qspi_err=%lu",
               (unsigned long)seconds, (unsigned long)stats.total_records,
               (unsigned long)stats.dropped_records, (unsigned long)stats.qspi_write_errors);

  /* Debug: log writer tick blocking reasons (defined in blackbox.c) */
  extern uint32_t s_writer_debug_counter;
  extern uint32_t s_writer_not_init;
  extern uint32_t s_writer_busy;
  extern uint32_t s_writer_dumping;
  extern uint32_t s_writer_not_idle;
  extern uint32_t s_writer_queue_empty;
  extern uint32_t s_writer_flush_ok;
  extern uint32_t s_writer_flush_fail;
  APP_LOG_INFO("Writer debug: ticks=%lu not_init=%lu busy=%lu dumping=%lu not_idle=%lu",
               (unsigned long)s_writer_debug_counter, (unsigned long)s_writer_not_init,
               (unsigned long)s_writer_busy, (unsigned long)s_writer_dumping,
               (unsigned long)s_writer_not_idle);
  APP_LOG_INFO("Writer queue: empty=%lu flush_ok=%lu flush_fail=%lu",
               (unsigned long)s_writer_queue_empty, (unsigned long)s_writer_flush_ok,
               (unsigned long)s_writer_flush_fail);

  /* Calculate dump window */
  if (!dump_calculate_window(seconds)) {
    return false;
  }

  /* Debug: Read first 16 bytes from QSPI at start address to verify content */
  {
    uint8_t qspi_sample[16];
    if (qspi_w25q64_is_busy()) {
      APP_LOG_WARN("QSPI busy, sample skipped");
    } else if (qspi_w25q64_read(s_dump_ctx.start_addr, qspi_sample, sizeof(qspi_sample))) {
      APP_LOG_INFO("QSPI sample at 0x%06lX: %02X %02X %02X %02X %02X %02X %02X %02X",
                   (unsigned long)s_dump_ctx.start_addr,
                   qspi_sample[0], qspi_sample[1], qspi_sample[2], qspi_sample[3],
                   qspi_sample[4], qspi_sample[5], qspi_sample[6], qspi_sample[7]);
      /* Check for expected magic (0xA55A = 5A A5 in little-endian bytes) */
      uint16_t magic = qspi_sample[0] | (qspi_sample[1] << 8);
      APP_LOG_INFO("First record magic: 0x%04X (expected 0xA55A)", magic);
    } else {
      APP_LOG_ERROR("QSPI read failed at 0x%06lX", (unsigned long)s_dump_ctx.start_addr);
    }
  }

  /* Generate filename */
  if (!dump_generate_filename()) {
    return false;
  }

  /* Initiate dump */
  dump_advance_state(DUMP_OPENING_FILE);
  return true;
}

bool log_is_dumping(void) {
  return (s_dump_ctx.state != DUMP_IDLE && s_dump_ctx.state != DUMP_DONE);
}

void log_dump_tick(void) {
  FRESULT res;
  UINT bytes_written;

  switch (s_dump_ctx.state) {
    case DUMP_IDLE:
      /* Nothing to do */
      break;

    case DUMP_OPENING_FILE:
      /* Open file for writing */
      res = f_open(&s_dump_ctx.file, s_dump_ctx.filename,
                   FA_CREATE_ALWAYS | FA_WRITE);
      if (res != FR_OK) {
        APP_LOG_ERROR("Dump open failed for %s (err=%d)",
                      s_dump_ctx.filename, res);
        dump_reset_context();
        break;
      }
      dump_advance_state(DUMP_WRITING_META);
      break;

    case DUMP_WRITING_META:
      /* Write metadata snapshot to file */
      {
        memset(&s_dump_meta, 0, sizeof(s_dump_meta));

        /* Populate meta (snapshot of current state) */
        memcpy(s_dump_meta.magic, LOG_META_MAGIC, 8);
        s_dump_meta.version = LOG_META_VERSION;
        s_dump_meta.record_size = LOG_RECORD_SIZE;
        s_dump_meta.rate_hz = log_get_rate_hz();
        s_dump_meta.log_fields_mask = log_get_fields_mask();
        s_dump_meta.ring_start = LOG_RING_START;
        s_dump_meta.ring_size = LOG_RING_SIZE;
        s_dump_meta.write_addr = log_get_write_addr();
        s_dump_meta.wrap_count = 0;  /* Not used in dump */
        s_dump_meta.boot_count = 0;  /* Not used in dump */
        s_dump_meta.last_dump_id = s_dump_ctx.last_dump_id;
        s_dump_meta.sequence = 0;  /* Not used in dump */
        s_dump_meta.meta_crc32 =
            robot_crc32((const uint8_t *)&s_dump_meta,
                        sizeof(LogMeta) - sizeof(uint32_t));

        dump_clean_cache(&s_dump_meta, sizeof(s_dump_meta));
        res = f_write(&s_dump_ctx.file, &s_dump_meta, sizeof(LogMeta),
                      &bytes_written);
        if (res != FR_OK || bytes_written != sizeof(LogMeta)) {
          APP_LOG_ERROR("Dump meta write failed (err=%d, wrote=%u/%u)",
                        res, (unsigned int)bytes_written,
                        (unsigned int)sizeof(LogMeta));
          f_close(&s_dump_ctx.file);
          dump_reset_context();
          break;
        }

        s_dump_ctx.bytes_written += bytes_written;
        dump_advance_state(DUMP_READING_QSPI);
      }
      break;

    case DUMP_READING_QSPI:
      /* Read chunk from QSPI flash */
      if (qspi_w25q64_is_busy()) {
        break;  /* Wait for QSPI ready */
      }

      {
        size_t chunk_size = DUMP_CHUNK_SIZE;
        if (chunk_size > s_dump_ctx.bytes_remaining) {
          chunk_size = s_dump_ctx.bytes_remaining;
        }
        s_dump_ctx.current_chunk_size = chunk_size;

        uint32_t read_addr = s_dump_ctx.current_addr;

        /* Handle ring wrap */
        if (read_addr >= LOG_RING_END) {
          read_addr = LOG_RING_START + (read_addr - LOG_RING_END);
        }

        /* Handle split read across ring boundary */
        if (read_addr + chunk_size > LOG_RING_END) {
          size_t first_part = LOG_RING_END - read_addr;
          if (!qspi_w25q64_read(read_addr, s_dump_buffer, first_part)) {
            f_close(&s_dump_ctx.file);
            dump_reset_context();
            break;
          }
          if (!qspi_w25q64_read(LOG_RING_START, s_dump_buffer + first_part,
                                chunk_size - first_part)) {
            f_close(&s_dump_ctx.file);
            dump_reset_context();
            break;
          }
        } else {
          /* Single contiguous read */
          if (!qspi_w25q64_read(read_addr, s_dump_buffer, chunk_size)) {
            f_close(&s_dump_ctx.file);
            dump_reset_context();
            break;
          }
        }

        /* Advance to write state */
        s_dump_ctx.current_addr += chunk_size;
        s_dump_ctx.bytes_remaining -= chunk_size;
        dump_advance_state(DUMP_WRITING_SD);
      }
      break;

    case DUMP_WRITING_SD:
      /* Write chunk to SD card */
      {
        size_t chunk_size = s_dump_ctx.current_chunk_size;

        /* Flush cache so SD DMA reads the latest QSPI data. */
        dump_clean_cache(s_dump_buffer, chunk_size);

        res = f_write(&s_dump_ctx.file, s_dump_buffer, chunk_size, &bytes_written);
        if (res != FR_OK || bytes_written != chunk_size) {
          APP_LOG_ERROR("Dump data write failed (err=%d, wrote=%u/%u)",
                        res, (unsigned int)bytes_written,
                        (unsigned int)chunk_size);
          f_close(&s_dump_ctx.file);
          dump_reset_context();
          break;
        }

        s_dump_ctx.bytes_written += bytes_written;

        /* Check if more data to dump */
        if (s_dump_ctx.bytes_remaining > 0) {
          dump_advance_state(DUMP_READING_QSPI);
        } else {
          dump_advance_state(DUMP_FINALIZING);
        }
      }
      break;

    case DUMP_FINALIZING:
      /* Write dump trailer and close file */
      {
        s_dump_trailer.start_seq = s_dump_ctx.start_seq;
        s_dump_trailer.end_seq = s_dump_ctx.end_seq;
        s_dump_trailer.record_count = s_dump_ctx.record_count;
        s_dump_trailer.dump_crc32 = 0;  /* CRC not computed (would need incremental API) */

        dump_clean_cache(&s_dump_trailer, sizeof(s_dump_trailer));
        res = f_write(&s_dump_ctx.file, &s_dump_trailer,
                      sizeof(LogDumpTrailer), &bytes_written);
        if (res != FR_OK || bytes_written != sizeof(LogDumpTrailer)) {
          APP_LOG_ERROR("Dump trailer write failed (err=%d, wrote=%u/%u)",
                        res, (unsigned int)bytes_written,
                        (unsigned int)sizeof(LogDumpTrailer));
          f_close(&s_dump_ctx.file);
          dump_reset_context();
          break;
        }

        s_dump_ctx.bytes_written += bytes_written;

        /* Sync and close */
        res = f_sync(&s_dump_ctx.file);
        if (res != FR_OK) {
          APP_LOG_ERROR("Dump sync failed (err=%d)", res);
          f_close(&s_dump_ctx.file);
          dump_reset_context();
          break;
        }
        res = f_close(&s_dump_ctx.file);
        if (res != FR_OK) {
          APP_LOG_ERROR("Dump close failed (err=%d)", res);
          dump_reset_context();
          break;
        }

        dump_advance_state(DUMP_DONE);
      }
      break;

    case DUMP_DONE:
      /* Dump complete */
      APP_LOG_INFO("Dump complete: %s (%lu bytes)",
                   s_dump_ctx.filename,
                   (unsigned long)s_dump_ctx.bytes_written);
      dump_reset_context();
      break;
  }
}

bool log_get_dump_stats(uint32_t *records_written, uint32_t *bytes_written) {
  if (records_written == NULL || bytes_written == NULL) {
    return false;
  }

  if (s_dump_ctx.state == DUMP_IDLE && s_dump_ctx.record_count == 0) {
    return false;  /* No dump has been performed */
  }

  *records_written = s_dump_ctx.record_count;
  *bytes_written = s_dump_ctx.bytes_written;
  return true;
}

/* Internal functions */

static void dump_reset_context(void) {
  if (s_dump_ctx_initialized) {
    APP_LOG_WARN("Dump reset (was in state %s)", dump_state_name(s_dump_ctx.state));
  }
  memset(&s_dump_ctx, 0, sizeof(s_dump_ctx));
  s_dump_ctx.state = DUMP_IDLE;
}

static bool dump_calculate_window(uint32_t seconds) {
  /* Get current and boot-time logger state */
  uint32_t write_addr = log_get_write_addr();
  uint32_t boot_write_addr = log_get_boot_write_addr();
  uint32_t current_seq = log_get_seq();
  log_stats_t stats;
  log_get_stats(&stats);

  /* Compute how many bytes are actually committed to QSPI this session. */
  uint32_t bytes_written = 0;
  if (write_addr >= boot_write_addr) {
    bytes_written = write_addr - boot_write_addr;
  } else {
    bytes_written = (LOG_RING_END - boot_write_addr) +
                    (write_addr - LOG_RING_START);
  }
  uint32_t records_written = bytes_written / LOG_RECORD_SIZE;
  APP_LOG_INFO("Dump debug: committed bytes=%lu records=%lu (stats.total=%lu)",
               (unsigned long)bytes_written, (unsigned long)records_written,
               (unsigned long)stats.total_records);

  /* Determine how many records to dump */
  uint16_t rate_hz = log_get_rate_hz();
  if (rate_hz == 0) {
    rate_hz = 1;
  }
  uint32_t records = seconds * (uint32_t)rate_hz;

  /* Don't dump more records than exist in this boot session */
  if (records > records_written) {
    records = records_written;
  }

  if (records == 0) {
    APP_LOG_WARN("No records to dump in this session.");
    return false;
  }

  uint32_t bytes = records * LOG_RECORD_SIZE;

  /*
   * Calculate the start address for the dump. This is complex because
   * the valid data from this session might wrap around the ring buffer.
   *
   * We have two main scenarios:
   * 1. The data written this session is contiguous (no wrap).
   * 2. The data has wrapped around the end of the ring buffer.
   */
  uint32_t start_addr;
  if (write_addr >= boot_write_addr) {
    /* SCENARIO 1: No wrap during this session.
     * Data is in a single block: [boot_write_addr, write_addr)
     */
    start_addr = write_addr - bytes;
  } else {
    /* SCENARIO 2: A wrap has occurred during this session.
     * Data is in two blocks:
     * - [boot_write_addr, LOG_RING_END)
     * - [LOG_RING_START, write_addr)
     */
    uint32_t bytes_after_wrap = write_addr - LOG_RING_START;

    if (bytes <= bytes_after_wrap) {
      /* The entire dump fits in the block at the start of the ring */
      start_addr = write_addr - bytes;
    } else {
      /* The dump spans both blocks (pre and post-wrap) */
      uint32_t deficit = bytes - bytes_after_wrap;
      start_addr = LOG_RING_END - deficit;
    }
  }


  /* Store dump window */
  s_dump_ctx.start_addr = start_addr;
  s_dump_ctx.end_addr = write_addr;
  s_dump_ctx.current_addr = start_addr;
  s_dump_ctx.bytes_remaining = bytes;
  s_dump_ctx.record_count = records;
  s_dump_ctx.start_seq = (current_seq >= records) ? (current_seq - records) : 0;
  s_dump_ctx.end_seq = current_seq;

  APP_LOG_INFO("Dump window: write_addr=0x%06lX, start=0x%06lX, bytes=%lu, records=%lu, seq=%lu, rate=%u Hz",
               (unsigned long)write_addr, (unsigned long)start_addr,
               (unsigned long)bytes, (unsigned long)records,
               (unsigned long)current_seq, (unsigned)rate_hz);

  /* Warn if no data to dump */
  if (records_written == 0) {
    APP_LOG_WARN("Dump requested but no records written yet (write_addr=0x%06lX, seq=%lu)",
                 (unsigned long)write_addr, (unsigned long)current_seq);
  }

  return true;
}

static bool dump_generate_filename(void) {
  DIR dir;
  FILINFO fno;
  FRESULT res;
  uint32_t max_id = 0;

  /* Scan SD card root for existing DUMP_*.BIN files */
  res = f_opendir(&dir, "/");
  if (res == FR_OK) {
    for (;;) {
      res = f_readdir(&dir, &fno);
      if (res != FR_OK || fno.fname[0] == 0) {
        break;  /* End of directory or error */
      }

      /* Check if filename matches DUMP_NNNN.BIN pattern */
      if (strncmp(fno.fname, "DUMP_", 5) == 0 &&
          strlen(fno.fname) == 13 &&
          strcmp(fno.fname + 9, ".BIN") == 0) {
        /* Extract number from filename */
        uint32_t id = 0;
        for (int i = 5; i < 9; i++) {
          char c = fno.fname[i];
          if (c >= '0' && c <= '9') {
            id = id * 10 + (c - '0');
          } else {
            id = 0;  /* Invalid format, skip */
            break;
          }
        }
        if (id > max_id) {
          max_id = id;
        }
      }
    }
    f_closedir(&dir);
  }

  /* Use next sequential number */
  s_dump_ctx.last_dump_id = max_id + 1;

  /* Generate filename: DUMP_0001.BIN */
  int ret = snprintf(s_dump_ctx.filename, sizeof(s_dump_ctx.filename),
                     "DUMP_%04lu.BIN", (unsigned long)s_dump_ctx.last_dump_id);

  if (ret < 0 || ret >= (int)sizeof(s_dump_ctx.filename)) {
    return false;
  }

  APP_LOG_INFO("Next dump file: %s (found %lu existing)",
               s_dump_ctx.filename, (unsigned long)max_id);

  return true;
}

static const char *dump_state_name(dump_state_t state) {
  switch (state) {
    case DUMP_IDLE: return "IDLE";
    case DUMP_OPENING_FILE: return "OPENING_FILE";
    case DUMP_WRITING_META: return "WRITING_META";
    case DUMP_READING_QSPI: return "READING_QSPI";
    case DUMP_WRITING_SD: return "WRITING_SD";
    case DUMP_FINALIZING: return "FINALIZING";
    case DUMP_DONE: return "DONE";
    default: return "UNKNOWN";
  }
}

static void dump_advance_state(dump_state_t next_state) {
  APP_LOG_INFO("Dump: %s -> %s", dump_state_name(s_dump_ctx.state),
               dump_state_name(next_state));
  s_dump_ctx.state = next_state;
}

static void dump_clean_cache(const void *addr, size_t len) {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  if (addr == NULL || len == 0U) {
    return;
  }
  uintptr_t start = (uintptr_t)addr;
  uintptr_t end = start + len;
  uintptr_t aligned_start = start & ~(uintptr_t)(32U - 1U);
  uintptr_t aligned_end = (end + (32U - 1U)) & ~(uintptr_t)(32U - 1U);
  SCB_CleanDCache_by_Addr((uint32_t *)aligned_start,
                          (int32_t)(aligned_end - aligned_start));
#else
  (void)addr;
  (void)len;
#endif
}
