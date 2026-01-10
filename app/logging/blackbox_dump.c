#include "blackbox_dump.h"

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
} s_dump_ctx;

/* Internal functions */
static void dump_reset_context(void);
static bool dump_calculate_window(uint32_t seconds);
static bool dump_generate_filename(void);
static void dump_advance_state(dump_state_t next_state);

bool log_dump_last_seconds(uint32_t seconds) {
  /* Check if already dumping */
  if (s_dump_ctx.state != DUMP_IDLE) {
    return false;
  }

  /* Check if SD is mounted */
  if (retSD != 0) {
    return false;
  }

  /* Calculate dump window */
  if (!dump_calculate_window(seconds)) {
    return false;
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
        dump_reset_context();
        break;
      }
      dump_advance_state(DUMP_WRITING_META);
      break;

    case DUMP_WRITING_META:
      /* Write metadata snapshot to file */
      {
        LogMeta meta;
        memset(&meta, 0, sizeof(meta));

        /* Populate meta (snapshot of current state) */
        memcpy(meta.magic, LOG_META_MAGIC, 8);
        meta.version = LOG_META_VERSION;
        meta.record_size = LOG_RECORD_SIZE;
        meta.rate_hz = 400;
        meta.log_fields_mask = 0;  /* TODO: Get from blackbox */
        meta.ring_start = LOG_RING_START;
        meta.ring_size = LOG_RING_SIZE;
        meta.write_addr = log_get_write_addr();
        meta.wrap_count = 0;  /* Not used in dump */
        meta.boot_count = 0;  /* Not used in dump */
        meta.last_dump_id = s_dump_ctx.last_dump_id;
        meta.sequence = 0;  /* Not used in dump */
        meta.meta_crc32 =
            robot_crc32((const uint8_t *)&meta, sizeof(LogMeta) - sizeof(uint32_t));

        res = f_write(&s_dump_ctx.file, &meta, sizeof(LogMeta), &bytes_written);
        if (res != FR_OK || bytes_written != sizeof(LogMeta)) {
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
        size_t chunk_size = DUMP_CHUNK_SIZE;
        if (chunk_size > (s_dump_ctx.current_addr - s_dump_ctx.start_addr)) {
          chunk_size = s_dump_ctx.current_addr - s_dump_ctx.start_addr;
        }

        res = f_write(&s_dump_ctx.file, s_dump_buffer, chunk_size, &bytes_written);
        if (res != FR_OK || bytes_written != chunk_size) {
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
        LogDumpTrailer trailer;
        trailer.start_seq = s_dump_ctx.start_seq;
        trailer.end_seq = s_dump_ctx.end_seq;
        trailer.record_count = s_dump_ctx.record_count;
        trailer.dump_crc32 = 0;  /* CRC not computed (would need incremental API) */

        res = f_write(&s_dump_ctx.file, &trailer, sizeof(LogDumpTrailer),
                      &bytes_written);
        if (res != FR_OK || bytes_written != sizeof(LogDumpTrailer)) {
          f_close(&s_dump_ctx.file);
          dump_reset_context();
          break;
        }

        s_dump_ctx.bytes_written += bytes_written;

        /* Sync and close */
        f_sync(&s_dump_ctx.file);
        f_close(&s_dump_ctx.file);

        dump_advance_state(DUMP_DONE);
      }
      break;

    case DUMP_DONE:
      /* Dump complete - reset to idle after one tick */
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
  memset(&s_dump_ctx, 0, sizeof(s_dump_ctx));
  s_dump_ctx.state = DUMP_IDLE;
}

static bool dump_calculate_window(uint32_t seconds) {
  /* Get current write position */
  uint32_t write_addr = log_get_write_addr();
  uint32_t current_seq = log_get_seq();

  /* Calculate bytes for time window */
  uint32_t records = seconds * 400U;  /* 400 Hz log rate */
  uint32_t bytes = records * LOG_RECORD_SIZE;

  /* Clamp to ring size */
  if (bytes > LOG_RING_SIZE) {
    bytes = LOG_RING_SIZE;
    records = bytes / LOG_RECORD_SIZE;
  }

  /* Calculate start address (going backwards from write position) */
  uint32_t start_addr;
  if (write_addr >= LOG_RING_START + bytes) {
    start_addr = write_addr - bytes;
  } else {
    /* Wrap around */
    uint32_t deficit = bytes - (write_addr - LOG_RING_START);
    start_addr = LOG_RING_END - deficit;
  }

  /* Store dump window */
  s_dump_ctx.start_addr = start_addr;
  s_dump_ctx.end_addr = write_addr;
  s_dump_ctx.current_addr = start_addr;
  s_dump_ctx.bytes_remaining = bytes;
  s_dump_ctx.record_count = records;
  s_dump_ctx.start_seq = (current_seq >= records) ? (current_seq - records) : 0;
  s_dump_ctx.end_seq = current_seq;

  return true;
}

static bool dump_generate_filename(void) {
  /* Increment dump ID */
  s_dump_ctx.last_dump_id++;

  /* Generate filename: DUMP_0001.BIN */
  int ret = snprintf(s_dump_ctx.filename, sizeof(s_dump_ctx.filename),
                     "DUMP_%04lu.BIN", (unsigned long)s_dump_ctx.last_dump_id);

  if (ret < 0 || ret >= (int)sizeof(s_dump_ctx.filename)) {
    return false;
  }

  return true;
}

static void dump_advance_state(dump_state_t next_state) {
  s_dump_ctx.state = next_state;
}
