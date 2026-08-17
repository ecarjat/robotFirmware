#include "qspi_w25q64.h"
#include "qspi_fake.h"

#include <stdbool.h>
#include <string.h>

static uint8_t s_flash[W25Q64_FLASH_SIZE];
static bool s_initialized;
static bool s_read_failure;
static qspi_w25q64_async_state_t s_async_state;
static uint32_t s_read_calls;
static uint32_t s_write_calls;
static uint32_t s_erase_calls;
static uint32_t s_last_write_addr;
static uint32_t s_last_erase_addr;
static uint32_t s_fail_next_reads;
static bool s_power_loss_during_erase;
static size_t s_power_loss_write_bytes;

void qspi_fake_reset(void) {
  memset(s_flash, 0xFF, sizeof(s_flash));
  s_initialized = false;
  s_read_failure = false;
  s_async_state = QSPI_W25Q64_ASYNC_IDLE;
  s_fail_next_reads = 0U;
  s_power_loss_during_erase = false;
  s_power_loss_write_bytes = 0U;
  qspi_fake_reset_counters();
}

void qspi_fake_power_cycle(void) {
  /* Preserve flash contents while dropping the volatile driver state. */
  s_initialized = false;
  s_read_failure = false;
  s_async_state = QSPI_W25Q64_ASYNC_IDLE;
  s_fail_next_reads = 0U;
  s_power_loss_during_erase = false;
  s_power_loss_write_bytes = 0U;
}

void qspi_fake_reset_counters(void) {
  s_read_calls = 0U;
  s_write_calls = 0U;
  s_erase_calls = 0U;
  s_last_write_addr = UINT32_MAX;
  s_last_erase_addr = UINT32_MAX;
}

void qspi_fake_set_read_failure(int enabled) {
  s_read_failure = enabled != 0;
}

void qspi_fake_fail_next_read(void) { s_fail_next_reads++; }

void qspi_fake_power_loss_during_next_erase(void) {
  s_power_loss_during_erase = true;
}

void qspi_fake_power_loss_during_next_write(size_t programmed_bytes) {
  s_power_loss_write_bytes = programmed_bytes;
}

uint32_t qspi_fake_read_calls(void) { return s_read_calls; }
uint32_t qspi_fake_write_calls(void) { return s_write_calls; }
uint32_t qspi_fake_erase_calls(void) { return s_erase_calls; }
uint32_t qspi_fake_last_write_addr(void) { return s_last_write_addr; }
uint32_t qspi_fake_last_erase_addr(void) { return s_last_erase_addr; }
const uint8_t *qspi_fake_data(void) { return s_flash; }

void qspi_w25q64_init(void) { s_initialized = true; }
bool qspi_w25q64_is_ready(void) { return s_initialized; }

bool qspi_w25q64_read(uint32_t addr, uint8_t *buf, size_t len) {
  s_read_calls++;
  if (s_read_failure || s_fail_next_reads > 0U || buf == NULL ||
      addr > W25Q64_FLASH_SIZE ||
      len > W25Q64_FLASH_SIZE - addr) {
    if (s_fail_next_reads > 0U) {
      s_fail_next_reads--;
    }
    return false;
  }
  memcpy(buf, s_flash + addr, len);
  return true;
}

bool qspi_w25q64_write_async_start(uint32_t addr, const uint8_t *buf,
                                    size_t len) {
  if (buf == NULL || addr > W25Q64_FLASH_SIZE ||
      len > W25Q64_FLASH_SIZE - addr ||
      s_async_state != QSPI_W25Q64_ASYNC_IDLE) {
    return false;
  }
  size_t programmed_len = len;
  bool power_loss = s_power_loss_write_bytes > 0U;
  if (power_loss && programmed_len > s_power_loss_write_bytes) {
    programmed_len = s_power_loss_write_bytes;
  }
  for (size_t i = 0; i < programmed_len; ++i) {
    /* NOR page programming can only change set bits to cleared bits. */
    s_flash[addr + i] &= buf[i];
  }
  s_write_calls++;
  s_last_write_addr = addr;
  if (power_loss) {
    s_power_loss_write_bytes = 0U;
    s_async_state = QSPI_W25Q64_ASYNC_BUSY;
  } else {
    s_async_state = QSPI_W25Q64_ASYNC_DONE;
  }
  return true;
}

qspi_w25q64_async_state_t qspi_w25q64_write_async_tick(void) {
  qspi_w25q64_async_state_t result = s_async_state;
  if (s_async_state != QSPI_W25Q64_ASYNC_IDLE) {
    s_async_state = QSPI_W25Q64_ASYNC_IDLE;
  }
  return result;
}

bool qspi_w25q64_erase_sector_async_start(uint32_t addr) {
  if (addr > W25Q64_FLASH_SIZE || W25Q64_SECTOR_SIZE > W25Q64_FLASH_SIZE - addr ||
      s_async_state != QSPI_W25Q64_ASYNC_IDLE) {
    return false;
  }
  addr &= ~(W25Q64_SECTOR_SIZE - 1U);
  memset(s_flash + addr, 0xFF, W25Q64_SECTOR_SIZE);
  s_erase_calls++;
  s_last_erase_addr = addr;
  if (s_power_loss_during_erase) {
    s_power_loss_during_erase = false;
    s_async_state = QSPI_W25Q64_ASYNC_BUSY;
  } else {
    s_async_state = QSPI_W25Q64_ASYNC_DONE;
  }
  return true;
}

bool qspi_w25q64_is_busy(void) {
  return s_async_state != QSPI_W25Q64_ASYNC_IDLE;
}

bool qspi_w25q64_write_page(uint32_t addr, const uint8_t *buf, size_t len) {
  return qspi_w25q64_write_async_start(addr, buf, len);
}

bool qspi_w25q64_erase_sector_4k(uint32_t addr) {
  return qspi_w25q64_erase_sector_async_start(addr);
}

bool qspi_w25q64_erase_block_64k(uint32_t addr) {
  if (addr > W25Q64_FLASH_SIZE || W25Q64_BLOCK_64K_SIZE > W25Q64_FLASH_SIZE - addr) {
    return false;
  }
  memset(s_flash + addr, 0xFF, W25Q64_BLOCK_64K_SIZE);
  return true;
}

uint8_t qspi_w25q64_read_status(void) { return 0U; }
bool qspi_w25q64_read_id(uint8_t *manufacturer, uint16_t *device) {
  if (manufacturer != NULL) {
    *manufacturer = 0xEFU;
  }
  if (device != NULL) {
    *device = 0x4017U;
  }
  return true;
}
