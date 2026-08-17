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

void qspi_fake_reset(void) {
  memset(s_flash, 0xFF, sizeof(s_flash));
  s_initialized = false;
  s_read_failure = false;
  s_async_state = QSPI_W25Q64_ASYNC_IDLE;
  qspi_fake_reset_counters();
}

void qspi_fake_reset_counters(void) {
  s_read_calls = 0U;
  s_write_calls = 0U;
  s_erase_calls = 0U;
}

void qspi_fake_set_read_failure(int enabled) {
  s_read_failure = enabled != 0;
}

uint32_t qspi_fake_read_calls(void) { return s_read_calls; }
uint32_t qspi_fake_write_calls(void) { return s_write_calls; }
uint32_t qspi_fake_erase_calls(void) { return s_erase_calls; }
const uint8_t *qspi_fake_data(void) { return s_flash; }

void qspi_w25q64_init(void) { s_initialized = true; }
bool qspi_w25q64_is_ready(void) { return s_initialized; }

bool qspi_w25q64_read(uint32_t addr, uint8_t *buf, size_t len) {
  s_read_calls++;
  if (s_read_failure || buf == NULL || addr > W25Q64_FLASH_SIZE ||
      len > W25Q64_FLASH_SIZE - addr) {
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
  memcpy(s_flash + addr, buf, len);
  s_write_calls++;
  s_async_state = QSPI_W25Q64_ASYNC_DONE;
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
  memset(s_flash + addr, 0xFF, W25Q64_SECTOR_SIZE);
  s_erase_calls++;
  s_async_state = QSPI_W25Q64_ASYNC_DONE;
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
