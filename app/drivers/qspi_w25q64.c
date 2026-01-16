#include "qspi_w25q64.h"

#include "app_config.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* External OSPI handle from main.c */
extern OSPI_HandleTypeDef hospi1;

/* Initialization flag */
static bool s_initialized = false;

typedef enum {
  QSPI_ASYNC_IDLE = 0,
  QSPI_ASYNC_START,
  QSPI_ASYNC_WAIT_TX,
  QSPI_ASYNC_WAIT_WIP,
  QSPI_ASYNC_DONE,
  QSPI_ASYNC_ERROR
} qspi_async_state_t;

typedef enum {
  QSPI_ASYNC_OP_NONE = 0,
  QSPI_ASYNC_OP_WRITE,
  QSPI_ASYNC_OP_ERASE_4K
} qspi_async_op_t;

static volatile qspi_async_state_t s_async_state = QSPI_ASYNC_IDLE;
static volatile qspi_async_op_t s_async_op = QSPI_ASYNC_OP_NONE;
static uint32_t s_async_addr = 0;
static const uint8_t *s_async_buf = NULL;
static size_t s_async_remaining = 0;
static size_t s_async_chunk = 0;
static volatile uint8_t s_async_tx_done = 0;
static volatile HAL_StatusTypeDef s_async_error_status = HAL_OK;
static volatile uint8_t s_async_error_reported = 0;

/* Internal helper functions */
static bool qspi_wait_ready(uint32_t timeout_ms);
static bool qspi_write_enable(void);
static void qspi_cache_clean(const void *addr, size_t len);
static void qspi_log_ospi_error(const char *op, HAL_StatusTypeDef status);
static bool qspi_async_start_page(void);
static bool qspi_read_status_reg(uint8_t cmd_byte, uint8_t *status);
static bool qspi_write_status(uint8_t sr1, uint8_t sr2);
static void qspi_clear_block_protection(void);

void qspi_w25q64_init(void) {
  /* OCTOSPI peripheral already initialized in main.c via MX_OCTOSPI1_Init() */
  /* Just verify we can communicate with the flash */

  uint8_t manufacturer;
  uint16_t device;

  if (qspi_w25q64_read_id(&manufacturer, &device)) {
    if (manufacturer == 0xEF && device == 0x4017) {
      s_initialized = true;
      qspi_clear_block_protection();
    }
  }
}

bool qspi_w25q64_is_ready(void) {
  return s_initialized && !qspi_w25q64_is_busy();
}

bool qspi_w25q64_read(uint32_t addr, uint8_t *buf, size_t len) {
  if (!s_initialized || buf == NULL || len == 0) {
    return false;
  }

  if (addr + len > W25Q64_FLASH_SIZE) {
    return false;
  }

  /* Configure command for read operation */
  OSPI_RegularCmdTypeDef cmd = {0};
  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
  cmd.FlashId = HAL_OSPI_FLASH_ID_1;
  cmd.Instruction = W25Q64_CMD_READ_DATA;
  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  cmd.Address = addr;
  cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
  cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
  cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  cmd.DataMode = HAL_OSPI_DATA_1_LINE;
  cmd.NbData = len;
  cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return false;
  }

  /* Receive data (polling mode - CPU writes directly to cache) */
  if (HAL_OSPI_Receive(&hospi1, buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return false;
  }

  /*
   * NOTE: Do NOT invalidate cache here. HAL_OSPI_Receive() uses polling mode
   * where the CPU reads from the peripheral and writes to the buffer. The data
   * is now in the CPU's cache. Invalidating would discard it!
   *
   * Cache invalidation is only needed after DMA writes to memory (so CPU can
   * see DMA's data). For polling, the CPU already has the data.
   *
   * Callers that pass this buffer to DMA (e.g., SD card write) must call
   * SCB_CleanDCache_by_Addr() to flush cache to RAM before DMA reads.
   */

  return true;
}

bool qspi_w25q64_write_page(uint32_t addr, const uint8_t *buf, size_t len) {
  if (!s_initialized || buf == NULL || len == 0) {
    return false;
  }

  if (addr + len > W25Q64_FLASH_SIZE) {
    return false;
  }

  size_t remaining = len;
  uint32_t cur_addr = addr;
  const uint8_t *cur_buf = buf;

  while (remaining > 0) {
    size_t page_offset = cur_addr % W25Q64_PAGE_SIZE;
    size_t chunk = W25Q64_PAGE_SIZE - page_offset;
    if (chunk > remaining) {
      chunk = remaining;
    }

    /* Wait for any previous operation to complete */
    if (!qspi_wait_ready(W25Q64_TIMEOUT_PAGE_PROGRAM)) {
      return false;
    }

    /* Enable write */
    if (!qspi_write_enable()) {
      return false;
    }

    /* Clean D-Cache before DMA write */
    qspi_cache_clean(cur_buf, chunk);

    /* Configure command for page program */
    OSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId = HAL_OSPI_FLASH_ID_1;
    cmd.Instruction = W25Q64_CMD_PAGE_PROGRAM;
    cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.Address = cur_addr;
    cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
    cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
    cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    cmd.NbData = chunk;
    cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
    cmd.DummyCycles = 0;
    cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
    cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

    if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
        HAL_OK) {
      return false;
    }

    /* Transmit data */
    if (HAL_OSPI_Transmit(&hospi1, (uint8_t *)cur_buf,
                          HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
      return false;
    }

    /* Wait for write to complete */
    if (!qspi_wait_ready(W25Q64_TIMEOUT_PAGE_PROGRAM)) {
      return false;
    }

    cur_addr += chunk;
    cur_buf += chunk;
    remaining -= chunk;
  }

  return true;
}

bool qspi_w25q64_write_async_start(uint32_t addr, const uint8_t *buf, size_t len) {
  if (!s_initialized || buf == NULL || len == 0) {
    return false;
  }

  if (addr + len > W25Q64_FLASH_SIZE) {
    return false;
  }

  if (s_async_state == QSPI_ASYNC_START ||
      s_async_state == QSPI_ASYNC_WAIT_TX ||
      s_async_state == QSPI_ASYNC_WAIT_WIP) {
    return false;
  }

  s_async_op = QSPI_ASYNC_OP_WRITE;
  s_async_addr = addr;
  s_async_buf = buf;
  s_async_remaining = len;
  s_async_chunk = 0;
  s_async_tx_done = 0;
  s_async_error_status = HAL_OK;
  s_async_error_reported = 0;
  s_async_state = QSPI_ASYNC_START;

  (void)qspi_w25q64_write_async_tick();
  return s_async_state != QSPI_ASYNC_ERROR;
}

qspi_w25q64_async_state_t qspi_w25q64_write_async_tick(void) {
  if (!s_initialized) {
    return QSPI_W25Q64_ASYNC_ERROR;
  }

  switch (s_async_state) {
    case QSPI_ASYNC_START:
      if (s_async_op != QSPI_ASYNC_OP_WRITE) {
        s_async_error_status = HAL_ERROR;
        s_async_state = QSPI_ASYNC_ERROR;
        break;
      }
      if ((qspi_w25q64_read_status() & W25Q64_SR_BUSY) != 0U) {
        break;
      }
      if (!qspi_async_start_page()) {
        break;
      }
      break;

    case QSPI_ASYNC_WAIT_TX:
      if (s_async_op != QSPI_ASYNC_OP_WRITE) {
        s_async_error_status = HAL_ERROR;
        s_async_state = QSPI_ASYNC_ERROR;
        break;
      }
      if (s_async_tx_done ||
          HAL_OSPI_GetState(&hospi1) == HAL_OSPI_STATE_READY) {
        s_async_tx_done = 0;
        s_async_state = QSPI_ASYNC_WAIT_WIP;
      }
      break;

    case QSPI_ASYNC_WAIT_WIP:
      if ((qspi_w25q64_read_status() & W25Q64_SR_BUSY) != 0U) {
        break;
      }
      if (s_async_op == QSPI_ASYNC_OP_ERASE_4K) {
        s_async_state = QSPI_ASYNC_DONE;
        break;
      }
      s_async_addr += s_async_chunk;
      s_async_buf += s_async_chunk;
      s_async_remaining -= s_async_chunk;
      s_async_chunk = 0;
      if (s_async_remaining == 0U) {
        s_async_state = QSPI_ASYNC_DONE;
      } else {
        s_async_state = QSPI_ASYNC_START;
      }
      break;

    case QSPI_ASYNC_DONE:
    case QSPI_ASYNC_ERROR:
    case QSPI_ASYNC_IDLE:
    default:
      break;
  }

  if (s_async_state == QSPI_ASYNC_ERROR && !s_async_error_reported) {
    s_async_error_reported = 1;
    qspi_log_ospi_error(
        (s_async_op == QSPI_ASYNC_OP_ERASE_4K) ? "async erase" : "async write",
        s_async_error_status);
  }

  if (s_async_state == QSPI_ASYNC_DONE) {
    s_async_state = QSPI_ASYNC_IDLE;
    s_async_op = QSPI_ASYNC_OP_NONE;
    return QSPI_W25Q64_ASYNC_DONE;
  }

  if (s_async_state == QSPI_ASYNC_ERROR) {
    s_async_state = QSPI_ASYNC_IDLE;
    s_async_op = QSPI_ASYNC_OP_NONE;
    return QSPI_W25Q64_ASYNC_ERROR;
  }

  if (s_async_state == QSPI_ASYNC_IDLE) {
    s_async_op = QSPI_ASYNC_OP_NONE;
    return QSPI_W25Q64_ASYNC_IDLE;
  }

  return QSPI_W25Q64_ASYNC_BUSY;
}

bool qspi_w25q64_erase_sector_async_start(uint32_t addr) {
  if (!s_initialized) {
    return false;
  }

  if (addr >= W25Q64_FLASH_SIZE) {
    return false;
  }

  /* Align to sector boundary */
  addr &= ~(W25Q64_SECTOR_SIZE - 1);

  if (s_async_state == QSPI_ASYNC_START ||
      s_async_state == QSPI_ASYNC_WAIT_TX ||
      s_async_state == QSPI_ASYNC_WAIT_WIP) {
    return false;
  }

  if ((qspi_w25q64_read_status() & W25Q64_SR_BUSY) != 0U) {
    return false;
  }

  /* Enable write */
  if (!qspi_write_enable()) {
    return false;
  }

  /* Configure command for sector erase */
  OSPI_RegularCmdTypeDef cmd = {0};
  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
  cmd.FlashId = HAL_OSPI_FLASH_ID_1;
  cmd.Instruction = W25Q64_CMD_SECTOR_ERASE_4K;
  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  cmd.Address = addr;
  cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
  cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
  cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  cmd.DataMode = HAL_OSPI_DATA_NONE;
  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  HAL_StatusTypeDef status =
      HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
  if (status != HAL_OK) {
    qspi_log_ospi_error("erase async", status);
    return false;
  }

  s_async_op = QSPI_ASYNC_OP_ERASE_4K;
  s_async_state = QSPI_ASYNC_WAIT_WIP;
  s_async_error_status = HAL_OK;
  s_async_error_reported = 0;
  s_async_addr = addr;
  s_async_chunk = 0;
  s_async_remaining = 0;
  s_async_buf = NULL;
  s_async_tx_done = 0;

  return true;
}

bool qspi_w25q64_erase_sector_4k(uint32_t addr) {
  if (!s_initialized) {
    return false;
  }

  if (addr >= W25Q64_FLASH_SIZE) {
    return false;
  }

  /* Align to sector boundary */
  addr &= ~(W25Q64_SECTOR_SIZE - 1);

  /* Wait for any previous operation to complete */
  if (!qspi_wait_ready(W25Q64_TIMEOUT_SECTOR_ERASE)) {
    return false;
  }

  /* Enable write */
  if (!qspi_write_enable()) {
    return false;
  }

  /* Configure command for sector erase */
  OSPI_RegularCmdTypeDef cmd = {0};
  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
  cmd.FlashId = HAL_OSPI_FLASH_ID_1;
  cmd.Instruction = W25Q64_CMD_SECTOR_ERASE_4K;
  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  cmd.Address = addr;
  cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
  cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
  cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  cmd.DataMode = HAL_OSPI_DATA_NONE;
  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return false;
  }

  /* Wait for erase to complete */
  return qspi_wait_ready(W25Q64_TIMEOUT_SECTOR_ERASE);
}

bool qspi_w25q64_erase_block_64k(uint32_t addr) {
  if (!s_initialized) {
    return false;
  }

  if (addr >= W25Q64_FLASH_SIZE) {
    return false;
  }

  /* Align to block boundary */
  addr &= ~(W25Q64_BLOCK_64K_SIZE - 1);

  /* Wait for any previous operation to complete */
  if (!qspi_wait_ready(W25Q64_TIMEOUT_BLOCK_ERASE_64K)) {
    return false;
  }

  /* Enable write */
  if (!qspi_write_enable()) {
    return false;
  }

  /* Configure command for block erase */
  OSPI_RegularCmdTypeDef cmd = {0};
  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
  cmd.FlashId = HAL_OSPI_FLASH_ID_1;
  cmd.Instruction = W25Q64_CMD_BLOCK_ERASE_64K;
  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  cmd.Address = addr;
  cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
  cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
  cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  cmd.DataMode = HAL_OSPI_DATA_NONE;
  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return false;
  }

  /* Wait for erase to complete */
  return qspi_wait_ready(W25Q64_TIMEOUT_BLOCK_ERASE_64K);
}

uint8_t qspi_w25q64_read_status(void) {
  if (!s_initialized) {
    return 0xFF;
  }

  uint8_t status = 0xFF;
  if (!qspi_read_status_reg(W25Q64_CMD_READ_STATUS_REG1, &status)) {
    return 0xFF;
  }
  return status;
}

bool qspi_w25q64_is_busy(void) {
  if (s_async_state == QSPI_ASYNC_START ||
      s_async_state == QSPI_ASYNC_WAIT_TX ||
      s_async_state == QSPI_ASYNC_WAIT_WIP) {
    return true;
  }

  uint8_t status = qspi_w25q64_read_status();
  return (status & W25Q64_SR_BUSY) != 0;
}

bool qspi_w25q64_read_id(uint8_t *manufacturer, uint16_t *device) {
  if (manufacturer == NULL || device == NULL) {
    return false;
  }

  /* Configure command for reading ID */
  OSPI_RegularCmdTypeDef cmd = {0};
  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
  cmd.FlashId = HAL_OSPI_FLASH_ID_1;
  cmd.Instruction = W25Q64_CMD_READ_ID;
  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  cmd.DataMode = HAL_OSPI_DATA_1_LINE;
  cmd.NbData = 3;
  cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  HAL_StatusTypeDef status =
      HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
  if (status != HAL_OK) {
    qspi_log_ospi_error("read_id command", status);
    return false;
  }

  uint8_t id_buf[3];
  status = HAL_OSPI_Receive(&hospi1, id_buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
  if (status != HAL_OK) {
    qspi_log_ospi_error("read_id receive", status);
    return false;
  }

  *manufacturer = id_buf[0];
  *device = (id_buf[1] << 8) | id_buf[2];

  return true;
}

/* Internal helper functions */

static bool qspi_wait_ready(uint32_t timeout_ms) {
  uint32_t start = HAL_GetTick();

  while (qspi_w25q64_is_busy()) {
    if (HAL_GetTick() - start >= timeout_ms) {
      return false;
    }
  }

  return true;
}

static bool qspi_write_enable(void) {
  /* Configure command for write enable */
  OSPI_RegularCmdTypeDef cmd = {0};
  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
  cmd.FlashId = HAL_OSPI_FLASH_ID_1;
  cmd.Instruction = W25Q64_CMD_WRITE_ENABLE;
  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  cmd.DataMode = HAL_OSPI_DATA_NONE;
  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return false;
  }

  uint8_t status = 0xFF;
  if (!qspi_read_status_reg(W25Q64_CMD_READ_STATUS_REG1, &status)) {
    return false;
  }

  return ((status & W25Q64_SR_WEL) != 0U);
}

static void qspi_cache_clean(const void *addr, size_t len) {
  if (len == 0) return;

  /* Align to 32-byte cache line boundary */
  uintptr_t start = (uintptr_t)addr;
  uintptr_t end = start + len;
  uintptr_t aligned_start = start & ~(uintptr_t)(32U - 1U);
  uintptr_t aligned_end = (end + (32U - 1U)) & ~(uintptr_t)(32U - 1U);

  SCB_CleanDCache_by_Addr((uint32_t *)aligned_start,
                          (int32_t)(aligned_end - aligned_start));
}

static void qspi_log_ospi_error(const char *op, HAL_StatusTypeDef status) {
  APP_LOG_ERROR("OSPI %s failed status=%d err=0x%08lx", op, (int)status,
                (unsigned long)hospi1.ErrorCode);
}

static bool qspi_read_status_reg(uint8_t cmd_byte, uint8_t *status) {
  if (!s_initialized || status == NULL) {
    return false;
  }

  OSPI_RegularCmdTypeDef cmd = {0};
  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
  cmd.FlashId = HAL_OSPI_FLASH_ID_1;
  cmd.Instruction = cmd_byte;
  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  cmd.DataMode = HAL_OSPI_DATA_1_LINE;
  cmd.NbData = 1;
  cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return false;
  }

  if (HAL_OSPI_Receive(&hospi1, status, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return false;
  }

  return true;
}

static bool qspi_write_status(uint8_t sr1, uint8_t sr2) {
  if (!s_initialized) {
    return false;
  }

  if (!qspi_wait_ready(W25Q64_TIMEOUT_PAGE_PROGRAM)) {
    return false;
  }

  if (!qspi_write_enable()) {
    return false;
  }

  uint8_t status_bytes[2] = {sr1, sr2};

  OSPI_RegularCmdTypeDef cmd = {0};
  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
  cmd.FlashId = HAL_OSPI_FLASH_ID_1;
  cmd.Instruction = W25Q64_CMD_WRITE_STATUS_REG;
  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  cmd.DataMode = HAL_OSPI_DATA_1_LINE;
  cmd.NbData = sizeof(status_bytes);
  cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return false;
  }

  if (HAL_OSPI_Transmit(&hospi1, status_bytes, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return false;
  }

  return qspi_wait_ready(W25Q64_TIMEOUT_PAGE_PROGRAM);
}

static void qspi_clear_block_protection(void) {
  uint8_t sr1 = 0xFF;
  uint8_t sr2 = 0xFF;

  if (!qspi_read_status_reg(W25Q64_CMD_READ_STATUS_REG1, &sr1) ||
      !qspi_read_status_reg(W25Q64_CMD_READ_STATUS_REG2, &sr2)) {
    APP_LOG_WARN("QSPI status read failed");
    return;
  }

  uint8_t new_sr1 = (uint8_t)(sr1 & ~0x1CU);  /* Clear BP[2:0] */
  uint8_t new_sr2 = (uint8_t)(sr2 & ~(1U << 6));  /* Clear CMP */

  if (new_sr1 == sr1 && new_sr2 == sr2) {
    return;
  }

  if (!qspi_write_status(new_sr1, new_sr2)) {
    APP_LOG_WARN("QSPI status write failed");
    return;
  }

  if (!qspi_read_status_reg(W25Q64_CMD_READ_STATUS_REG1, &sr1) ||
      !qspi_read_status_reg(W25Q64_CMD_READ_STATUS_REG2, &sr2)) {
    APP_LOG_WARN("QSPI status verify failed");
    return;
  }

  if ((sr1 & 0x1CU) != 0U || (sr2 & (1U << 6)) != 0U) {
    APP_LOG_WARN("QSPI block protection still enabled (sr1=0x%02x sr2=0x%02x)",
                 sr1, sr2);
  } else {
    APP_LOG_INFO("QSPI block protection cleared (sr1=0x%02x sr2=0x%02x)",
                 sr1, sr2);
  }
}

static bool qspi_async_start_page(void) {
  size_t page_offset = s_async_addr % W25Q64_PAGE_SIZE;
  size_t chunk = W25Q64_PAGE_SIZE - page_offset;
  if (chunk > s_async_remaining) {
    chunk = s_async_remaining;
  }

  if (!qspi_write_enable()) {
    s_async_error_status = HAL_ERROR;
    s_async_state = QSPI_ASYNC_ERROR;
    return false;
  }

  qspi_cache_clean(s_async_buf, chunk);

  OSPI_RegularCmdTypeDef cmd = {0};
  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
  cmd.FlashId = HAL_OSPI_FLASH_ID_1;
  cmd.Instruction = W25Q64_CMD_PAGE_PROGRAM;
  cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  cmd.Address = s_async_addr;
  cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
  cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
  cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
  cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  cmd.DataMode = HAL_OSPI_DATA_1_LINE;
  cmd.NbData = chunk;
  cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
  cmd.DummyCycles = 0;
  cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
  cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

  HAL_StatusTypeDef status =
      HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
  if (status != HAL_OK) {
    s_async_error_status = status;
    s_async_state = QSPI_ASYNC_ERROR;
    return false;
  }

  bool use_dma = ((chunk & 0x3U) == 0U);

  if (use_dma) {
    status = HAL_OSPI_Transmit_DMA(&hospi1, (uint8_t *)s_async_buf);
  } else {
    APP_LOG_WARN("QSPI DMA fallback: length not word-aligned addr=0x%06lx len=%lu",
                 (unsigned long)s_async_addr, (unsigned long)chunk);
    status = HAL_OSPI_Transmit_IT(&hospi1, (uint8_t *)s_async_buf);
  }
  if (status != HAL_OK) {
    s_async_error_status = status;
    s_async_state = QSPI_ASYNC_ERROR;
    return false;
  }

  s_async_chunk = chunk;
  s_async_tx_done = 0;
  s_async_state = QSPI_ASYNC_WAIT_TX;
  return true;
}

void HAL_OSPI_TxCpltCallback(OSPI_HandleTypeDef *hospi) {
  if (hospi != &hospi1) {
    return;
  }

  if (s_async_state == QSPI_ASYNC_WAIT_TX) {
    s_async_tx_done = 1;
  }
}

void HAL_OSPI_ErrorCallback(OSPI_HandleTypeDef *hospi) {
  if (hospi != &hospi1) {
    return;
  }

  if (s_async_state != QSPI_ASYNC_IDLE) {
    s_async_error_status = HAL_ERROR;
    s_async_state = QSPI_ASYNC_ERROR;
  }
}
