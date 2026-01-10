#include "qspi_w25q64.h"

#include "main.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* External OSPI handle from main.c */
extern OSPI_HandleTypeDef hospi1;

/* Initialization flag */
static bool s_initialized = false;

/* Internal helper functions */
static bool qspi_wait_ready(uint32_t timeout_ms);
static bool qspi_write_enable(void);
static void qspi_cache_clean(const void *addr, size_t len);
static void qspi_cache_invalidate(void *addr, size_t len);

void qspi_w25q64_init(void) {
  /* OCTOSPI peripheral already initialized in main.c via MX_OCTOSPI1_Init() */
  /* Just verify we can communicate with the flash */

  uint8_t manufacturer;
  uint16_t device;

  if (qspi_w25q64_read_id(&manufacturer, &device)) {
    if (manufacturer == 0xEF && device == 0x4017) {
      s_initialized = true;
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

  /* Receive data */
  if (HAL_OSPI_Receive(&hospi1, buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return false;
  }

  /* Invalidate D-Cache for DMA coherency */
  qspi_cache_invalidate(buf, len);

  return true;
}

bool qspi_w25q64_write_page(uint32_t addr, const uint8_t *buf, size_t len) {
  if (!s_initialized || buf == NULL || len == 0 || len > W25Q64_PAGE_SIZE) {
    return false;
  }

  if (addr + len > W25Q64_FLASH_SIZE) {
    return false;
  }

  /* Check if we need to split write across page boundary */
  uint32_t page_offset = addr % W25Q64_PAGE_SIZE;
  if (page_offset + len > W25Q64_PAGE_SIZE) {
    /* Split into two writes */
    size_t first_chunk = W25Q64_PAGE_SIZE - page_offset;
    if (!qspi_w25q64_write_page(addr, buf, first_chunk)) {
      return false;
    }
    return qspi_w25q64_write_page(addr + first_chunk, buf + first_chunk,
                                    len - first_chunk);
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
  qspi_cache_clean(buf, len);

  /* Configure command for page program */
  OSPI_RegularCmdTypeDef cmd = {0};
  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
  cmd.FlashId = HAL_OSPI_FLASH_ID_1;
  cmd.Instruction = W25Q64_CMD_PAGE_PROGRAM;
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

  /* Transmit data */
  if (HAL_OSPI_Transmit(&hospi1, (uint8_t *)buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) !=
      HAL_OK) {
    return false;
  }

  /* Wait for write to complete */
  return qspi_wait_ready(W25Q64_TIMEOUT_PAGE_PROGRAM);
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

  /* Configure command for reading status register */
  OSPI_RegularCmdTypeDef cmd = {0};
  cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
  cmd.FlashId = HAL_OSPI_FLASH_ID_1;
  cmd.Instruction = W25Q64_CMD_READ_STATUS_REG1;
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
    return 0xFF;
  }

  uint8_t status = 0;
  if (HAL_OSPI_Receive(&hospi1, &status, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return 0xFF;
  }

  return status;
}

bool qspi_w25q64_is_busy(void) {
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

  if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return false;
  }

  uint8_t id_buf[3];
  if (HAL_OSPI_Receive(&hospi1, id_buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
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

  return true;
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

static void qspi_cache_invalidate(void *addr, size_t len) {
  if (len == 0) return;

  /* Align to 32-byte cache line boundary */
  uintptr_t start = (uintptr_t)addr;
  uintptr_t end = start + len;
  uintptr_t aligned_start = start & ~(uintptr_t)(32U - 1U);
  uintptr_t aligned_end = (end + (32U - 1U)) & ~(uintptr_t)(32U - 1U);

  SCB_InvalidateDCache_by_Addr((uint32_t *)aligned_start,
                               (int32_t)(aligned_end - aligned_start));
}
