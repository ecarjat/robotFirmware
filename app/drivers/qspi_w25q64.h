#ifndef QSPI_W25Q64_H
#define QSPI_W25Q64_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* W25Q64 Flash Specifications */
#define W25Q64_FLASH_SIZE          0x800000U  /* 8 MB */
#define W25Q64_PAGE_SIZE           256U       /* 256 bytes per page */
#define W25Q64_SECTOR_SIZE         0x1000U    /* 4 KB sector erase */
#define W25Q64_BLOCK_32K_SIZE      0x8000U    /* 32 KB block erase */
#define W25Q64_BLOCK_64K_SIZE      0x10000U   /* 64 KB block erase */

/* W25Q64 Command Set */
#define W25Q64_CMD_WRITE_ENABLE       0x06
#define W25Q64_CMD_WRITE_DISABLE      0x04
#define W25Q64_CMD_READ_STATUS_REG1   0x05
#define W25Q64_CMD_READ_STATUS_REG2   0x35
#define W25Q64_CMD_WRITE_STATUS_REG   0x01
#define W25Q64_CMD_PAGE_PROGRAM       0x02
#define W25Q64_CMD_READ_DATA          0x03
#define W25Q64_CMD_FAST_READ          0x0B
#define W25Q64_CMD_SECTOR_ERASE_4K    0x20
#define W25Q64_CMD_BLOCK_ERASE_32K    0x52
#define W25Q64_CMD_BLOCK_ERASE_64K    0xD8
#define W25Q64_CMD_CHIP_ERASE         0xC7
#define W25Q64_CMD_READ_ID            0x9F
#define W25Q64_CMD_POWER_DOWN         0xB9
#define W25Q64_CMD_RELEASE_POWER_DOWN 0xAB

/* Status Register Bits */
#define W25Q64_SR_BUSY                0x01  /* Write in progress */
#define W25Q64_SR_WEL                 0x02  /* Write enable latch */

/* Timeouts */
#define W25Q64_TIMEOUT_PAGE_PROGRAM   5     /* ms */
#define W25Q64_TIMEOUT_SECTOR_ERASE   400   /* ms */
#define W25Q64_TIMEOUT_BLOCK_ERASE_64K 2000 /* ms */
#define W25Q64_TIMEOUT_CHIP_ERASE     50000 /* ms */

/**
 * @brief Initialize W25Q64 QSPI flash
 *
 * Configures QSPI peripheral for indirect mode operation.
 * Must be called after HAL_OSPI_Init().
 */
void qspi_w25q64_init(void);

/**
 * @brief Check if flash is ready
 * @return true if initialized and not busy
 */
bool qspi_w25q64_is_ready(void);

/**
 * @brief Read data from flash
 *
 * @param addr Flash address (0x000000 - 0x7FFFFF)
 * @param buf  Buffer to store read data (must be 32-byte aligned for DMA)
 * @param len  Number of bytes to read
 * @return true if read successful
 *
 * Note: For optimal performance, use len >= 256 bytes to trigger MDMA.
 *       Performs cache invalidation automatically.
 */
bool qspi_w25q64_read(uint32_t addr, uint8_t *buf, size_t len);

/**
 * @brief Program a page (up to 256 bytes)
 *
 * @param addr Flash address (must be page-aligned for optimal performance)
 * @param buf  Data to write (must be 32-byte aligned for DMA)
 * @param len  Number of bytes (1-256)
 * @return true if write successful
 *
 * Note: Sector must be erased before programming.
 *       Performs cache clean automatically.
 *       Does NOT cross page boundaries - splits writes if needed.
 */
bool qspi_w25q64_write_page(uint32_t addr, const uint8_t *buf, size_t len);

/**
 * @brief Async write status
 */
typedef enum {
  QSPI_W25Q64_ASYNC_IDLE = 0,
  QSPI_W25Q64_ASYNC_BUSY,
  QSPI_W25Q64_ASYNC_DONE,
  QSPI_W25Q64_ASYNC_ERROR
} qspi_w25q64_async_state_t;

/**
 * @brief Start an async write (non-blocking)
 *
 * The caller must keep @p buf valid until the async write completes.
 *
 * @param addr Flash address
 * @param buf  Data to write (must be 32-byte aligned for DMA)
 * @param len  Number of bytes (>= 1)
 * @return true if the write was accepted, false otherwise
 */
bool qspi_w25q64_write_async_start(uint32_t addr, const uint8_t *buf, size_t len);

/**
 * @brief Advance async state machine (writes/erases)
 *
 * Call periodically from the main loop to progress any async operation.
 *
 * @return Current async state
 */
qspi_w25q64_async_state_t qspi_w25q64_write_async_tick(void);

/**
 * @brief Start an async 4 KB sector erase (non-blocking)
 *
 * @param addr Sector start address (must be 4KB-aligned)
 * @return true if the erase was accepted, false otherwise
 */
bool qspi_w25q64_erase_sector_async_start(uint32_t addr);

/**
 * @brief Erase 4 KB sector
 *
 * @param addr Sector start address (must be 4KB-aligned)
 * @return true if erase successful
 *
 * Note: Blocks for ~45-400ms depending on flash condition.
 *       Check qspi_w25q64_is_busy() before calling in time-critical code.
 */
bool qspi_w25q64_erase_sector_4k(uint32_t addr);

/**
 * @brief Erase 64 KB block
 *
 * @param addr Block start address (must be 64KB-aligned)
 * @return true if erase successful
 *
 * Note: Blocks for up to 2 seconds.
 */
bool qspi_w25q64_erase_block_64k(uint32_t addr);

/**
 * @brief Read status register
 * @return Status register value (SR_BUSY, SR_WEL bits)
 */
uint8_t qspi_w25q64_read_status(void);

/**
 * @brief Check if flash is busy (write/erase in progress)
 * @return true if busy
 */
bool qspi_w25q64_is_busy(void);

/**
 * @brief Read manufacturer and device ID
 *
 * @param manufacturer Output: Manufacturer ID (expected: 0xEF for Winbond)
 * @param device       Output: Device ID (expected: 0x4017 for W25Q64)
 * @return true if read successful
 */
bool qspi_w25q64_read_id(uint8_t *manufacturer, uint16_t *device);

#ifdef __cplusplus
}
#endif

#endif /* QSPI_W25Q64_H */
