#include "param_storage.h"
#include "app_config.h"
#include "config_control.h"
#include "crc32.h"
#include "debug_wdog.h"
#include "motion_modes.h"
#include "stm32h7xx_hal.h"

#include "../app/logging/blackbox_format.h"
#include <string.h>

/**
 * @brief Parameter storage implementation with wear-leveling
 *
 * Uses an append-only write strategy to extend flash endurance.
 * Records are written sequentially until the sector fills,
 * then the entire sector is erased and writing restarts.
 *
 * STM32H7 flash constraints:
 *   - Minimum erase unit: 128 KB sector
 *   - Programming unit: 32 bytes (flashword)
 *   - Typical endurance: 10,000 erase cycles per sector
 */

#define PARAM_DCACHE_LINE_SIZE  32U
#define PARAM_VOLTAGE_RANGE     FLASH_VOLTAGE_RANGE_3

/* Linker symbols */
extern uint32_t _param_flash_start;
extern uint32_t _param_flash_end;
extern uint32_t _param_flash_size;

/* Module state */
static struct {
    uint32_t next_write_addr;   /* Next available write address */
    uint32_t current_sequence;  /* Sequence of last valid record */
    uint32_t record_count;      /* Records since last erase */
    bool     initialized;
    robot_params_t cached_params;
    bool     cache_valid;
} s_param;

static robot_params_t s_param_scan_params;
static uint8_t s_param_crc_buf[PARAM_HEADER_SIZE + sizeof(robot_params_t)]
    __attribute__((aligned(4)));

/**
 * @brief Calculate record size (header + data + CRC, 32-byte aligned)
 */
static uint32_t param_record_size(uint32_t data_len)
{
    uint32_t raw_size = PARAM_HEADER_SIZE + data_len + PARAM_CRC_SIZE;
    return (raw_size + PARAM_FLASHWORD_SIZE - 1U) & ~(PARAM_FLASHWORD_SIZE - 1U);
}

/**
 * @brief Migrate parameters from older versions to current schema
 *
 * Migration assumes older versions are prefix-compatible. When making
 * non-append changes, add explicit per-version transforms here.
 */
static bool param_migrate_params(const param_header_t *header, const uint8_t *data,
                                 robot_params_t *params_out)
{
    if (header == NULL || data == NULL || params_out == NULL)
    {
        return false;
    }

    if (header->version > PARAM_VERSION)
    {
        return false;
    }

    if (header->data_length == 0U || header->data_length > sizeof(robot_params_t))
    {
        return false;
    }

    param_storage_get_defaults(params_out);
    memcpy(params_out, data, header->data_length);
    return true;
}

/**
 * @brief Check if a flash region is erased (all 0xFF)
 */
static bool param_region_erased(uint32_t address, uint32_t length)
{
    const uint8_t *ptr = (const uint8_t *)address;
    for (uint32_t i = 0U; i < length; i++)
    {
        if (ptr[i] != 0xFFU)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief Invalidate D-cache for a flash region
 */
static void param_cache_invalidate(uint32_t address, uint32_t length)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if ((SCB->CCR & SCB_CCR_DC_Msk) == 0U || length == 0U)
    {
        return;
    }
    uint32_t start = address & ~(PARAM_DCACHE_LINE_SIZE - 1U);
    uint32_t end = address + length;
    if (end > start)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
    }
#else
    (void)address;
    (void)length;
#endif
}

/**
 * @brief Get flash sector index for an address
 */
static uint32_t param_get_sector(uint32_t address)
{
    return (address - FLASH_BASE) / FLASH_SECTOR_SIZE;
}

/**
 * @brief Erase the parameter sector
 *
 * Keep interrupts enabled during the long erase operation to minimize
 * latency impact. Note that if ISRs execute from the same flash bank,
 * instruction fetches will still stall while the erase is active.
 */
static int param_erase_sector(void)
{
    debug_wdog_refresh();
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
        __set_PRIMASK(primask);
        return PARAM_ERR_FLASH;
    }
    __set_PRIMASK(primask);

    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = PARAM_VOLTAGE_RANGE;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = param_get_sector(PARAM_FLASH_BASE);
    erase.NbSectors = 1U;

    uint32_t sector_error = 0U;
    status = HAL_FLASHEx_Erase(&erase, &sector_error);
    debug_wdog_refresh();

    __disable_irq();
    HAL_FLASH_Lock();
    __set_PRIMASK(primask);

    if (status != HAL_OK)
    {
        return PARAM_ERR_FLASH;
    }

    param_cache_invalidate(PARAM_FLASH_BASE, PARAM_FLASH_SIZE);
    return PARAM_OK;
}

/**
 * @brief Program data to flash (must be 32-byte aligned)
 *
 * Interrupts are disabled during programming for the same reasons as erase.
 * Programming is faster than erase, but still blocks Bank 1 execution.
 */
static int param_program(uint32_t address, const uint8_t *data, uint32_t length)
{
    if ((address % PARAM_FLASHWORD_SIZE) != 0U)
    {
        return PARAM_ERR_FLASH;
    }

    debug_wdog_refresh();
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
        __set_PRIMASK(primask);
        return PARAM_ERR_FLASH;
    }

    uint32_t remaining = length;
    uint32_t write_addr = address;
    const uint8_t *src = data;
    uint32_t flashword[PARAM_FLASHWORD_SIZE / sizeof(uint32_t)];
    int result = PARAM_OK;

    while (remaining > 0U)
    {
        uint32_t chunk = (remaining >= PARAM_FLASHWORD_SIZE) ?
                         PARAM_FLASHWORD_SIZE : remaining;

        /* Pad with 0xFF if partial flashword */
        memset(flashword, 0xFF, PARAM_FLASHWORD_SIZE);
        memcpy(flashword, src, chunk);

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                                   write_addr, (uint32_t)flashword);
        if (status != HAL_OK)
        {
            result = PARAM_ERR_FLASH;
            break;
        }

        write_addr += PARAM_FLASHWORD_SIZE;
        src += chunk;
        remaining = (remaining > chunk) ? (remaining - chunk) : 0U;
    }

    HAL_FLASH_Lock();
    __set_PRIMASK(primask);

    if (result == PARAM_OK)
    {
        param_cache_invalidate(address, length);
    }
    return result;
}

/**
 * @brief Compute CRC32 of a record (excluding the CRC field)
 */
static uint32_t param_compute_crc(const param_header_t *header,
                                   const uint8_t *data, uint32_t data_len)
{
    if (data_len > sizeof(robot_params_t))
    {
        return 0U;
    }

    /* CRC over header + data (use static buffer to reduce stack usage). */
    memcpy(s_param_crc_buf, header, PARAM_HEADER_SIZE);
    memcpy(s_param_crc_buf + PARAM_HEADER_SIZE, data, data_len);
    return robot_crc32(s_param_crc_buf, PARAM_HEADER_SIZE + data_len);
}

/**
 * @brief Validate a parameter record
 */
static bool param_validate_record(uint32_t address, param_header_t *header_out,
                                   robot_params_t *params_out)
{
    if (param_region_erased(address, PARAM_FLASHWORD_SIZE))
    {
        return false;
    }

    const param_header_t *header = (const param_header_t *)address;

    if (header->magic != PARAM_MAGIC)
    {
        return false;
    }

    if (header->version > PARAM_VERSION)
    {
        return false;
    }

    if (header->data_length == 0U || header->data_length > sizeof(robot_params_t))
    {
        return false;
    }

    uint32_t record_size = param_record_size(header->data_length);
    if (address + record_size > PARAM_FLASH_END)
    {
        return false;
    }

    const uint8_t *data = (const uint8_t *)(address + PARAM_HEADER_SIZE);
    uint32_t crc_offset = PARAM_HEADER_SIZE + header->data_length;
    /* Align CRC offset to 4-byte boundary */
    crc_offset = (crc_offset + 3U) & ~3U;
    const uint32_t *stored_crc = (const uint32_t *)(address + crc_offset);

    uint32_t computed = param_compute_crc(header, data, header->data_length);
    if (computed != *stored_crc)
    {
        return false;
    }

    if (header_out != NULL)
    {
        memcpy(header_out, header, sizeof(param_header_t));
    }
    if (params_out != NULL)
    {
        if (!param_migrate_params(header, data, params_out))
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief Scan sector to find latest valid record and next write address
 */
static void param_scan_sector(void)
{
    WDOG_CHECKPOINT(WDOG_CP_PARAM_SCAN_START);
    s_param.next_write_addr = PARAM_FLASH_BASE;
    s_param.current_sequence = 0U;
    s_param.record_count = 0U;
    s_param.cache_valid = false;

    uint32_t addr = PARAM_FLASH_BASE;

    while (addr + PARAM_FLASHWORD_SIZE <= PARAM_FLASH_END)
    {
        debug_wdog_refresh();

        if (param_region_erased(addr, PARAM_FLASHWORD_SIZE))
        {
            /* Found first erased slot */
            s_param.next_write_addr = addr;
            break;
        }

        param_header_t header;
        memcpy(&header, (const void *)addr, sizeof(header));
        if (header.magic != PARAM_MAGIC)
        {
            s_param.next_write_addr = addr;
            break;
        }

        if (header.data_length == 0U || header.data_length > sizeof(robot_params_t))
        {
            s_param.next_write_addr = addr;
            break;
        }

        uint32_t record_size = param_record_size(header.data_length);
        if (addr + record_size > PARAM_FLASH_END)
        {
            s_param.next_write_addr = addr;
            break;
        }

        if (param_validate_record(addr, &header, &s_param_scan_params))
        {
            if (header.sequence >= s_param.current_sequence)
            {
                s_param.current_sequence = header.sequence;
                memcpy(&s_param.cached_params, &s_param_scan_params,
                       sizeof(robot_params_t));
                s_param.cache_valid = true;
            }
            s_param.record_count++;
        }

        addr += record_size;
        s_param.next_write_addr = addr;
    }

    /* If we reached the end, sector is full */
    if (addr + PARAM_FLASHWORD_SIZE > PARAM_FLASH_END)
    {
        s_param.next_write_addr = PARAM_FLASH_END;
    }
    WDOG_CHECKPOINT(WDOG_CP_PARAM_SCAN_DONE);
}

int param_storage_init(void)
{
    memset(&s_param, 0, sizeof(s_param));
    param_cache_invalidate(PARAM_FLASH_BASE, PARAM_FLASH_SIZE);
    param_scan_sector();
    s_param.initialized = true;
    return PARAM_OK;
}

int param_storage_load(robot_params_t *params)
{
    if (params == NULL)
    {
        return PARAM_ERR;
    }

    if (!s_param.initialized)
    {
        param_storage_init();
    }

    if (s_param.cache_valid)
    {
        memcpy(params, &s_param.cached_params, sizeof(robot_params_t));
        APP_LOG_INFO("Loaded params from flash");
        return PARAM_OK;
    }

    /* No valid parameters found, use defaults */
    param_storage_get_defaults(params);
    APP_LOG_INFO("No saved params, using defaults");
    return PARAM_ERR_NOT_FOUND;
}

bool param_storage_can_save(void)
{
    motion_mode_t mode = motion_modes_get();
    return (mode != MOTION_MODE_BALANCING);
}

int param_storage_save(const robot_params_t *params)
{
    if (params == NULL)
    {
        return PARAM_ERR;
    }

    /* Safety check: do not allow saves while balancing.
     * Flash operations disable interrupts for 100-2000ms which
     * would cause the robot to fall. */
    if (!param_storage_can_save())
    {
        return PARAM_ERR_BUSY;
    }

    if (!s_param.initialized)
    {
        param_storage_init();
    }

    uint32_t record_size = param_record_size(sizeof(robot_params_t));

    /* Check if we need to erase (sector full) */
    if (s_param.next_write_addr + record_size > PARAM_FLASH_END)
    {
        int rc = param_erase_sector();
        if (rc != PARAM_OK)
        {
            return rc;
        }
        s_param.next_write_addr = PARAM_FLASH_BASE;
        s_param.record_count = 0U;
    }

    /* Build record in RAM */
    uint8_t record_buf[PARAM_HEADER_SIZE + sizeof(robot_params_t) +
                       PARAM_CRC_SIZE + PARAM_FLASHWORD_SIZE]
        __attribute__((aligned(32)));
    memset(record_buf, 0xFF, sizeof(record_buf));

    if (record_size > sizeof(record_buf))
    {
        return PARAM_ERR_SIZE;
    }

    param_header_t *header = (param_header_t *)record_buf;
    header->magic = PARAM_MAGIC;
    header->version = PARAM_VERSION;
    header->sequence = s_param.current_sequence + 1U;
    header->data_length = sizeof(robot_params_t);

    uint8_t *data_ptr = record_buf + PARAM_HEADER_SIZE;
    memcpy(data_ptr, params, sizeof(robot_params_t));

    /* Compute and store CRC */
    uint32_t crc_offset = PARAM_HEADER_SIZE + sizeof(robot_params_t);
    crc_offset = (crc_offset + 3U) & ~3U;  /* Align to 4 bytes */
    uint32_t *crc_ptr = (uint32_t *)(record_buf + crc_offset);
    *crc_ptr = param_compute_crc(header, data_ptr, sizeof(robot_params_t));

    /* Program to flash */
    int rc = param_program(s_param.next_write_addr, record_buf, record_size);
    if (rc != PARAM_OK)
    {
        return rc;
    }

    /* Update state */
    s_param.current_sequence = header->sequence;
    s_param.next_write_addr += record_size;
    s_param.record_count++;
    memcpy(&s_param.cached_params, params, sizeof(robot_params_t));
    s_param.cache_valid = true;

    return PARAM_OK;
}

/**
 * @brief Initialize an IMU calibration struct to identity (no correction)
 */
static void param_init_imu_calib(imu_calib_t *calib)
{
    /* Zero biases */
    for (int i = 0; i < 3; i++)
    {
        calib->gyro_bias[i] = 0;
        calib->accel_bias[i] = 0;
    }
    /* Identity rotation matrix */
    memset(calib->rotation, 0, sizeof(calib->rotation));
    calib->rotation[0] = 1.0f;  /* [0][0] */
    calib->rotation[4] = 1.0f;  /* [1][1] */
    calib->rotation[8] = 1.0f;  /* [2][2] */
}

/**
 * @brief Initialize a magnetometer calibration struct to identity
 */
static void param_init_mag_calib(mag_calib_t *calib)
{
    /* Zero hard iron offset */
    for (int i = 0; i < 3; i++)
    {
        calib->hard_iron[i] = 0;
    }
    /* Identity soft iron matrix */
    memset(calib->soft_iron, 0, sizeof(calib->soft_iron));
    calib->soft_iron[0] = 1.0f;  /* [0][0] */
    calib->soft_iron[4] = 1.0f;  /* [1][1] */
    calib->soft_iron[8] = 1.0f;  /* [2][2] */
}

static void param_init_balance_gains(balance_gains_t *gains)
{
    gains->Kp_theta = BALANCE_DEFAULT_KP_THETA;
    gains->Kd_theta = BALANCE_DEFAULT_KD_THETA;
    gains->Kp_v_to_theta = BALANCE_DEFAULT_KP_V_TO_THETA;
    gains->Ki_v_to_theta = BALANCE_DEFAULT_KI_V_TO_THETA;
    gains->max_tilt_ref = BALANCE_DEFAULT_MAX_TILT_REF;
    gains->Kv_damp = BALANCE_DEFAULT_KV_DAMP;
    gains->K_turn = BALANCE_DEFAULT_K_TURN;
    gains->K_yawDamp = BALANCE_DEFAULT_K_YAW_DAMP;
    gains->alpha_yaw = BALANCE_DEFAULT_ALPHA_YAW;
    gains->IqMax = BALANCE_DEFAULT_IQ_MAX;
    gains->thetaKill = BALANCE_DEFAULT_THETA_KILL;
    gains->iV_max = BALANCE_DEFAULT_IV_MAX;
}

static void param_init_lqr_params(lqr_params_t *lqr)
{
    lqr->K[0] = LQR_K0_X;
    lqr->K[1] = LQR_K1_V;
    lqr->K[2] = LQR_K2_THETA;
    lqr->K[3] = LQR_K3_THETADOT;
    lqr->u_limit = LQR_U_LIMIT;
    lqr->du_limit = LQR_DU_LIMIT;
    lqr->theta_ref_limit = LQR_THETA_REF_LIMIT;
    lqr->v_ref_limit = LQR_V_REF_LIMIT;
    lqr->engage_ramp_ms = LQR_ENGAGE_RAMP_MS;
    lqr->disengage_ramp_ms = LQR_DISENGAGE_RAMP_MS;
    lqr->default_mode = 0;  /* PID by default */
    memset(lqr->reserved, 0, sizeof(lqr->reserved));
}

void param_storage_get_defaults(robot_params_t *params)
{
    if (params == NULL)
    {
        return;
    }

    memset(params, 0, sizeof(robot_params_t));

    /* Motor direction (1 = forward, -1 = reversed) */
    params->motor_direction[0] = 1;  /* Left motor */
    params->motor_direction[1] = 1;  /* Right motor */

    /* Wheel geometry defaults */
    params->wheel_radius_m = 0.033f;         /* 33mm radius */
    params->wheel_base_m = 0.15f;            /* 150mm wheel base */

    /* IMU calibration (identity = uncalibrated) */
    param_init_imu_calib(&params->imu_bmi270);
    param_init_imu_calib(&params->imu_icm42688);

    /* Magnetometer calibration */
    param_init_mag_calib(&params->mag_bmm150);

    /* Balance controller gains */
    param_init_balance_gains(&params->balance);

    /* LQR inner loop parameters */
    param_init_lqr_params(&params->lqr);

    /* Motion limits */
    params->max_linear_vel_mps = 0.5f;
    params->max_angular_vel_rps = 2.0f;
    params->max_linear_accel_mps2 = 1.0f;
    params->max_angular_accel_rps2 = 4.0f;

    /* Control loop */
    params->control_rate_hz = CONTROL_DEFAULT_HZ;

    /* Communication */
    params->uart_baudrate = 115200U;
    params->robot_id = 1U;

    /* ADC voltage divider */
    params->adc_voltage_multiplier = 1.0f; /* Default 1:1, no voltage division */

    /* Blackbox logging */
    params->log_fields_mask = LOG_FIELDS_MASK_DEFAULT; /* IMU, EKF, wheels, PID */
    params->dump_seconds_default = 30U; /* 30 seconds default dump window */

    /* Hip calibration defaults */
    params->hip_left_zero_offset_rev = 0.0f;
    params->hip_right_zero_offset_rev = 0.0f;
    params->hip_left_dir_sign = 1;
    params->hip_right_dir_sign = 1;
}

int param_storage_erase(void)
{
    int rc = param_erase_sector();
    if (rc == PARAM_OK)
    {
        s_param.next_write_addr = PARAM_FLASH_BASE;
        s_param.current_sequence = 0U;
        s_param.record_count = 0U;
        s_param.cache_valid = false;
    }
    return rc;
}

int param_storage_stats(uint32_t *used_bytes, uint32_t *free_bytes,
                        uint32_t *record_count)
{
    if (!s_param.initialized)
    {
        param_storage_init();
    }

    uint32_t used = s_param.next_write_addr - PARAM_FLASH_BASE;
    uint32_t free = PARAM_FLASH_END - s_param.next_write_addr;

    if (used_bytes != NULL)
    {
        *used_bytes = used;
    }
    if (free_bytes != NULL)
    {
        *free_bytes = free;
    }
    if (record_count != NULL)
    {
        *record_count = s_param.record_count;
    }
    return PARAM_OK;
}

bool param_storage_is_dirty(const robot_params_t *params)
{
    if (params == NULL || !s_param.cache_valid)
    {
        return true;
    }
    return memcmp(params, &s_param.cached_params, sizeof(robot_params_t)) != 0;
}
