#include "param_storage.h"
#include "config_control.h"
#include "crc32.h"
#include "stm32h7xx_hal.h"

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

/**
 * @brief Calculate record size (header + data + CRC, 32-byte aligned)
 */
static uint32_t param_record_size(uint32_t data_len)
{
    uint32_t raw_size = PARAM_HEADER_SIZE + data_len + PARAM_CRC_SIZE;
    return (raw_size + PARAM_FLASHWORD_SIZE - 1U) & ~(PARAM_FLASHWORD_SIZE - 1U);
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
 * Interrupts are disabled during the erase operation because:
 * 1. STM32H7 flash operations block code execution from the same bank
 * 2. If ISRs reside in Bank 1 (common), they cannot execute during erase
 * 3. This prevents system hangs from missed interrupts or flash errors
 */
static int param_erase_sector(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
        __set_PRIMASK(primask);
        return PARAM_ERR_FLASH;
    }

    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = PARAM_VOLTAGE_RANGE;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = param_get_sector(PARAM_FLASH_BASE);
    erase.NbSectors = 1U;

    uint32_t sector_error = 0U;
    status = HAL_FLASHEx_Erase(&erase, &sector_error);

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
    /* CRC over header + data */
    uint8_t buf[PARAM_HEADER_SIZE + sizeof(robot_params_t)];
    memcpy(buf, header, PARAM_HEADER_SIZE);
    memcpy(buf + PARAM_HEADER_SIZE, data, data_len);
    return robot_crc32(buf, PARAM_HEADER_SIZE + data_len);
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

    if (header->version != PARAM_VERSION)
    {
        /* TODO: Handle version migration here */
        return false;
    }

    if (header->data_length != sizeof(robot_params_t))
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
        memcpy(params_out, data, sizeof(robot_params_t));
    }
    return true;
}

/**
 * @brief Scan sector to find latest valid record and next write address
 */
static void param_scan_sector(void)
{
    s_param.next_write_addr = PARAM_FLASH_BASE;
    s_param.current_sequence = 0U;
    s_param.record_count = 0U;
    s_param.cache_valid = false;

    uint32_t addr = PARAM_FLASH_BASE;
    uint32_t record_size = param_record_size(sizeof(robot_params_t));

    while (addr + record_size <= PARAM_FLASH_END)
    {
        if (param_region_erased(addr, PARAM_FLASHWORD_SIZE))
        {
            /* Found first erased slot */
            s_param.next_write_addr = addr;
            break;
        }

        param_header_t header;
        robot_params_t params;
        if (param_validate_record(addr, &header, &params))
        {
            if (header.sequence >= s_param.current_sequence)
            {
                s_param.current_sequence = header.sequence;
                memcpy(&s_param.cached_params, &params, sizeof(robot_params_t));
                s_param.cache_valid = true;
            }
            s_param.record_count++;
        }

        addr += record_size;
        s_param.next_write_addr = addr;
    }

    /* If we reached the end, sector is full */
    if (addr + record_size > PARAM_FLASH_END)
    {
        s_param.next_write_addr = PARAM_FLASH_END;
    }
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
        return PARAM_OK;
    }

    /* No valid parameters found, use defaults */
    param_storage_get_defaults(params);
    return PARAM_ERR_NOT_FOUND;
}

int param_storage_save(const robot_params_t *params)
{
    if (params == NULL)
    {
        return PARAM_ERR;
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

/**
 * @brief Initialize PID gains to reasonable defaults
 */
static void param_init_pid(pid_gains_t *pid, float kp, float ki, float kd,
                           float max_integral, float max_output)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_integral = max_integral;
    pid->max_output = max_output;
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
    params->encoder_ticks_per_rev = 1440.0f; /* Typical encoder */

    /* IMU calibration (identity = uncalibrated) */
    param_init_imu_calib(&params->imu_bmi270);
    param_init_imu_calib(&params->imu_icm42688);

    /* Magnetometer calibration */
    param_init_mag_calib(&params->mag_bmm150);

    /* Motion controller PID defaults */
    param_init_pid(&params->motion_linear,
                   1.0f,   /* kp */
                   0.1f,   /* ki */
                   0.01f,  /* kd */
                   0.5f,   /* max_integral */
                   1.0f);  /* max_output */

    param_init_pid(&params->motion_angular,
                   2.0f,   /* kp */
                   0.2f,   /* ki */
                   0.02f,  /* kd */
                   1.0f,   /* max_integral */
                   2.0f);  /* max_output (rad/s) */

    /* Balance controller gains */
    param_init_balance_gains(&params->balance);

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
