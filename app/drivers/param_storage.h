#ifndef PARAM_STORAGE_H
#define PARAM_STORAGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Parameter storage error codes
 */
#define PARAM_OK           0
#define PARAM_ERR         -1
#define PARAM_ERR_FLASH   -2
#define PARAM_ERR_CRC     -3
#define PARAM_ERR_SIZE    -4
#define PARAM_ERR_FULL    -5
#define PARAM_ERR_NOT_FOUND -6
#define PARAM_ERR_BUSY    -7

/**
 * @brief Parameter storage configuration
 *
 * The parameter sector (128 KB at 0x080C0000) uses append-only writes
 * with wear leveling. Records are written sequentially until the sector
 * fills, then the sector is erased and writing restarts from the beginning.
 *
 * Record format:
 *   - 4 bytes: magic (0x524F424F = 'ROBO')
 *   - 4 bytes: version (schema version)
 *   - 4 bytes: sequence (monotonic counter)
 *   - 4 bytes: data_length
 *   - N bytes: data payload (32-byte aligned)
 *   - 4 bytes: CRC32 of entire record (excluding this field)
 *
 * Records are always 32-byte aligned for STM32H7 flash programming.
 *
 * ============================================================================
 * FLASH ERASE TIMING vs WATCHDOG REQUIREMENTS
 * ============================================================================
 *
 * CRITICAL: Flash operations disable interrupts and block the CPU.
 *
 * Timing constraints:
 *   - STM32H7 128KB sector erase: 1000-2000ms typical, 4000ms max (datasheet)
 *   - STM32H7 32-byte flash word program: ~20us per word
 *   - Full parameter write (after erase): ~10ms
 *
 * IWDG Configuration (in Core/Src/main.c MX_IWDG1_Init):
 *   - LSI clock: 32 kHz
 *   - Prescaler: 256
 *   - Reload: 500
 *   - Timeout = (500 * 256) / 32000 = 4000ms = 4 seconds
 *
 * Safety margin analysis:
 *   - Worst-case erase: 4000ms (per datasheet max)
 *   - IWDG timeout: 4000ms
 *   - Margin: 0ms (INSUFFICIENT for worst case!)
 *
 * MITIGATIONS:
 *   1. debug_wdog_refresh() is called immediately before disabling interrupts
 *   2. param_storage_can_save() prevents saves during MOTION_MODE_BALANCING
 *   3. Typical erase time is 1-2 seconds, well within the 4-second timeout
 *
 * RISK ASSESSMENT:
 *   - In practice, erases complete in 1-2 seconds (50% of timeout)
 *   - If a flash cell is degraded, erase can approach worst-case timing
 *   - A watchdog reset during parameter save is recoverable (params not saved)
 *   - Robot must be stationary during saves (enforced by can_save check)
 *
 * RECOMMENDATIONS:
 *   - If watchdog resets occur during saves, increase IWDG timeout to 8 seconds
 *   - Consider dual-bank flash to allow erase during execution from other bank
 *   - Monitor for PARAM_ERR_FLASH returns which may indicate flash degradation
 *
 * ============================================================================
 */

#define PARAM_MAGIC             0x524F424FUL  /* 'ROBO' */
#define PARAM_VERSION           9U
#define PARAM_FLASH_BASE        0x080C0000UL
#define PARAM_FLASH_SIZE        0x00020000UL  /* 128 KB */
#define PARAM_FLASH_END         (PARAM_FLASH_BASE + PARAM_FLASH_SIZE)
#define PARAM_FLASHWORD_SIZE    32U           /* H7 programming unit */
#define PARAM_HEADER_SIZE       16U           /* magic + version + seq + len */
#define PARAM_CRC_SIZE          4U

/**
 * @brief Parameter record header (stored in flash)
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t sequence;
    uint32_t data_length;
} param_header_t;

/**
 * @brief IMU calibration parameters (per sensor)
 */
typedef struct {
    int16_t gyro_bias[3];       /* mdps offset */
    int16_t accel_bias[3];      /* mg offset */
    float   rotation[9];        /* 3x3 rotation matrix (sensor to robot frame) */
} imu_calib_t;

/**
 * @brief Magnetometer calibration parameters
 */
typedef struct {
    int16_t hard_iron[3];       /* uT offset (hard iron correction) */
    float   soft_iron[9];       /* 3x3 soft iron correction matrix */
} mag_calib_t;

/**
 * @brief Balance controller gains (outer loop)
 */
typedef struct balance_gains {
    float Kp_theta;         /* Pitch proportional gain */
    float Kd_theta;         /* Pitch derivative gain */
    float Kp_v_to_theta;    /* Velocity error -> tilt reference P gain */
    float Ki_v_to_theta;    /* Velocity error -> tilt reference I gain */
    float max_tilt_ref;     /* Max tilt reference (rad) */
    float Kv_damp;          /* Velocity damping gain */
    float K_turn;           /* Turn command gain */
    float K_yawDamp;        /* Yaw rate damping gain */
    float alpha_yaw;        /* Yaw rate blend factor */
    float IqMax;            /* Max motor current (A) */
    float thetaKill;        /* Kill-switch angle (rad) */
    float iV_max;           /* Velocity integrator limit (rad) */
} balance_gains_t;

/**
 * @brief LQR inner loop parameters
 *
 * 4-state LQR: state = [x_err, v_err, theta_err, thetaDot]
 * Control law: u_sum = -K * state
 */
typedef struct lqr_params {
    float K[4];             /* LQR gains: [K_x, K_v, K_theta, K_thetaDot] */
    float u_limit;          /* Max |u_sum| (Nm) */
    float du_limit;         /* Max rate of change |du_sum| per second (Nm/s) */
    float theta_ref_limit;  /* Max |theta_ref| (rad) */
    float v_ref_limit;      /* Max |v_ref| (m/s) */
    uint16_t engage_ramp_ms;    /* PID→LQR blend time (ms) */
    uint16_t disengage_ramp_ms; /* LQR→PID blend time (ms) */
    uint8_t default_mode;   /* 0=PID, 1=LQR at startup */
    uint8_t reserved[3];    /* Padding for alignment */
} lqr_params_t;

/**
 * @brief Robot parameters structure
 *
 * This structure contains all persistent robot configuration.
 * When adding new fields, increment PARAM_VERSION and handle
 * migration in param_storage_load().
 */
typedef struct {
    /* Motor configuration */
    int8_t motor_direction[2];  /* 1 or -1 for each motor (left/right) */

    /* Wheel geometry */
    float wheel_radius_m;
    float wheel_base_m;

    /* IMU calibration (two sensors) */
    imu_calib_t imu_bmi270;
    imu_calib_t imu_icm42688;

    /* Magnetometer calibration (BMM150) */
    mag_calib_t mag_bmm150;

    /* Balance controller gains */
    balance_gains_t balance;

    /* LQR inner loop parameters */
    lqr_params_t lqr;

    /* Motion limits */
    float max_linear_vel_mps;
    float max_angular_vel_rps;
    float max_linear_accel_mps2;
    float max_angular_accel_rps2;

    /* Control loop */
    float control_rate_hz;

    /* Communication */
    uint32_t uart_baudrate; /* not used */
    uint8_t  robot_id;

    /* ADC voltage divider (PC4) */
    float adc_voltage_multiplier; /* Voltage divider multiplier for ADC on PC4 */

    /* Blackbox logging configuration */
    uint32_t log_fields_mask;        /* Bitmask of LOGF_* flags for conditional logging */
    uint16_t dump_seconds_default;   /* Default dump window in seconds (e.g., 30) */

    /* Hip calibration / configuration */
    float hip_left_zero_offset_rev;
    float hip_right_zero_offset_rev;
    int8_t hip_left_dir_sign;   /* +1 or -1 */
    int8_t hip_right_dir_sign;  /* +1 or -1 */
    uint8_t hip_reserved[2];

    /* Reserved for future use */
    uint8_t  reserved[6];
} robot_params_t;

/**
 * @brief Initialize parameter storage
 *
 * Scans the parameter sector to find the latest valid record.
 * Call once at startup before accessing parameters.
 *
 * @return PARAM_OK on success, error code otherwise
 */
int param_storage_init(void);

/**
 * @brief Load parameters from flash
 *
 * Retrieves the most recent valid parameter set from flash.
 * If no valid parameters exist, fills with defaults.
 *
 * @param params Pointer to parameter structure to fill
 * @return PARAM_OK if valid params found, PARAM_ERR_NOT_FOUND if using defaults
 */
int param_storage_load(robot_params_t *params);

/**
 * @brief Save parameters to flash
 *
 * Appends a new parameter record to the sector. If the sector is full,
 * erases it and writes at the beginning.
 *
 * @param params Pointer to parameters to save
 * @return PARAM_OK on success, error code otherwise
 *
 * @note Returns PARAM_ERR_BUSY if robot is in BALANCING mode.
 *       Flash operations disable interrupts and could cause falls.
 */
int param_storage_save(const robot_params_t *params);

/**
 * @brief Check if parameter save is allowed
 *
 * @return true if save is allowed (robot not balancing), false otherwise
 */
bool param_storage_can_save(void);

/**
 * @brief Get default parameters
 *
 * Fills the structure with compile-time defaults.
 *
 * @param params Pointer to parameter structure to fill
 */
void param_storage_get_defaults(robot_params_t *params);

/**
 * @brief Erase parameter sector
 *
 * Erases the entire parameter sector. Use with caution.
 *
 * @return PARAM_OK on success, PARAM_ERR_FLASH on failure
 */
int param_storage_erase(void);

/**
 * @brief Get storage statistics
 *
 * @param used_bytes  Output: bytes used in sector (NULL to skip)
 * @param free_bytes  Output: bytes free in sector (NULL to skip)
 * @param record_count Output: number of records written since last erase
 * @return PARAM_OK on success
 */
int param_storage_stats(uint32_t *used_bytes, uint32_t *free_bytes,
                        uint32_t *record_count);

/**
 * @brief Check if parameters have been modified since load
 *
 * Compares current parameters against the last loaded values.
 *
 * @param params Current parameters to check
 * @return true if modified, false if unchanged
 */
bool param_storage_is_dirty(const robot_params_t *params);

#ifdef __cplusplus
}
#endif

#endif /* PARAM_STORAGE_H */
