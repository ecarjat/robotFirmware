#ifndef CONTROL_MOTION_MODES_H
#define CONTROL_MOTION_MODES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MOTION_MODE_DISARMED = 0,
    MOTION_MODE_CALIBRATION,
    MOTION_MODE_BALANCING,
    MOTION_MODE_FALLEN,
    MOTION_MODE_FAULT
} motion_mode_t;

/**
 * @brief Input data for state machine transitions
 *
 * Populated by motion_control_tick() and passed to motion_modes_step()
 * to centralize all state transition logic.
 */
typedef struct
{
    uint32_t now_ms;              /* Current timestamp */

    /* Estimator data */
    bool estimate_valid;          /* EKF estimate is valid */
    float theta_rad;              /* Current pitch angle (rad) */
    float theta_kill_rad;         /* Kill-switch threshold (rad) */

    /* IMU health */
    bool imu_ok;                  /* At least one IMU is producing valid data */
    uint32_t last_imu_ok_ms;      /* Timestamp of last valid IMU sample */

    /* Motor link health */
    bool motor_ok;                /* Motor link is responding */
    uint32_t last_motor_ok_ms;    /* Timestamp of last motor response */
} motion_modes_input_t;

/**
 * @brief Output from state machine step
 */
typedef struct
{
    bool mode_changed;            /* True if mode transition occurred */
    motion_mode_t new_mode;       /* New mode (if changed) */
    bool disable_motors;          /* True if motors should be disabled */
    bool reset_pid;               /* True if PID integrators should be reset */
} motion_modes_output_t;

void motion_modes_init(void);
void motion_modes_set(motion_mode_t mode);
motion_mode_t motion_modes_get(void);

/**
 * @brief Evaluate state machine transitions based on input
 *
 * This function encapsulates all state transition logic:
 * - BALANCING → FALLEN: kill-switch, IMU fault, motor fault
 * - Any → FAULT: fatal IMU or motor link loss
 *
 * @param input Current system state
 * @param output Resulting actions (may be NULL if not needed)
 */
void motion_modes_step(const motion_modes_input_t *input, motion_modes_output_t *output);

/**
 * @brief Check if current mode allows motor output
 */
bool motion_modes_allows_output(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_MOTION_MODES_H */
