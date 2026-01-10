#include "motion_modes.h"

#include <math.h>

#include "config_control.h"
#include "log.h"

extern "C" {
#include "../app/logging/blackbox_dump.h"
#include "param_storage.h"
}

extern robot_params_t g_robot_params;

#define MOTION_LOG_ERROR(fmt, ...) \
    app_log_printf("[APP][ERROR] " fmt "\r\n", ##__VA_ARGS__)

static volatile motion_mode_t s_mode = MOTION_MODE_DISARMED;

void motion_modes_init(void)
{
    s_mode = MOTION_MODE_DISARMED;
}

void motion_modes_set(motion_mode_t mode)
{
    s_mode = mode;
}

motion_mode_t motion_modes_get(void)
{
    return s_mode;
}

void motion_modes_step(const motion_modes_input_t *input, motion_modes_output_t *output)
{
    if (input == NULL)
    {
        return;
    }

    motion_mode_t start_mode = s_mode;
    /* Initialize output */
    motion_modes_output_t result = {
        .mode_changed = false,
        .new_mode = s_mode,
        .disable_motors = false,
        .reset_pid = false,
    };

    motion_mode_t mode = s_mode;
    uint32_t now_ms = input->now_ms;
    bool log_exit = false;
    bool reason_kill = false;
    bool reason_imu_fallen = false;
    bool reason_motor_fallen = false;
    bool reason_imu_fatal = false;
    bool reason_motor_fatal = false;
    uint32_t imu_age = 0U;
    uint32_t motor_age = 0U;

    if (start_mode == MOTION_MODE_BALANCING)
    {
        if (input->last_imu_ok_ms != 0U)
        {
            imu_age = now_ms - input->last_imu_ok_ms;
        }
        if (input->last_motor_ok_ms != 0U)
        {
            motor_age = now_ms - input->last_motor_ok_ms;
        }
        reason_kill = input->estimate_valid &&
                      input->theta_kill_rad > 0.0f &&
                      fabsf(input->theta_rad) > input->theta_kill_rad;
        reason_imu_fallen = (input->last_imu_ok_ms != 0U) &&
                            (imu_age > IMU_FAULT_FALLEN_MS);
        reason_motor_fallen = (input->last_motor_ok_ms != 0U) &&
                              (motor_age > MOTOR_LINK_FAULT_FALLEN_MS);
        reason_imu_fatal = (input->last_imu_ok_ms != 0U) &&
                           (imu_age > IMU_FAULT_FATAL_MS);
        reason_motor_fatal = (input->last_motor_ok_ms != 0U) &&
                             (motor_age > MOTOR_LINK_FAULT_FATAL_MS);
    }

    /*
     * BALANCING → FALLEN transitions:
     * - Kill-switch: |theta| > thetaKill
     * - IMU fault: no valid IMU for > IMU_FAULT_FALLEN_MS
     * - Motor fault: no motor response for > MOTOR_LINK_FAULT_FALLEN_MS
     */
    if (mode == MOTION_MODE_BALANCING)
    {
        bool trigger_fallen = false;

        /* Kill-switch check */
        if (input->estimate_valid &&
            input->theta_kill_rad > 0.0f &&
            fabsf(input->theta_rad) > input->theta_kill_rad)
        {
            trigger_fallen = true;
        }

        /* IMU fault check */
        if (input->last_imu_ok_ms != 0U &&
            (now_ms - input->last_imu_ok_ms) > IMU_FAULT_FALLEN_MS)
        {
            trigger_fallen = true;
        }

        /* Motor link fault check */
        if (input->last_motor_ok_ms != 0U &&
            (now_ms - input->last_motor_ok_ms) > MOTOR_LINK_FAULT_FALLEN_MS)
        {
            trigger_fallen = true;
        }

        if (trigger_fallen)
        {
            mode = MOTION_MODE_FALLEN;
            result.mode_changed = true;
            result.new_mode = mode;
            result.disable_motors = true;
            result.reset_pid = true;
            s_mode = mode;
            log_exit = (start_mode == MOTION_MODE_BALANCING);

            /* Trigger blackbox dump on fall */
            uint32_t dump_seconds = g_robot_params.dump_seconds_default;
            if (dump_seconds == 0U) {
                dump_seconds = 30U;  /* Fallback default */
            }
            if (!log_dump_last_seconds(dump_seconds)) {
                MOTION_LOG_ERROR("Auto-dump failed (already in progress or SD not ready)");
            }
        }
    }

    /*
     * Any (except DISARMED/FAULT) → FAULT transitions:
     * - Fatal IMU fault: no valid IMU for > IMU_FAULT_FATAL_MS
     * - Fatal motor fault: no motor response for > MOTOR_LINK_FAULT_FATAL_MS
     */
    if (mode != MOTION_MODE_DISARMED && mode != MOTION_MODE_FAULT)
    {
        bool trigger_fault = false;

        /* Fatal IMU fault */
        if (input->last_imu_ok_ms != 0U &&
            (now_ms - input->last_imu_ok_ms) > IMU_FAULT_FATAL_MS)
        {
            trigger_fault = true;
        }

        /* Fatal motor link fault */
        if (input->last_motor_ok_ms != 0U &&
            (now_ms - input->last_motor_ok_ms) > MOTOR_LINK_FAULT_FATAL_MS)
        {
            trigger_fault = true;
        }

        if (trigger_fault)
        {
            mode = MOTION_MODE_FAULT;
            result.mode_changed = true;
            result.new_mode = mode;
            result.disable_motors = true;
            result.reset_pid = true;
            s_mode = mode;
            log_exit = (start_mode == MOTION_MODE_BALANCING);
        }
    }

    if (log_exit && s_mode != MOTION_MODE_BALANCING)
    {
        if (s_mode == MOTION_MODE_FALLEN)
        {
            bool logged = false;
            if (reason_kill)
            {
                MOTION_LOG_ERROR("Mode BALANCING -> FALLEN reason=kill theta=%.3f kill=%.3f",
                                 (double)input->theta_rad,
                                 (double)input->theta_kill_rad);
                logged = true;
            }
            if (reason_imu_fallen)
            {
                MOTION_LOG_ERROR("Mode BALANCING -> FALLEN reason=imu timeout age=%lu ms",
                                 (unsigned long)imu_age);
                logged = true;
            }
            if (reason_motor_fallen)
            {
                MOTION_LOG_ERROR("Mode BALANCING -> FALLEN reason=motor timeout age=%lu ms",
                                 (unsigned long)motor_age);
                logged = true;
            }
            if (!logged)
            {
                MOTION_LOG_ERROR("Mode BALANCING -> FALLEN reason=unknown");
            }
        }
        else if (s_mode == MOTION_MODE_FAULT)
        {
            bool logged = false;
            if (reason_imu_fatal)
            {
                MOTION_LOG_ERROR("Mode BALANCING -> FAULT reason=imu fatal age=%lu ms",
                                 (unsigned long)imu_age);
                logged = true;
            }
            if (reason_motor_fatal)
            {
                MOTION_LOG_ERROR("Mode BALANCING -> FAULT reason=motor fatal age=%lu ms",
                                 (unsigned long)motor_age);
                logged = true;
            }
            if (!logged)
            {
                MOTION_LOG_ERROR("Mode BALANCING -> FAULT reason=unknown");
            }
        }
        else
        {
            MOTION_LOG_ERROR("Mode BALANCING -> %u reason=unknown",
                             (unsigned int)s_mode);
        }
    }

    if (output != NULL)
    {
        *output = result;
    }
}

bool motion_modes_allows_output(void)
{
    return (s_mode == MOTION_MODE_BALANCING);
}
