#include "hip_behavior.h"

#include <math.h>

#include "app_config.h"
#include "app_log_macros.h"

typedef struct {
    hip_behavior_mode_t mode;
    uint32_t mode_start_ms;
    uint32_t last_update_ms;
    uint8_t jump_requested;
    float height_ref_m;
    float nominal_height_m;
    float imu_accel_z_g;
    uint8_t imu_accel_valid;
    uint8_t phase_progress;
} hip_behavior_state_t;

static hip_behavior_state_t s_behavior = {
    .mode = HIP_BEHAVIOR_NORMAL,
    .mode_start_ms = 0U,
    .last_update_ms = 0U,
    .jump_requested = 0U,
    .height_ref_m = NAN,
    .nominal_height_m = NAN,
    .imu_accel_z_g = 0.0f,
    .imu_accel_valid = 0U,
    .phase_progress = 0U,
};

static float hip_average_height(const hip_state_t *left, const hip_state_t *right)
{
    float sum = 0.0f;
    int count = 0;
    if (left && left->valid) {
        sum += left->height_m;
        count++;
    }
    if (right && right->valid) {
        sum += right->height_m;
        count++;
    }
    if (count == 0) {
        return NAN;
    }
    return sum / (float)count;
}

static void hip_behavior_set_mode(hip_behavior_mode_t mode, uint32_t now_ms)
{
    if (s_behavior.mode != mode) {
        APP_LOG_INFO("Hip behavior mode %u -> %u", (unsigned int)s_behavior.mode,
                     (unsigned int)mode);
    }
    s_behavior.mode = mode;
    s_behavior.mode_start_ms = now_ms;
    s_behavior.last_update_ms = now_ms;
}

void hip_behavior_init(float nominal_height_m)
{
    s_behavior.mode = HIP_BEHAVIOR_NORMAL;
    s_behavior.mode_start_ms = 0U;
    s_behavior.last_update_ms = 0U;
    s_behavior.jump_requested = 0U;
    s_behavior.height_ref_m = nominal_height_m;
    s_behavior.nominal_height_m = nominal_height_m;
    s_behavior.imu_accel_z_g = 0.0f;
    s_behavior.imu_accel_valid = 0U;
    s_behavior.phase_progress = 0U;
}

void hip_behavior_set_height_ref(float height_m)
{
    s_behavior.height_ref_m = height_m;
}

void hip_behavior_request_jump(void)
{
    s_behavior.jump_requested = 1U;
}

void hip_behavior_cancel_jump(void)
{
    s_behavior.jump_requested = 0U;
    hip_behavior_set_mode(HIP_BEHAVIOR_NORMAL, s_behavior.last_update_ms);
}

void hip_behavior_set_imu_accel(float accel_z_g, bool valid)
{
    s_behavior.imu_accel_z_g = accel_z_g;
    s_behavior.imu_accel_valid = valid ? 1U : 0U;
}

uint8_t hip_behavior_get_phase_progress(void)
{
    return s_behavior.phase_progress;
}

void hip_behavior_tick(uint32_t now_ms,
                       motion_mode_t motion_mode,
                       const hip_state_t *left,
                       const hip_state_t *right,
                       hip_target_t *target_out,
                       hip_behavior_mode_t *mode_out)
{
    if (target_out == NULL) {
        return;
    }

    if (s_behavior.last_update_ms == 0U) {
        s_behavior.last_update_ms = now_ms;
    }

    float measured_height = hip_average_height(left, right);
    if (isnan(s_behavior.height_ref_m)) {
        if (!isnan(measured_height)) {
            s_behavior.height_ref_m = measured_height;
        } else {
            s_behavior.height_ref_m = HIP_HEIGHT_DEFAULT_M;
        }
    }
    if (isnan(s_behavior.nominal_height_m)) {
        s_behavior.nominal_height_m = s_behavior.height_ref_m;
    }

    if (motion_mode != MOTION_MODE_BALANCING) {
        s_behavior.jump_requested = 0U;
        hip_behavior_set_mode(HIP_BEHAVIOR_NORMAL, now_ms);
    } else if (s_behavior.mode == HIP_BEHAVIOR_NORMAL && s_behavior.jump_requested) {
        s_behavior.jump_requested = 0U;
        hip_behavior_set_mode(HIP_BEHAVIOR_CROUCH, now_ms);
    }

    float height_rate = 0.0f;
    float stiffness = HIP_STIFFNESS_N_M_DEFAULT;
    float damping = HIP_DAMPING_N_S_M_DEFAULT;
    float height_dot = 0.0f;
    if (left && left->valid) {
        height_dot += left->height_dot_m_s;
    }
    if (right && right->valid) {
        height_dot += right->height_dot_m_s;
    }
    if (left && right && left->valid && right->valid) {
        height_dot *= 0.5f;
    }

    switch (s_behavior.mode) {
    case HIP_BEHAVIOR_NORMAL:
        height_rate = 0.0f;
        break;
    case HIP_BEHAVIOR_CROUCH:
        height_rate = HIP_CROUCH_RATE_MPS;
        if ((now_ms - s_behavior.mode_start_ms) >= HIP_BEHAVIOR_CROUCH_MS ||
            (!isnan(measured_height) &&
             measured_height >= (s_behavior.nominal_height_m + HIP_CROUCH_DEPTH_M))) {
            hip_behavior_set_mode(HIP_BEHAVIOR_IMPULSE, now_ms);
        }
        break;
    case HIP_BEHAVIOR_IMPULSE:
        height_rate = -HIP_IMPULSE_RATE_MPS;
        if ((now_ms - s_behavior.mode_start_ms) >= HIP_BEHAVIOR_IMPULSE_MS ||
            height_dot <= -HIP_IMPULSE_LIFTOFF_VEL_MPS ||
            (s_behavior.imu_accel_valid &&
             s_behavior.imu_accel_z_g < HIP_LIFTOFF_ACCEL_G)) {
            hip_behavior_set_mode(HIP_BEHAVIOR_FLIGHT, now_ms);
        }
        break;
    case HIP_BEHAVIOR_FLIGHT:
        height_rate = 0.0f;
        stiffness = HIP_FLIGHT_STIFFNESS_N_M;
        damping = HIP_FLIGHT_DAMPING_N_S_M;
        if ((now_ms - s_behavior.mode_start_ms) >= HIP_BEHAVIOR_FLIGHT_MS ||
            (s_behavior.imu_accel_valid &&
             s_behavior.imu_accel_z_g > HIP_LANDING_ACCEL_G)) {
            hip_behavior_set_mode(HIP_BEHAVIOR_LANDING, now_ms);
        }
        break;
    case HIP_BEHAVIOR_LANDING:
        height_rate = 0.0f;
        stiffness = HIP_LANDING_STIFFNESS_N_M;
        damping = HIP_LANDING_DAMPING_N_S_M;
        if ((now_ms - s_behavior.mode_start_ms) >= HIP_BEHAVIOR_LANDING_MS) {
            hip_behavior_set_mode(HIP_BEHAVIOR_NORMAL, now_ms);
        }
        break;
    default:
        hip_behavior_set_mode(HIP_BEHAVIOR_NORMAL, now_ms);
        break;
    }

    if (s_behavior.mode == HIP_BEHAVIOR_NORMAL &&
        motion_mode == MOTION_MODE_BALANCING) {
        if (!isnan(measured_height)) {
            s_behavior.height_ref_m = measured_height;
        }
    } else {
        float dt_s = 0.0f;
        if (s_behavior.last_update_ms != 0U && now_ms > s_behavior.last_update_ms) {
            uint32_t elapsed_ms = now_ms - s_behavior.last_update_ms;
            dt_s = 0.001f * (float)elapsed_ms;
            s_behavior.last_update_ms = now_ms;
        }
        if (dt_s > 0.0f) {
            s_behavior.height_ref_m += height_rate * dt_s;
        }
    }

    target_out->height_ref_m = s_behavior.height_ref_m;
    target_out->height_rate_ref_m_s = height_rate;
    target_out->stiffness_n_m = stiffness;
    target_out->damping_n_s_m = damping;
    target_out->mode = (s_behavior.mode == HIP_BEHAVIOR_NORMAL) ?
        HIP_MODE_HOLD : HIP_MODE_JUMP;
    target_out->enabled = (motion_mode == MOTION_MODE_BALANCING);

    if (mode_out != NULL) {
        *mode_out = s_behavior.mode;
    }

    uint32_t elapsed = now_ms - s_behavior.mode_start_ms;
    uint32_t duration = 0U;
    switch (s_behavior.mode) {
    case HIP_BEHAVIOR_CROUCH:
        duration = HIP_BEHAVIOR_CROUCH_MS;
        break;
    case HIP_BEHAVIOR_IMPULSE:
        duration = HIP_BEHAVIOR_IMPULSE_MS;
        break;
    case HIP_BEHAVIOR_FLIGHT:
        duration = HIP_BEHAVIOR_FLIGHT_MS;
        break;
    case HIP_BEHAVIOR_LANDING:
        duration = HIP_BEHAVIOR_LANDING_MS;
        break;
    default:
        duration = 0U;
        break;
    }

    if (duration == 0U) {
        s_behavior.phase_progress = 0U;
    } else if (elapsed >= duration) {
        s_behavior.phase_progress = 100U;
    } else {
        s_behavior.phase_progress = (uint8_t)((elapsed * 100U) / duration);
    }
}
