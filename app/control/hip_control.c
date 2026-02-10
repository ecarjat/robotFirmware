#include "hip_control.h"

#include <math.h>
#include <string.h>

#include "app_log_macros.h"
#include "app_config.h"
#include "hip_kinematics.h"
#include "hip_limits.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"

#ifndef HIP_CAN_NODE_LEFT
#define HIP_CAN_NODE_LEFT 0x03U
#endif
#ifndef HIP_CAN_NODE_RIGHT
#define HIP_CAN_NODE_RIGHT 0x04U
#endif

#ifndef HIP_CMD_PERIOD_MS
#define HIP_CMD_PERIOD_MS 10U
#endif
#ifndef HIP_TELEM_PERIOD_MS
#define HIP_TELEM_PERIOD_MS 20U
#endif
#ifndef HIP_HEARTBEAT_PERIOD_MS
#define HIP_HEARTBEAT_PERIOD_MS 200U
#endif
#ifndef HIP_TELEM_TIMEOUT_MS
#define HIP_TELEM_TIMEOUT_MS 200U
#endif
#ifndef HIP_LIMIT_DEBOUNCE_SAMPLES
#define HIP_LIMIT_DEBOUNCE_SAMPLES 4U
#endif

/* CAN protocol: standard ID = (node_id << 5) | cmd_id.
 * Encoder telemetry: 2x float32 {pos_rev, vel_rev_s}
 * Torque telemetry: 2x float32 {torque_setpoint_nm, torque_measured_nm}
 * Set input pos: float32 pos_rev + int16 vel_ff (rev/s*1000) + int16 torque_ff (Nm*1000)
 */
#define HIP_CMD_SET_AXIS_STATE 0x007U
#define HIP_CMD_SET_CTRL_MODE  0x00BU
#define HIP_CMD_SET_INPUT_POS  0x00CU
#define HIP_CMD_GET_ENCODER    0x009U
#define HIP_CMD_GET_TORQUES    0x01CU
#define HIP_CMD_HEARTBEAT      0x001U

#define HIP_CTRL_MODE_POSITION 3U
#define HIP_INPUT_MODE_POS_FILTER 3U
#define HIP_AXIS_STATE_CLOSED_LOOP 8U

#define HIP_PI 3.14159265358979323846f
#define HIP_EPS 1.0e-6f

extern FDCAN_HandleTypeDef hfdcan1;

typedef struct {
    uint8_t node_id;
    int8_t dir_sign;
    float zero_offset_rev;
    float pos_rev;
    float vel_rev_s;
    float last_pos_rev;
    float torque_setpoint_nm;
    float torque_nm;
    float last_height_m;
    float height_m;
    float height_dot_m_s;
    uint32_t last_encoder_ms;
    uint32_t last_torque_ms;
    uint32_t last_heartbeat_ms;
    uint32_t last_move_ms;
    uint8_t encoder_valid;
    uint8_t torque_valid;
    uint8_t heartbeat_valid;
    uint8_t limit_upper;
    uint8_t limit_lower;
    uint8_t recovery_mode;
    float recovery_theta_start;
    uint8_t recovery_start_valid;
    uint32_t recovery_start_ms;
    uint8_t recovery_blocked;
} hip_internal_state_t;

static hip_internal_state_t s_hips[2];
static hip_state_t s_state_pub[2];
static hip_command_t s_cmd_pub[2];
static hip_target_t s_target = {
    .height_ref_m = NAN,
    .height_rate_ref_m_s = 0.0f,
    .stiffness_n_m = HIP_STIFFNESS_N_M_DEFAULT,
    .damping_n_s_m = HIP_DAMPING_N_S_M_DEFAULT,
    .mode = HIP_MODE_HOLD,
    .enabled = true,
};
static uint32_t s_faults = HIP_FAULT_NONE;
static uint32_t s_last_target_ms = 0U;
static uint32_t s_last_cmd_ms = 0U;
static uint32_t s_last_telem_ms = 0U;
static uint32_t s_last_heartbeat_ms = 0U;
static uint8_t s_telem_phase = 0U;
static uint8_t s_inited = 0U;
static uint8_t s_bus_off = 0U;
static uint8_t s_startup_step = 0U;
static uint32_t s_startup_next_ms = 0U;

static hip_limit_debounce_t s_left_upper_sw;
static hip_limit_debounce_t s_left_lower_sw;
static hip_limit_debounce_t s_right_upper_sw;
static hip_limit_debounce_t s_right_lower_sw;

static float s_height_ref_m = NAN;

static float hip_rad_to_rev(float rad) {
    return rad / (2.0f * HIP_PI);
}

static float hip_rev_to_rad(float rev) {
    return rev * (2.0f * HIP_PI);
}

static float hip_clampf(float value, float min_v, float max_v) {
    if (value < min_v) {
        return min_v;
    }
    if (value > max_v) {
        return max_v;
    }
    return value;
}

static uint32_t hip_make_id(uint8_t node_id, uint8_t cmd_id) {
    return ((uint32_t)node_id << 5) | (uint32_t)(cmd_id & 0x1FU);
}

static void hip_pack_f32(uint8_t *dst, float value) {
    memcpy(dst, &value, sizeof(value));
}

static float hip_unpack_f32(const uint8_t *src) {
    float value = 0.0f;
    memcpy(&value, src, sizeof(value));
    return value;
}

static uint32_t hip_len_to_dlc(uint8_t len) {
    switch (len) {
    case 0: return FDCAN_DLC_BYTES_0;
    case 1: return FDCAN_DLC_BYTES_1;
    case 2: return FDCAN_DLC_BYTES_2;
    case 3: return FDCAN_DLC_BYTES_3;
    case 4: return FDCAN_DLC_BYTES_4;
    case 5: return FDCAN_DLC_BYTES_5;
    case 6: return FDCAN_DLC_BYTES_6;
    case 7: return FDCAN_DLC_BYTES_7;
    default: return FDCAN_DLC_BYTES_8;
    }
}

static bool hip_send(uint8_t node_id, uint8_t cmd_id,
                     const uint8_t *payload, uint8_t len) {
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier = hip_make_id(node_id, cmd_id);
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = hip_len_to_dlc(len);
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0U;

    uint8_t data[8] = {0};
    if (payload != NULL && len > 0U) {
        if (len > 8U) {
            len = 8U;
        }
        memcpy(data, payload, len);
    }

    return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, data) == HAL_OK);
}

static void hip_set_axis_state(uint8_t node_id, uint8_t state) {
    uint8_t data[8] = {0};
    data[0] = state;
    (void)hip_send(node_id, HIP_CMD_SET_AXIS_STATE, data, 1U);
}

static void hip_set_ctrl_mode(uint8_t node_id, uint8_t control_mode, uint8_t input_mode) {
    uint8_t data[8] = {0};
    data[0] = control_mode;
    data[4] = input_mode;
    (void)hip_send(node_id, HIP_CMD_SET_CTRL_MODE, data, 8U);
}

static void hip_set_input_pos(uint8_t node_id, float pos_rev, float vel_ff_rev_s,
                              float torque_ff_nm) {
    uint8_t data[8] = {0};
    hip_pack_f32(&data[0], pos_rev);
    int16_t vel_ff = (int16_t)lrintf(vel_ff_rev_s * 1000.0f);
    int16_t torque_ff = (int16_t)lrintf(torque_ff_nm * 1000.0f);
    memcpy(&data[4], &vel_ff, sizeof(vel_ff));
    memcpy(&data[6], &torque_ff, sizeof(torque_ff));
    (void)hip_send(node_id, HIP_CMD_SET_INPUT_POS, data, 8U);
}

static uint8_t hip_read_limit(GPIO_TypeDef *port, uint16_t pin) {
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static void hip_update_limits(uint32_t now_ms) {
    uint8_t left_upper = hip_read_limit(LeftHipUpperLimit_GPIO_Port, LeftHipUpperLimit_Pin);
    uint8_t left_lower = hip_read_limit(LeftHipLowerLimit_GPIO_Port, LeftHipLowerLimit_Pin);
    uint8_t right_upper = hip_read_limit(RightHipUpperLimit_GPIO_Port, RightHipUpperLimit_Pin);
    uint8_t right_lower = hip_read_limit(RightHipLowerLimit_GPIO_Port, RightHipLowerLimit_Pin);

    s_hips[0].limit_upper = hip_limit_update(&s_left_upper_sw, left_upper);
    s_hips[0].limit_lower = hip_limit_update(&s_left_lower_sw, left_lower);
    s_hips[1].limit_upper = hip_limit_update(&s_right_upper_sw, right_upper);
    s_hips[1].limit_lower = hip_limit_update(&s_right_lower_sw, right_lower);

    for (int i = 0; i < 2; ++i) {
        if (!s_hips[i].recovery_mode && !s_hips[i].recovery_blocked &&
            (s_hips[i].limit_upper || s_hips[i].limit_lower)) {
            s_hips[i].recovery_mode = 1U;
            s_hips[i].recovery_start_valid = 0U;
            s_hips[i].recovery_start_ms = now_ms;
        }
        if (s_hips[i].recovery_mode &&
            !s_hips[i].limit_upper && !s_hips[i].limit_lower) {
            s_hips[i].recovery_mode = 0U;
            s_hips[i].recovery_start_valid = 0U;
            s_hips[i].recovery_start_ms = 0U;
            s_hips[i].recovery_blocked = 0U;
        }
        if (s_hips[i].recovery_mode && s_hips[i].recovery_start_ms != 0U &&
            (now_ms - s_hips[i].recovery_start_ms) > HIP_RECOVERY_TIMEOUT_MS) {
            s_hips[i].recovery_mode = 0U;
            s_hips[i].recovery_start_valid = 0U;
            s_hips[i].recovery_start_ms = 0U;
            s_hips[i].recovery_blocked = 1U;
        }
    }
}

static float hip_compute_height_dot(const hip_internal_state_t *hip, float now_height_m, float dt_s) {
    if (dt_s <= 0.0f) {
        return 0.0f;
    }
    return (now_height_m - hip->last_height_m) / dt_s;
}

static float hip_compute_jacobian(float theta_rad) {
    const float eps = 1.0e-4f;
    float h_plus = 0.0f;
    float h_minus = 0.0f;
    if (!hip_kinematics_height_from_theta(theta_rad + eps, &h_plus) ||
        !hip_kinematics_height_from_theta(theta_rad - eps, &h_minus)) {
        return 0.0f;
    }
    return (h_plus - h_minus) / (2.0f * eps);
}

static void hip_update_state_from_encoder(hip_internal_state_t *hip, float pos_rev, float vel_rev_s, uint32_t now_ms) {
    uint32_t prev_ms = hip->last_encoder_ms;
    if (fabsf(pos_rev - hip->last_pos_rev) > 1e-4f) {
        hip->last_move_ms = now_ms;
        hip->last_pos_rev = pos_rev;
    }
    hip->pos_rev = pos_rev;
    hip->vel_rev_s = vel_rev_s;
    hip->last_encoder_ms = now_ms;
    hip->encoder_valid = 1U;

    float theta_rad = hip_rev_to_rad((pos_rev - hip->zero_offset_rev) * (float)hip->dir_sign);
    float height_m = 0.0f;
    if (hip_kinematics_height_from_theta(theta_rad, &height_m)) {
        float dt_s = 0.0f;
        if (prev_ms != 0U && now_ms > prev_ms) {
            dt_s = 0.001f * (float)(now_ms - prev_ms);
        }
        hip->height_dot_m_s = hip_compute_height_dot(hip, height_m, dt_s);
        hip->last_height_m = hip->height_m;
        hip->height_m = height_m;
    }

    if (hip->recovery_mode && !hip->recovery_start_valid) {
        hip->recovery_theta_start = theta_rad;
        hip->recovery_start_valid = 1U;
    }
}

static void hip_publish_state(void) {
    for (int i = 0; i < 2; ++i) {
        hip_internal_state_t *src = &s_hips[i];
        hip_state_t *dst = &s_state_pub[i];
        dst->theta_rad = hip_rev_to_rad((src->pos_rev - src->zero_offset_rev) * (float)src->dir_sign);
        dst->theta_dot_rad_s = hip_rev_to_rad(src->vel_rev_s) * (float)src->dir_sign;
        dst->pos_rev = src->pos_rev;
        dst->vel_rev_s = src->vel_rev_s;
        dst->height_m = src->height_m;
        dst->height_dot_m_s = src->height_dot_m_s;
        dst->torque_nm = src->torque_nm;
        dst->torque_setpoint_nm = src->torque_setpoint_nm;
        dst->valid = src->encoder_valid;
        dst->last_rx_ms = src->last_encoder_ms;
        dst->limit_upper = src->limit_upper;
        dst->limit_lower = src->limit_lower;
    }
}

static void hip_update_faults(uint32_t now_ms) {
    uint32_t faults = s_faults;

    for (int i = 0; i < 2; ++i) {
        hip_internal_state_t *hip = &s_hips[i];
        if (hip->heartbeat_valid &&
            (now_ms - hip->last_heartbeat_ms) > HIP_TELEM_TIMEOUT_MS) {
            faults |= HIP_FAULT_HEARTBEAT_TIMEOUT;
            hip->heartbeat_valid = 0U;
        }
        if (hip->last_encoder_ms != 0U &&
            (now_ms - hip->last_encoder_ms) > HIP_TELEM_TIMEOUT_MS) {
            faults |= HIP_FAULT_ENCODER_TIMEOUT;
            hip->encoder_valid = 0U;
        }
        if (hip->last_torque_ms != 0U &&
            (now_ms - hip->last_torque_ms) > HIP_TELEM_TIMEOUT_MS) {
            faults |= HIP_FAULT_TORQUE_TIMEOUT;
            hip->torque_valid = 0U;
        }
        if (hip->encoder_valid) {
            uint32_t stall_bit = (i == 0) ? HIP_FAULT_STALL_LEFT : HIP_FAULT_STALL_RIGHT;
            float cmd_pos = s_cmd_pub[i].pos_cmd_rev;
            if (fabsf(cmd_pos - hip->pos_rev) > HIP_STALL_ERR_REV &&
                hip->last_move_ms != 0U &&
                (now_ms - hip->last_move_ms) > HIP_STALL_TIMEOUT_MS) {
                faults |= stall_bit;
            } else if (fabsf(cmd_pos - hip->pos_rev) <= HIP_STALL_ERR_REV) {
                faults &= ~stall_bit;
            }
        }
    }

    s_faults = faults;
}

static void hip_update_height_ref(uint32_t now_ms) {
    float min_h = 0.0f;
    float max_h = 0.0f;
    bool have_range = hip_kinematics_get_height_range(&min_h, &max_h);

    if (!isnan(s_target.height_ref_m)) {
        if (isnan(s_height_ref_m)) {
            s_height_ref_m = s_target.height_ref_m;
        }
        float dt_s = 0.0f;
        if (s_last_target_ms != 0U && now_ms > s_last_target_ms) {
            dt_s = 0.001f * (float)(now_ms - s_last_target_ms);
        }
        s_last_target_ms = now_ms;
        if (dt_s > 0.0f) {
            float max_step = HIP_HEIGHT_SLEW_MPS * dt_s;
            float delta = s_target.height_ref_m - s_height_ref_m;
            if (fabsf(delta) > max_step) {
                s_height_ref_m += (delta > 0.0f) ? max_step : -max_step;
            } else {
                s_height_ref_m = s_target.height_ref_m;
            }
        }
    }

    if (have_range && !isnan(s_height_ref_m)) {
        s_height_ref_m = hip_clampf(s_height_ref_m, min_h, max_h);
    }
}

static void hip_send_commands(uint32_t now_ms) {
    if (!s_inited || s_bus_off) {
        return;
    }

    if (!s_target.enabled) {
        return;
    }

    if (s_faults & (HIP_FAULT_HEARTBEAT_TIMEOUT | HIP_FAULT_ENCODER_TIMEOUT)) {
        return;
    }

    hip_update_height_ref(now_ms);

    if (isnan(s_height_ref_m)) {
        // Hold current height if no target specified.
        for (int i = 0; i < 2; ++i) {
            if (s_hips[i].encoder_valid) {
                s_height_ref_m = s_hips[i].height_m;
                break;
            }
        }
    }

    for (int i = 0; i < 2; ++i) {
        hip_internal_state_t *hip = &s_hips[i];
        if (!hip->encoder_valid) {
            continue;
        }

        float target_height = s_height_ref_m;

        if (hip->recovery_mode) {
            if (hip->limit_upper && !hip->limit_lower) {
                target_height = hip->height_m - 0.005f;
            } else if (hip->limit_lower && !hip->limit_upper) {
                target_height = hip->height_m + 0.005f;
            }
            s_height_ref_m = target_height;

            float theta_now = hip_rev_to_rad((hip->pos_rev - hip->zero_offset_rev) * (float)hip->dir_sign);
            float theta_range = (HIP_THETA_MAX_DEG - HIP_THETA_MIN_DEG) * (HIP_PI / 180.0f);
            if (hip->recovery_start_valid &&
                fabsf(theta_now - hip->recovery_theta_start) > theta_range) {
                target_height = hip->height_m;
                s_height_ref_m = target_height;
            }
        }

        if (hip->limit_upper && target_height > hip->height_m) {
            target_height = hip->height_m;
        }
        if (hip->limit_lower && target_height < hip->height_m) {
            target_height = hip->height_m;
        }

        float theta_target_rad = 0.0f;
        if (!hip_kinematics_theta_from_height(target_height, &theta_target_rad)) {
            s_faults |= HIP_FAULT_KINEMATICS;
            s_height_ref_m = hip->height_m;
            continue;
        }

        float pos_cmd_rev = hip_rad_to_rev(theta_target_rad) * (float)hip->dir_sign + hip->zero_offset_rev;

        float height_error = target_height - hip->height_m;
        float height_rate_error = s_target.height_rate_ref_m_s - hip->height_dot_m_s;
        float force_cmd = s_target.stiffness_n_m * height_error + s_target.damping_n_s_m * height_rate_error;
        float jacobian = hip_compute_jacobian(theta_target_rad);
        float torque_ff = force_cmd * jacobian;
        float vel_ff_rev_s = 0.0f;
        if (fabsf(jacobian) > HIP_EPS) {
            float theta_dot = height_rate_error / jacobian;
            vel_ff_rev_s = hip_rad_to_rev(theta_dot) * (float)hip->dir_sign;
        }

        vel_ff_rev_s = hip_clampf(vel_ff_rev_s, -HIP_VEL_MAX_REV_S, HIP_VEL_MAX_REV_S);
        torque_ff = hip_clampf(torque_ff, -HIP_TORQUE_MAX_NM, HIP_TORQUE_MAX_NM);

        s_cmd_pub[i].pos_cmd_rev = pos_cmd_rev;
        s_cmd_pub[i].vel_ff_rev_s = vel_ff_rev_s;
        s_cmd_pub[i].torque_ff_nm = torque_ff;

        hip_set_input_pos(hip->node_id, pos_cmd_rev, vel_ff_rev_s, torque_ff);
    }
}

void hip_control_init(void) {
    s_hips[0] = (hip_internal_state_t){0};
    s_hips[1] = (hip_internal_state_t){0};
    s_hips[0].node_id = HIP_CAN_NODE_LEFT;
    s_hips[1].node_id = HIP_CAN_NODE_RIGHT;
    s_hips[0].dir_sign = 1;
    s_hips[1].dir_sign = 1;

    hip_limit_init(&s_left_upper_sw, HIP_LIMIT_DEBOUNCE_SAMPLES, 0U);
    hip_limit_init(&s_left_lower_sw, HIP_LIMIT_DEBOUNCE_SAMPLES, 0U);
    hip_limit_init(&s_right_upper_sw, HIP_LIMIT_DEBOUNCE_SAMPLES, 0U);
    hip_limit_init(&s_right_lower_sw, HIP_LIMIT_DEBOUNCE_SAMPLES, 0U);

    hip_update_limits(0U);

    s_last_cmd_ms = 0U;
    s_last_telem_ms = 0U;
    s_last_heartbeat_ms = 0U;
    s_telem_phase = 0U;
    s_bus_off = 0U;
    s_startup_step = 0U;
    s_startup_next_ms = 0U;
    s_faults = HIP_FAULT_NONE;
    s_height_ref_m = NAN;
    s_last_target_ms = 0U;
    s_target.height_ref_m = NAN;
    s_target.height_rate_ref_m_s = 0.0f;
    s_target.stiffness_n_m = HIP_STIFFNESS_N_M_DEFAULT;
    s_target.damping_n_s_m = HIP_DAMPING_N_S_M_DEFAULT;
    s_target.mode = HIP_MODE_HOLD;
    s_target.enabled = true;
    s_cmd_pub[0] = (hip_command_t){0};
    s_cmd_pub[1] = (hip_command_t){0};
    s_state_pub[0] = (hip_state_t){0};
    s_state_pub[1] = (hip_state_t){0};
    s_inited = 1U;
}

void hip_control_apply_params(const robot_params_t *params) {
    if (params == NULL) {
        return;
    }
    s_hips[0].zero_offset_rev = params->hip_left_zero_offset_rev;
    s_hips[1].zero_offset_rev = params->hip_right_zero_offset_rev;
    s_hips[0].dir_sign = (params->hip_left_dir_sign >= 0) ? 1 : -1;
    s_hips[1].dir_sign = (params->hip_right_dir_sign >= 0) ? 1 : -1;
}

void hip_control_set_target(const hip_target_t *target) {
    if (target == NULL) {
        return;
    }
    s_target = *target;
}

void hip_control_get_target(hip_target_t *target) {
    if (target != NULL) {
        *target = s_target;
    }
}

void hip_control_get_state(hip_state_t *left, hip_state_t *right) {
    if (left != NULL) {
        *left = s_state_pub[0];
    }
    if (right != NULL) {
        *right = s_state_pub[1];
    }
}

void hip_control_get_command(hip_command_t *left, hip_command_t *right) {
    if (left != NULL) {
        *left = s_cmd_pub[0];
    }
    if (right != NULL) {
        *right = s_cmd_pub[1];
    }
}

uint32_t hip_control_get_faults(void) {
    return s_faults;
}

bool hip_control_on_can_rx(uint32_t id, const uint8_t *data, uint8_t len, uint32_t t_ms) {
    uint8_t node_id = (uint8_t)((id >> 5) & 0x3FU);
    uint8_t cmd_id = (uint8_t)(id & 0x1FU);
    hip_internal_state_t *hip = NULL;

    if (node_id == s_hips[0].node_id) {
        hip = &s_hips[0];
    } else if (node_id == s_hips[1].node_id) {
        hip = &s_hips[1];
    } else {
        return false;
    }

    if (cmd_id == HIP_CMD_HEARTBEAT) {
        hip->last_heartbeat_ms = t_ms;
        hip->heartbeat_valid = 1U;
        s_faults &= ~(uint32_t)HIP_FAULT_HEARTBEAT_TIMEOUT;
        return true;
    }

    if (data == NULL || len < 4U) {
        return true;
    }

    if (cmd_id == HIP_CMD_GET_ENCODER && len >= 8U) {
        float pos = hip_unpack_f32(&data[0]);
        float vel = hip_unpack_f32(&data[4]);
        hip_update_state_from_encoder(hip, pos, vel, t_ms);
        s_faults &= ~(uint32_t)HIP_FAULT_ENCODER_TIMEOUT;
        return true;
    }
    if (cmd_id == HIP_CMD_GET_TORQUES && len >= 8U) {
        hip->torque_setpoint_nm = hip_unpack_f32(&data[0]);
        hip->torque_nm = hip_unpack_f32(&data[4]);
        hip->last_torque_ms = t_ms;
        hip->torque_valid = 1U;
        s_faults &= ~(uint32_t)HIP_FAULT_TORQUE_TIMEOUT;
        return true;
    }
    return false;
}

void hip_control_on_bus_off(void) {
    s_bus_off = 1U;
    s_faults |= HIP_FAULT_BUS_OFF;
    s_target.enabled = false;
#if HIP_HAS_ENABLE_PIN
    HAL_GPIO_WritePin(HIP_ENABLE_GPIO_Port, HIP_ENABLE_Pin, GPIO_PIN_RESET);
#endif
}

void hip_control_on_bus_recovered(void) {
    s_bus_off = 0U;
    s_faults &= ~(uint32_t)HIP_FAULT_BUS_OFF;
    s_startup_step = 0U;
    s_startup_next_ms = 0U;
    s_target.enabled = true;
#if HIP_HAS_ENABLE_PIN
    HAL_GPIO_WritePin(HIP_ENABLE_GPIO_Port, HIP_ENABLE_Pin, GPIO_PIN_SET);
#endif
}

static void hip_control_startup_step(uint32_t now_ms) {
    if (s_startup_step == 0U) {
        hip_set_ctrl_mode(s_hips[0].node_id, HIP_CTRL_MODE_POSITION, HIP_INPUT_MODE_POS_FILTER);
        hip_set_ctrl_mode(s_hips[1].node_id, HIP_CTRL_MODE_POSITION, HIP_INPUT_MODE_POS_FILTER);
        s_startup_step = 1U;
        s_startup_next_ms = now_ms + 10U;
        return;
    }
    if (s_startup_step == 1U && now_ms >= s_startup_next_ms) {
        hip_set_axis_state(s_hips[0].node_id, HIP_AXIS_STATE_CLOSED_LOOP);
        hip_set_axis_state(s_hips[1].node_id, HIP_AXIS_STATE_CLOSED_LOOP);
        s_startup_step = 2U;
        s_startup_next_ms = now_ms;
    }
}

void hip_control_tick(uint32_t now_ms) {
    if (!s_inited) {
        return;
    }

    hip_update_limits(now_ms);

    if (s_startup_step < 2U) {
        if (now_ms >= s_startup_next_ms) {
            hip_control_startup_step(now_ms);
        }
        return;
    }

    if ((now_ms - s_last_cmd_ms) >= HIP_CMD_PERIOD_MS) {
        s_last_cmd_ms = now_ms;
        hip_send_commands(now_ms);
    }

    if ((now_ms - s_last_telem_ms) >= HIP_TELEM_PERIOD_MS) {
        s_last_telem_ms = now_ms;
        uint8_t cmd = (s_telem_phase == 0U) ? HIP_CMD_GET_ENCODER : HIP_CMD_GET_TORQUES;
        s_telem_phase ^= 1U;
        (void)hip_send(s_hips[0].node_id, cmd, NULL, 0U);
        (void)hip_send(s_hips[1].node_id, cmd, NULL, 0U);
    }

    if ((now_ms - s_last_heartbeat_ms) >= HIP_HEARTBEAT_PERIOD_MS) {
        s_last_heartbeat_ms = now_ms;
        (void)hip_send(s_hips[0].node_id, HIP_CMD_HEARTBEAT, NULL, 0U);
        (void)hip_send(s_hips[1].node_id, HIP_CMD_HEARTBEAT, NULL, 0U);
    }

    hip_update_faults(now_ms);
    hip_publish_state();
}
