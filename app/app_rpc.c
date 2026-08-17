#include "app_rpc.h"
#include "app_arm.h"
#include "app_config.h"
#include "app_imu.h"
#include "app_link.h"
#include "app_main.h"
#include "app_motor.h"
#include "hip_control.h"
#include "param_storage.h"
#include "robot_protocol.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "control_timer.h"
#include "motion_control.h"
#include "motor_link.h"

/* Bounds are intentionally broader than the shipping tune, but narrow enough
 * to keep a malformed RPC payload from installing an unsafe configuration. */
#define APP_RPC_WHEEL_RADIUS_MIN_M 0.005f
#define APP_RPC_WHEEL_RADIUS_MAX_M 0.500f
#define APP_RPC_WHEEL_BASE_MIN_M 0.020f
#define APP_RPC_WHEEL_BASE_MAX_M 2.000f
#define APP_RPC_CONTROL_RATE_MIN_HZ 50.0f
#define APP_RPC_CONTROL_RATE_MAX_HZ 2000.0f
#define APP_RPC_LOG_FIELDS_MASK 0x7FU

typedef struct {
  size_t offset;
  size_t length;
} app_rpc_param_field_t;

#define APP_RPC_PARAM_FIELD(field)                                              \
  { offsetof(robot_params_t, field), sizeof(((robot_params_t *)0)->field) }

static bool app_rpc_finite_in_range(float value, float min_value,
                                    float max_value) {
  return isfinite(value) && value >= min_value && value <= max_value;
}

static bool app_rpc_rotation_is_valid(const float rotation[9]) {
  float row_norm_sq[3] = {0.0f, 0.0f, 0.0f};
  for (size_t row = 0U; row < 3U; ++row) {
    for (size_t col = 0U; col < 3U; ++col) {
      float value = rotation[(row * 3U) + col];
      if (!isfinite(value)) {
        return false;
      }
      row_norm_sq[row] += value * value;
    }
    if (fabsf(row_norm_sq[row] - 1.0f) > 0.05f) {
      return false;
    }
  }

  for (size_t first = 0U; first < 3U; ++first) {
    for (size_t second = first + 1U; second < 3U; ++second) {
      float dot = 0.0f;
      for (size_t col = 0U; col < 3U; ++col) {
        dot += rotation[(first * 3U) + col] *
               rotation[(second * 3U) + col];
      }
      if (fabsf(dot) > 0.05f) {
        return false;
      }
    }
  }

  float determinant =
      rotation[0] * ((rotation[4] * rotation[8]) - (rotation[5] * rotation[7])) -
      rotation[1] * ((rotation[3] * rotation[8]) - (rotation[5] * rotation[6])) +
      rotation[2] * ((rotation[3] * rotation[7]) - (rotation[4] * rotation[6]));
  return determinant >= 0.95f && determinant <= 1.05f;
}

static bool app_rpc_mag_matrix_is_valid(const float matrix[9]) {
  for (size_t i = 0U; i < 9U; ++i) {
    if (!app_rpc_finite_in_range(matrix[i], -100.0f, 100.0f)) {
      return false;
    }
  }

  float determinant =
      matrix[0] * ((matrix[4] * matrix[8]) - (matrix[5] * matrix[7])) -
      matrix[1] * ((matrix[3] * matrix[8]) - (matrix[5] * matrix[6])) +
      matrix[2] * ((matrix[3] * matrix[7]) - (matrix[4] * matrix[6]));
  return fabsf(determinant) >= 0.001f && fabsf(determinant) <= 1000.0f;
}

static bool app_rpc_params_are_valid(const robot_params_t *params) {
  if (params == NULL) {
    return false;
  }

  if ((params->motor_direction[0] != -1 && params->motor_direction[0] != 1) ||
      (params->motor_direction[1] != -1 && params->motor_direction[1] != 1) ||
      (params->hip_left_dir_sign != -1 && params->hip_left_dir_sign != 1) ||
      (params->hip_right_dir_sign != -1 && params->hip_right_dir_sign != 1)) {
    return false;
  }

  if (!app_rpc_finite_in_range(params->wheel_radius_m,
                               APP_RPC_WHEEL_RADIUS_MIN_M,
                               APP_RPC_WHEEL_RADIUS_MAX_M) ||
      !app_rpc_finite_in_range(params->wheel_base_m, APP_RPC_WHEEL_BASE_MIN_M,
                               APP_RPC_WHEEL_BASE_MAX_M) ||
      !app_rpc_rotation_is_valid(params->imu_bmi270.rotation) ||
      !app_rpc_rotation_is_valid(params->imu_icm42688.rotation) ||
      !app_rpc_mag_matrix_is_valid(params->mag_bmm150.soft_iron)) {
    return false;
  }

  const balance_gains_t *balance = &params->balance;
  if (!app_rpc_finite_in_range(balance->Kp_theta, -1000.0f, 1000.0f) ||
      !app_rpc_finite_in_range(balance->Kd_theta, -1000.0f, 1000.0f) ||
      !app_rpc_finite_in_range(balance->Kp_v_to_theta, -1000.0f, 1000.0f) ||
      !app_rpc_finite_in_range(balance->Ki_v_to_theta, -1000.0f, 1000.0f) ||
      !app_rpc_finite_in_range(balance->max_tilt_ref, 0.001f, 0.5f) ||
      !app_rpc_finite_in_range(balance->Kv_damp, -1000.0f, 1000.0f) ||
      !app_rpc_finite_in_range(balance->K_turn, -1000.0f, 1000.0f) ||
      !app_rpc_finite_in_range(balance->K_yawDamp, -1000.0f, 1000.0f) ||
      !app_rpc_finite_in_range(balance->alpha_yaw, 0.0f, 1.0f) ||
      !app_rpc_finite_in_range(balance->IqMax, 0.001f, 50.0f) ||
      !app_rpc_finite_in_range(balance->thetaKill, 0.05f, 1.5f) ||
      !app_rpc_finite_in_range(balance->iV_max, 0.0f, 10.0f) ||
      balance->max_tilt_ref >= balance->thetaKill) {
    return false;
  }

  const lqr_params_t *lqr = &params->lqr;
  for (size_t i = 0U; i < 4U; ++i) {
    if (!app_rpc_finite_in_range(lqr->K[i], -10000.0f, 10000.0f)) {
      return false;
    }
  }
  if (!app_rpc_finite_in_range(lqr->u_limit, 0.001f, 50.0f) ||
      !app_rpc_finite_in_range(lqr->du_limit, 0.001f, 10000.0f) ||
      !app_rpc_finite_in_range(lqr->theta_ref_limit, 0.0001f, 0.5f) ||
      !app_rpc_finite_in_range(lqr->v_ref_limit, 0.001f, 10.0f) ||
      lqr->default_mode > 1U) {
    return false;
  }

  if (!app_rpc_finite_in_range(params->max_linear_vel_mps, 0.001f, 10.0f) ||
      !app_rpc_finite_in_range(params->max_angular_vel_rps, 0.001f, 20.0f) ||
      !app_rpc_finite_in_range(params->max_linear_accel_mps2, 0.001f, 50.0f) ||
      !app_rpc_finite_in_range(params->max_angular_accel_rps2, 0.001f,
                               100.0f) ||
      !app_rpc_finite_in_range(params->control_rate_hz,
                               APP_RPC_CONTROL_RATE_MIN_HZ,
                               APP_RPC_CONTROL_RATE_MAX_HZ) ||
      !app_rpc_finite_in_range(params->adc_voltage_multiplier, 0.01f, 100.0f) ||
      !app_rpc_finite_in_range(params->hip_left_zero_offset_rev, -10000.0f,
                               10000.0f) ||
      !app_rpc_finite_in_range(params->hip_right_zero_offset_rev, -10000.0f,
                               10000.0f)) {
    return false;
  }

  if ((params->log_fields_mask & ~APP_RPC_LOG_FIELDS_MASK) != 0U ||
      params->dump_seconds_default > 3600U) {
    return false;
  }

  for (size_t i = 0U; i < sizeof(params->hip_reserved); ++i) {
    if (params->hip_reserved[i] != 0U) {
      return false;
    }
  }
  for (size_t i = 0U; i < sizeof(lqr->reserved); ++i) {
    if (lqr->reserved[i] != 0U) {
      return false;
    }
  }
  for (size_t i = 0U; i < sizeof(params->reserved); ++i) {
    if (params->reserved[i] != 0U) {
      return false;
    }
  }
  return true;
}

static bool app_rpc_write_targets_param_field(size_t offset, size_t length) {
  static const app_rpc_param_field_t writable_fields[] = {
      APP_RPC_PARAM_FIELD(motor_direction),
      APP_RPC_PARAM_FIELD(wheel_radius_m),
      APP_RPC_PARAM_FIELD(wheel_base_m),
      APP_RPC_PARAM_FIELD(imu_bmi270),
      APP_RPC_PARAM_FIELD(imu_icm42688),
      APP_RPC_PARAM_FIELD(mag_bmm150),
      APP_RPC_PARAM_FIELD(balance),
      APP_RPC_PARAM_FIELD(lqr),
      APP_RPC_PARAM_FIELD(max_linear_vel_mps),
      APP_RPC_PARAM_FIELD(max_angular_vel_rps),
      APP_RPC_PARAM_FIELD(max_linear_accel_mps2),
      APP_RPC_PARAM_FIELD(max_angular_accel_rps2),
      APP_RPC_PARAM_FIELD(control_rate_hz),
      APP_RPC_PARAM_FIELD(uart_baudrate),
      APP_RPC_PARAM_FIELD(robot_id),
      APP_RPC_PARAM_FIELD(adc_voltage_multiplier),
      APP_RPC_PARAM_FIELD(log_fields_mask),
      APP_RPC_PARAM_FIELD(dump_seconds_default),
      APP_RPC_PARAM_FIELD(hip_left_zero_offset_rev),
      APP_RPC_PARAM_FIELD(hip_right_zero_offset_rev),
      APP_RPC_PARAM_FIELD(hip_left_dir_sign),
      APP_RPC_PARAM_FIELD(hip_right_dir_sign),
  };

  for (size_t i = 0U; i < sizeof(writable_fields) / sizeof(writable_fields[0]);
       ++i) {
    size_t field_start = writable_fields[i].offset;
    size_t field_end = field_start + writable_fields[i].length;
    if (offset >= field_start && offset + length <= field_end) {
      return true;
    }
  }
  return false;
}

static bool app_rpc_params_are_mutable(void) {
  return motion_control_get_mode() != MOTION_MODE_BALANCING &&
         s_motor_manual.enabled == 0U;
}

static bool app_rpc_send_param_resp(uint8_t method, uint8_t status,
                                    uint16_t offset, uint16_t length,
                                    const uint8_t *data, uint16_t data_len,
                                    uint16_t seq_override) {
  robot_rpc_param_t resp_hdr;
  uint8_t payload[ROBOT_FRAME_MAX_PAYLOAD];
  size_t total_len = sizeof(resp_hdr) + (size_t)data_len;

  if (total_len > sizeof(payload)) {
    return false;
  }

  resp_hdr.method = method;
  resp_hdr.flags = status;
  resp_hdr.offset = offset;
  resp_hdr.length = length;
  memcpy(payload, &resp_hdr, sizeof(resp_hdr));
  if (data_len > 0U && data != NULL) {
    memcpy(payload + sizeof(resp_hdr), data, data_len);
  }

  return app_link_send(ROBOT_MSG_RPC_RESP, ROBOT_FLAG_ACK_REQ, payload,
                       (uint16_t)total_len, seq_override);
}

void app_rpc_handle(const robot_frame_t *frame) {
  if (frame == NULL) {
    return;
  }
  if (frame->hdr.len < sizeof(robot_rpc_param_t)) {
    APP_LOG_ERROR("RPC payload too short (%u)", (unsigned int)frame->hdr.len);
    return;
  }

  robot_rpc_param_t req;
  memcpy(&req, frame->payload, sizeof(req));

  const size_t params_size = sizeof(robot_params_t);
  const size_t req_data_len = frame->hdr.len - sizeof(req);
  const uint8_t *req_data = frame->payload + sizeof(req);
  const uint16_t offset = req.offset;
  const uint16_t length = req.length;
  const uint16_t resp_seq = (frame->hdr.seq != 0U) ? frame->hdr.seq : 0U;

  if ((size_t)offset > params_size) {
    app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_OFFSET, offset,
                            length, NULL, 0U, resp_seq);
    return;
  }
  if ((size_t)offset + (size_t)length > params_size) {
    app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                            length, NULL, 0U, resp_seq);
    return;
  }

  switch (req.method) {
  case ROBOT_RPC_METHOD_GET_PARAM: {
    if (req_data_len != 0U || length == 0U) {
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                              length, NULL, 0U, resp_seq);
      return;
    }
    if ((size_t)length >
        (ROBOT_FRAME_MAX_PAYLOAD - sizeof(robot_rpc_param_t))) {
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                              length, NULL, 0U, resp_seq);
      return;
    }
    const uint8_t *src = (const uint8_t *)&g_robot_params + offset;
    app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_OK, offset, length,
                            src, length, resp_seq);
    return;
  }
  case ROBOT_RPC_METHOD_IMU_CALIB_FACE: {
    if (req_data_len != sizeof(robot_rpc_imu_calib_face_t)) {
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                              length, NULL, 0U, resp_seq);
      return;
    }
    robot_rpc_imu_calib_face_t calib_req;
    memcpy(&calib_req, req_data, sizeof(calib_req));

    uint8_t status = app_imu_calib_capture_face(calib_req.imu, calib_req.face,
                                                calib_req.samples);
    app_rpc_send_param_resp(req.method, status, calib_req.face, 0U, NULL, 0U,
                            resp_seq);
    return;
  }
  case ROBOT_RPC_METHOD_MOTOR_ENABLE: {
    if (req_data_len != 0U) {
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                              length, NULL, 0U, resp_seq);
      return;
    }
    uint8_t status = app_motor_manual_enable(true);
    app_rpc_send_param_resp(req.method, status, 0U, 0U, NULL, 0U, resp_seq);
    return;
  }
  case ROBOT_RPC_METHOD_MOTOR_DISABLE: {
    if (req_data_len != 0U) {
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                              length, NULL, 0U, resp_seq);
      return;
    }
    uint8_t status = app_motor_manual_enable(false);
    app_rpc_send_param_resp(req.method, status, 0U, 0U, NULL, 0U, resp_seq);
    return;
  }
  case ROBOT_RPC_METHOD_MOTOR_RUN: {
    if (req_data_len != sizeof(robot_rpc_motor_run_t)) {
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                              length, NULL, 0U, resp_seq);
      return;
    }
    robot_rpc_motor_run_t run_req;
    memcpy(&run_req, req_data, sizeof(run_req));
    uint8_t status = app_motor_manual_run(run_req.side, run_req.intensity);
    app_rpc_send_param_resp(req.method, status, run_req.side, 0U, NULL, 0U,
                            resp_seq);
    return;
  }
  case ROBOT_RPC_METHOD_BALANCE_ENABLE: {
    if (req_data_len != 0U) {
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                              length, NULL, 0U, resp_seq);
      return;
    }
    if (!app_try_arm_balancing(true)) {
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_NOT_READY, 0U, 0U,
                              NULL, 0U, resp_seq);
      return;
    }
    app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_OK, 0U, 0U, NULL, 0U,
                            resp_seq);
    return;
  }
  case ROBOT_RPC_METHOD_BALANCE_DISABLE: {
    if (req_data_len != 0U) {
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                              length, NULL, 0U, resp_seq);
      return;
    }
    app_disarm_robot();
    app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_OK, 0U, 0U, NULL, 0U,
                            resp_seq);
    return;
  }
  case ROBOT_RPC_METHOD_HIP_CALIB_ZERO: {
    if (req_data_len != 0U) {
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                              length, NULL, 0U, resp_seq);
      return;
    }
    hip_state_t left = {0};
    hip_state_t right = {0};
    hip_control_get_state(&left, &right);
    if (!left.valid || !right.valid) {
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_NOT_READY, 0U, 0U,
                              NULL, 0U, resp_seq);
      return;
    }
    g_robot_params.hip_left_zero_offset_rev = left.pos_rev;
    g_robot_params.hip_right_zero_offset_rev = right.pos_rev;
    motion_control_apply_params();
    uint8_t status = ROBOT_RPC_STATUS_OK;
    if ((req.flags & ROBOT_RPC_FLAG_SAVE) != 0U) {
      if (!param_storage_can_save()) {
        status = ROBOT_RPC_STATUS_NOT_READY;
      } else {
        int rc = param_storage_save(&g_robot_params);
        if (rc != PARAM_OK) {
          status = ROBOT_RPC_STATUS_STORAGE;
        }
      }
    }
    app_rpc_send_param_resp(req.method, status, 0U, 0U, NULL, 0U, resp_seq);
    return;
  }
  case ROBOT_RPC_METHOD_CAN_SIMPLE_SET_NODE_ID: {
    if (req_data_len != sizeof(robot_rpc_can_simple_node_id_t)) {
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                              length, NULL, 0U, resp_seq);
      return;
    }
    robot_rpc_can_simple_node_id_t prog_req;
    memcpy(&prog_req, req_data, sizeof(prog_req));
    if (prog_req.current_node_id > ROBOT_CAN_SIMPLE_NODE_ID_MAX ||
        prog_req.new_node_id > ROBOT_CAN_SIMPLE_NODE_ID_MAX) {
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_PARAM,
                              prog_req.current_node_id, prog_req.new_node_id,
                              NULL, 0U, resp_seq);
      return;
    }

    bool save = ((prog_req.flags & ROBOT_CAN_SIMPLE_PROG_FLAG_SAVE) != 0U) ||
                ((req.flags & ROBOT_RPC_FLAG_SAVE) != 0U);
    uint8_t status =
        hip_control_program_node_id(prog_req.current_node_id,
                                    prog_req.new_node_id, save)
            ? ROBOT_RPC_STATUS_OK
            : ROBOT_RPC_STATUS_NOT_READY;
    app_rpc_send_param_resp(req.method, status, prog_req.current_node_id,
                            prog_req.new_node_id, NULL, 0U, resp_seq);
    return;
  }
  case ROBOT_RPC_METHOD_SET_PARAM: {
    float old_rate_hz = g_robot_params.control_rate_hz;
    if (length == 0U) {
      if (req_data_len != 0U || (req.flags & ROBOT_RPC_FLAG_SAVE) == 0U) {
        app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                                length, NULL, 0U, resp_seq);
        return;
      }
      if (!app_rpc_params_are_mutable()) {
        app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_NOT_READY, offset,
                                length, NULL, 0U, resp_seq);
        return;
      }
      if (!app_rpc_params_are_valid(&g_robot_params)) {
        app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_PARAM, offset,
                                length, NULL, 0U, resp_seq);
        return;
      }
      uint8_t status = ROBOT_RPC_STATUS_OK;
      int rc = param_storage_save(&g_robot_params);
      if (rc != PARAM_OK) {
        status = ROBOT_RPC_STATUS_STORAGE;
      }
      app_rpc_send_param_resp(req.method, status, offset, length, NULL, 0U,
                              resp_seq);
      return;
    }
    if (req_data_len != length) {
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
                              length, NULL, 0U, resp_seq);
      return;
    }
    if (!app_rpc_write_targets_param_field(offset, length)) {
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_PARAM, offset,
                              length, NULL, 0U, resp_seq);
      return;
    }
    if (!app_rpc_params_are_mutable()) {
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_NOT_READY, offset,
                              length, NULL, 0U, resp_seq);
      return;
    }

    robot_params_t updated;
    memcpy(&updated, &g_robot_params, sizeof(updated));
    memcpy((uint8_t *)&updated + offset, req_data, length);
    if (!app_rpc_params_are_valid(&updated)) {
      app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_PARAM, offset,
                              length, NULL, 0U, resp_seq);
      return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool changed = (memcmp(&updated, &g_robot_params, sizeof(updated)) != 0);
    if (changed) {
      memcpy(&g_robot_params, &updated, sizeof(updated));
    }
    __set_PRIMASK(primask);
    if (changed) {
      motion_control_apply_params();
      if (fabsf(g_robot_params.control_rate_hz - old_rate_hz) > 1e-3f) {
        control_timer_set_rate_hz(g_robot_params.control_rate_hz);
      }
    }

    uint8_t status = ROBOT_RPC_STATUS_OK;
    if ((req.flags & ROBOT_RPC_FLAG_SAVE) != 0U && changed) {
      int rc = param_storage_save(&g_robot_params);
      if (rc != PARAM_OK) {
        status = ROBOT_RPC_STATUS_STORAGE;
      }
    }

    app_rpc_send_param_resp(req.method, status, offset, length, NULL, 0U,
                            resp_seq);
    return;
  }
  default:
    app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_METHOD, offset,
                            length, NULL, 0U, resp_seq);
    return;
  }
}
