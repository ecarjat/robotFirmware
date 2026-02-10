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
#include <string.h>

#include "control_timer.h"
#include "motion_control.h"
#include "motor_link.h"

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
  case ROBOT_RPC_METHOD_SET_PARAM: {
    float old_rate_hz = g_robot_params.control_rate_hz;
    if (length == 0U) {
      if (req_data_len != 0U || (req.flags & ROBOT_RPC_FLAG_SAVE) == 0U) {
        app_rpc_send_param_resp(req.method, ROBOT_RPC_STATUS_BAD_LEN, offset,
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

    robot_params_t updated;
    memcpy(&updated, &g_robot_params, sizeof(updated));
    memcpy((uint8_t *)&updated + offset, req_data, length);

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
