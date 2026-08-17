#include "motor_link.h"

#include <math.h>
#include <string.h>

#include "app_config.h"
#include "app_log_macros.h"
#include "hip_control.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"

#ifndef MOTOR_LINK_TELEM_WARN_MS
#define MOTOR_LINK_TELEM_WARN_MS 15U
#endif

#ifndef MOTOR_LINK_TELEM_STALE_MS
#define MOTOR_LINK_TELEM_STALE_MS 30U
#endif

#ifndef MOTOR_LINK_KT_NM_PER_A
#define MOTOR_LINK_KT_NM_PER_A 0.0f
#endif

#ifndef MOTOR_LINK_KV_RPM_PER_V
#define MOTOR_LINK_KV_RPM_PER_V 0.0f
#endif

#ifndef MOTOR_LINK_TARGET_PERIOD_MS
#define MOTOR_LINK_TARGET_PERIOD_MS 4U
#endif

#ifndef MOTOR_LINK_FLOAT_EPS
#define MOTOR_LINK_FLOAT_EPS 1.0e-4f
#endif

#ifndef MOTOR_LINK_TARGET_EPS
#define MOTOR_LINK_TARGET_EPS 1.0e-3f
#endif

#define MOTOR_LINK_PI 3.14159265358979323846f

/* GDS68 CAN Simple command IDs. Arbitration ID = (node_id << 5) | command. */
#define MOTOR_LINK_CMD_SET_AXIS_STATE 0x007U
#define MOTOR_LINK_CMD_GET_ENCODER_ESTIMATES 0x009U
#define MOTOR_LINK_CMD_SET_CONTROLLER_MODE 0x00BU
#define MOTOR_LINK_CMD_SET_INPUT_TORQUE 0x00EU
#define MOTOR_LINK_CMD_CLEAR_ERRORS 0x018U

#define MOTOR_LINK_AXIS_STATE_IDLE 1U
#define MOTOR_LINK_AXIS_STATE_CLOSED_LOOP 8U
#define MOTOR_LINK_CTRL_MODE_TORQUE 1U
#define MOTOR_LINK_CTRL_MODE_VELOCITY 2U
#define MOTOR_LINK_CTRL_MODE_POSITION 3U
#define MOTOR_LINK_INPUT_MODE_PASSTHROUGH 1U
#define MOTOR_LINK_INPUT_MODE_VEL_RAMP 2U
#define MOTOR_LINK_INPUT_MODE_POS_FILTER 3U

#define MOTOR_LINK_RX_RING_SIZE 32U

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
  uint32_t t_ms;
} motor_link_can_frame_t;

typedef struct {
  float velocity;
  uint32_t last_vel_ms;
  uint32_t stale_count;
  uint32_t late_count;
  uint32_t ack_timeouts;
  uint8_t vel_valid;
} motor_telem_t;

extern FDCAN_HandleTypeDef hfdcan1;

static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;
static motor_link_can_frame_t s_rx_ring[MOTOR_LINK_RX_RING_SIZE];
static uint32_t s_rx_overflow = 0U;

static volatile uint8_t s_initialized = 0U;
static volatile uint8_t s_enable_applied = 0U;
static volatile uint8_t s_bus_off = 0U;
static volatile uint8_t s_bus_off_logged = 0U;

static motor_control_mode_t s_control_mode = MOTOR_CONTROL_TORQUE;
static int8_t s_left_dir = 1;
static int8_t s_right_dir = 1;
static float s_left_cmd = 0.0f;
static float s_right_cmd = 0.0f;
static float s_left_sent = 0.0f;
static float s_right_sent = 0.0f;
static uint8_t s_left_sent_valid = 0U;
static uint8_t s_right_sent_valid = 0U;
static uint32_t s_last_target_ms = 0U;
static motor_telem_t s_left_telem;
static motor_telem_t s_right_telem;

static uint32_t s_left_crc_errors = 0U;
static uint32_t s_right_crc_errors = 0U;
static uint32_t s_left_sync_losses = 0U;
static uint32_t s_right_sync_losses = 0U;

static uint32_t s_left_ack_timeouts = 0U;
static uint32_t s_right_ack_timeouts = 0U;

static void motor_link_apply_enable_gpio(bool on) {
#if defined(MOTORS_ENABLE_GPIO_Port) && defined(MOTORS_ENABLE_Pin)
  HAL_GPIO_WritePin(MOTORS_ENABLE_GPIO_Port, MOTORS_ENABLE_Pin,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
  (void)on;
#endif
}

static uint32_t motor_link_len_to_dlc(uint8_t len) {
  switch (len) {
  case 0:
    return FDCAN_DLC_BYTES_0;
  case 1:
    return FDCAN_DLC_BYTES_1;
  case 2:
    return FDCAN_DLC_BYTES_2;
  case 3:
    return FDCAN_DLC_BYTES_3;
  case 4:
    return FDCAN_DLC_BYTES_4;
  case 5:
    return FDCAN_DLC_BYTES_5;
  case 6:
    return FDCAN_DLC_BYTES_6;
  case 7:
    return FDCAN_DLC_BYTES_7;
  case 8:
    return FDCAN_DLC_BYTES_8;
  default:
    return FDCAN_DLC_BYTES_8;
  }
}

static uint8_t motor_link_dlc_to_len(uint32_t dlc) {
  switch (dlc) {
  case FDCAN_DLC_BYTES_0:
    return 0U;
  case FDCAN_DLC_BYTES_1:
    return 1U;
  case FDCAN_DLC_BYTES_2:
    return 2U;
  case FDCAN_DLC_BYTES_3:
    return 3U;
  case FDCAN_DLC_BYTES_4:
    return 4U;
  case FDCAN_DLC_BYTES_5:
    return 5U;
  case FDCAN_DLC_BYTES_6:
    return 6U;
  case FDCAN_DLC_BYTES_7:
    return 7U;
  case FDCAN_DLC_BYTES_8:
    return 8U;
  default:
    return 8U;
  }
}

static bool motor_link_rx_push(uint32_t id, uint8_t dlc, const uint8_t *data,
                               uint32_t t_ms) {
  uint16_t next = (uint16_t)((s_rx_head + 1U) % MOTOR_LINK_RX_RING_SIZE);
  if (next == s_rx_tail) {
    s_rx_overflow++;
    return false;
  }
  s_rx_ring[s_rx_head].id = id;
  s_rx_ring[s_rx_head].dlc = dlc;
  s_rx_ring[s_rx_head].t_ms = t_ms;
  if (data != NULL && dlc > 0U) {
    uint8_t copy_len = (dlc > 8U) ? 8U : dlc;
    memcpy(s_rx_ring[s_rx_head].data, data, copy_len);
    if (copy_len < 8U) {
      memset(&s_rx_ring[s_rx_head].data[copy_len], 0, (size_t)(8U - copy_len));
    }
  } else {
    memset(s_rx_ring[s_rx_head].data, 0, sizeof(s_rx_ring[s_rx_head].data));
  }
  s_rx_head = next;
  return true;
}

static bool motor_link_rx_pop(motor_link_can_frame_t *out) {
  if (out == NULL) {
    return false;
  }
  if (s_rx_tail == s_rx_head) {
    return false;
  }
  *out = s_rx_ring[s_rx_tail];
  s_rx_tail = (uint16_t)((s_rx_tail + 1U) % MOTOR_LINK_RX_RING_SIZE);
  return true;
}

static float motor_link_clamp(float v, float limit) {
  if (limit <= 0.0f) {
    return 0.0f;
  }
  if (v > limit) {
    return limit;
  }
  if (v < -limit) {
    return -limit;
  }
  return v;
}

static float motor_link_resolve_kt(void) {
  if (MOTOR_LINK_KT_NM_PER_A > 0.0f) {
    return MOTOR_LINK_KT_NM_PER_A;
  }
  if (MOTOR_LINK_KV_RPM_PER_V > 0.0f) {
    return 60.0f / (2.0f * MOTOR_LINK_PI * MOTOR_LINK_KV_RPM_PER_V);
  }
  return 0.0f;
}

static bool motor_link_can_send(uint16_t std_id, const uint8_t *data,
                                uint8_t len) {
  if (!s_initialized || s_bus_off) {
    return false;
  }

  if (len > 8U) {
    return false;
  }

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) {
    return false;
  }

  FDCAN_TxHeaderTypeDef tx = {0};
  tx.Identifier = std_id & 0x7FFU;
  tx.IdType = FDCAN_STANDARD_ID;
  tx.TxFrameType = FDCAN_DATA_FRAME;
  tx.DataLength = motor_link_len_to_dlc(len);
  tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx.BitRateSwitch = FDCAN_BRS_OFF;
  tx.FDFormat = FDCAN_CLASSIC_CAN;
  tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx.MessageMarker = 0U;

  uint8_t payload[8] = {0U};
  if (data != NULL && len > 0U) {
    memcpy(payload, data, len);
  }

  return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, payload) == HAL_OK);
}

static uint16_t motor_link_can_simple_id(uint8_t node_id, uint8_t cmd) {
  return (uint16_t)(((uint16_t)(node_id & 0x3FU) << 5) | (cmd & 0x1FU));
}

static bool motor_link_can_send_command(uint8_t node_id, uint8_t cmd,
                                        const uint8_t *payload, uint8_t len) {
  return motor_link_can_send(motor_link_can_simple_id(node_id, cmd), payload,
                             len);
}

static void motor_link_pack_u32(uint8_t *dst, uint32_t value) {
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8) & 0xFFU);
  dst[2] = (uint8_t)((value >> 16) & 0xFFU);
  dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static bool motor_link_set_axis_state(uint8_t node_id, uint32_t state) {
  uint8_t payload[4] = {0U};
  motor_link_pack_u32(payload, state);
  return motor_link_can_send_command(node_id, MOTOR_LINK_CMD_SET_AXIS_STATE,
                                     payload, sizeof(payload));
}

static bool motor_link_set_controller_mode(uint8_t node_id,
                                           motor_control_mode_t mode) {
  uint32_t control_mode = 0U;
  uint32_t input_mode = 0U;
  switch (mode) {
  case MOTOR_CONTROL_TORQUE:
    control_mode = MOTOR_LINK_CTRL_MODE_TORQUE;
    input_mode = MOTOR_LINK_INPUT_MODE_PASSTHROUGH;
    break;
  case MOTOR_CONTROL_VELOCITY:
    control_mode = MOTOR_LINK_CTRL_MODE_VELOCITY;
    input_mode = MOTOR_LINK_INPUT_MODE_VEL_RAMP;
    break;
  case MOTOR_CONTROL_ANGLE:
    control_mode = MOTOR_LINK_CTRL_MODE_POSITION;
    input_mode = MOTOR_LINK_INPUT_MODE_POS_FILTER;
    break;
  default:
    return false;
  }

  uint8_t payload[8] = {0U};
  motor_link_pack_u32(&payload[0], control_mode);
  motor_link_pack_u32(&payload[4], input_mode);
  return motor_link_can_send_command(node_id, MOTOR_LINK_CMD_SET_CONTROLLER_MODE,
                                     payload, sizeof(payload));
}

static bool motor_link_send_torque(uint8_t node_id, float torque_nm) {
  uint8_t payload[4] = {0U};
  memcpy(payload, &torque_nm, sizeof(torque_nm));
  return motor_link_can_send_command(node_id, MOTOR_LINK_CMD_SET_INPUT_TORQUE,
                                     payload, sizeof(payload));
}

static bool motor_link_send_zero_torque(uint8_t node_id) {
  return motor_link_send_torque(node_id, 0.0f);
}

static void motor_link_update_velocity(motor_telem_t *telem, float vel_rad_s,
                                       uint32_t t_ms) {
  if (telem == NULL) {
    return;
  }
  telem->velocity = vel_rad_s;
  telem->last_vel_ms = t_ms;
  telem->vel_valid = 1U;
}

static void motor_link_handle_encoder_estimate(motor_telem_t *telem,
                                               const uint8_t *data, uint8_t len,
                                               uint32_t t_ms) {
  if (telem == NULL || data == NULL || len < 8U) {
    return;
  }
  float velocity_rev_s = 0.0f;
  memcpy(&velocity_rev_s, &data[4], sizeof(velocity_rev_s));
  if (!isfinite(velocity_rev_s)) {
    return;
  }
  float rad_s = velocity_rev_s * (2.0f * MOTOR_LINK_PI);
  motor_link_update_velocity(telem, rad_s, t_ms);
}

static void motor_link_process_frame(const motor_link_can_frame_t *frame) {
  if (frame == NULL) {
    return;
  }

  uint8_t node_id = (uint8_t)((frame->id >> 5) & 0x3FU);
  motor_telem_t *telem = NULL;
  if (node_id == MOTOR_LINK_CAN_LEFT_ID) {
    telem = &s_left_telem;
  } else if (node_id == MOTOR_LINK_CAN_RIGHT_ID) {
    telem = &s_right_telem;
  } else {
    return;
  }

  uint8_t cmd = (uint8_t)(frame->id & 0x1FU);
  switch (cmd) {
  case MOTOR_LINK_CMD_GET_ENCODER_ESTIMATES:
    motor_link_handle_encoder_estimate(telem, frame->data, frame->dlc,
                                        frame->t_ms);
    break;
  default:
    break;
  }
}

static void motor_link_process_rx(void) {
  motor_link_can_frame_t frame;
  while (motor_link_rx_pop(&frame)) {
    motor_link_process_frame(&frame);
  }
}

static bool motor_link_config_filters(void) {
  FDCAN_FilterTypeDef filter = {0};

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID2 = 0x7E0U; /* Match all CAN Simple commands for one node. */

  filter.FilterIndex = 0U;
  filter.FilterID1 = ((uint32_t)MOTOR_LINK_CAN_LEFT_ID << 5);
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
    return false;
  }

  filter.FilterIndex = 1U;
  filter.FilterID1 = ((uint32_t)MOTOR_LINK_CAN_RIGHT_ID << 5);
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
    return false;
  }

  filter.FilterIndex = 2U;
  filter.FilterID1 = (HIP_CAN_NODE_LEFT << 5);
  filter.FilterID2 = 0x7E0U;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
    return false;
  }

  filter.FilterIndex = 3U;
  filter.FilterID1 = (HIP_CAN_NODE_RIGHT << 5);
  filter.FilterID2 = 0x7E0U;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
    return false;
  }

  (void)HAL_FDCAN_ConfigGlobalFilter(
      &hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE,
      FDCAN_REJECT_REMOTE);

  return true;
}

static bool motor_link_config_notifications(void) {
  uint32_t it_mask = FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF;

  if (HAL_FDCAN_ConfigInterruptLines(&hfdcan1, it_mask,
                                     FDCAN_INTERRUPT_LINE0) != HAL_OK) {
    return false;
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, it_mask, 0U) != HAL_OK) {
    return false;
  }

  return true;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                              uint32_t RxFifo0ITs) {
  if (hfdcan == NULL || hfdcan->Instance != FDCAN1) {
    return;
  }
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
    FDCAN_RxHeaderTypeDef rx = {0};
    uint8_t data[8];
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) {
      break;
    }
    if (rx.IdType != FDCAN_STANDARD_ID) {
      continue;
    }
    uint8_t len = motor_link_dlc_to_len(rx.DataLength);
    if (hip_control_on_can_rx(rx.Identifier, data, len, HAL_GetTick())) {
      continue;
    }
    (void)motor_link_rx_push(rx.Identifier, len, data, HAL_GetTick());
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs) {
  if (hfdcan == NULL || hfdcan->Instance != FDCAN1) {
    return;
  }

  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U) {
    s_bus_off = 1U;
    s_enable_applied = 0U;
    motor_link_apply_enable_gpio(false);
    hip_control_on_bus_off();
  }
}

bool motor_link_init(void) {
  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_rx_overflow = 0U;

  s_left_telem = (motor_telem_t){0};
  s_right_telem = (motor_telem_t){0};

  s_left_crc_errors = 0U;
  s_right_crc_errors = 0U;
  s_left_sync_losses = 0U;
  s_right_sync_losses = 0U;

  s_left_ack_timeouts = 0U;
  s_right_ack_timeouts = 0U;

  s_bus_off = 0U;
  s_bus_off_logged = 0U;
  s_control_mode = MOTOR_CONTROL_TORQUE;
  s_left_dir = 1;
  s_right_dir = 1;
  s_left_cmd = 0.0f;
  s_right_cmd = 0.0f;
  s_left_sent = 0.0f;
  s_right_sent = 0.0f;
  s_left_sent_valid = 0U;
  s_right_sent_valid = 0U;
  s_last_target_ms = 0U;

  if (!motor_link_config_filters()) {
    return false;
  }

  if (!motor_link_config_notifications()) {
    return false;
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    return false;
  }

  s_initialized = 1U;
  APP_LOG_INFO("Motor link initialized (GDS68 CAN Simple)");
  return true;
}

void motor_link_poll(void) {
  if (!s_initialized) {
    return;
  }

  if (s_bus_off && !s_bus_off_logged) {
    APP_LOG_ERROR("Motor link bus-off detected");
    s_bus_off_logged = 1U;
  }

  motor_link_process_rx();

  if (s_bus_off) {
    return;
  }

  /* Encoder estimates are periodic CAN Simple messages configured in the
   * drive. There is no host request frame for command 0x009. */
}

bool motor_link_enable(bool on) {
  if (!s_initialized) {
    return false;
  }

  if (s_bus_off) {
    return false;
  }

  bool was_enabled = (s_enable_applied != 0U);
  bool ok = true;

  if (on) {
    motor_link_apply_enable_gpio(true);
    const uint8_t node_ids[2] = {MOTOR_LINK_CAN_LEFT_ID,
                                 MOTOR_LINK_CAN_RIGHT_ID};
    for (uint32_t i = 0U; i < 2U; ++i) {
      uint8_t node_id = node_ids[i];
      ok = motor_link_can_send_command(node_id, MOTOR_LINK_CMD_CLEAR_ERRORS,
                                       NULL, 0U) &&
           ok;
      ok = motor_link_set_controller_mode(node_id, s_control_mode) && ok;
      ok = motor_link_send_zero_torque(node_id) && ok;
      ok = motor_link_set_axis_state(node_id, MOTOR_LINK_AXIS_STATE_CLOSED_LOOP) &&
           ok;
    }
    s_enable_applied = ok ? 1U : 0U;
    if (!ok) {
      motor_link_apply_enable_gpio(false);
    }
  } else {
    if (was_enabled) {
      ok = motor_link_send_zero_torque(MOTOR_LINK_CAN_LEFT_ID) && ok;
      ok = motor_link_send_zero_torque(MOTOR_LINK_CAN_RIGHT_ID) && ok;
    }
    ok = motor_link_set_axis_state(MOTOR_LINK_CAN_LEFT_ID,
                                   MOTOR_LINK_AXIS_STATE_IDLE) &&
         ok;
    ok = motor_link_set_axis_state(MOTOR_LINK_CAN_RIGHT_ID,
                                   MOTOR_LINK_AXIS_STATE_IDLE) &&
         ok;
    s_left_cmd = 0.0f;
    s_right_cmd = 0.0f;
    s_enable_applied = 0U;
    s_left_sent_valid = 0U;
    s_right_sent_valid = 0U;
    motor_link_apply_enable_gpio(false);
  }

  return ok;
}

void motor_link_set_control_mode(motor_control_mode_t mode) {
  if (!s_initialized) {
    return;
  }

  switch (mode) {
  case MOTOR_CONTROL_TORQUE:
  case MOTOR_CONTROL_VELOCITY:
  case MOTOR_CONTROL_ANGLE:
    s_control_mode = mode;
    if (s_enable_applied) {
      (void)motor_link_set_controller_mode(MOTOR_LINK_CAN_LEFT_ID, mode);
      (void)motor_link_set_controller_mode(MOTOR_LINK_CAN_RIGHT_ID, mode);
      (void)motor_link_send_zero_torque(MOTOR_LINK_CAN_LEFT_ID);
      (void)motor_link_send_zero_torque(MOTOR_LINK_CAN_RIGHT_ID);
      s_left_sent_valid = 0U;
      s_right_sent_valid = 0U;
    }
    break;
  default:
    APP_LOG_WARN("Motor link: open-loop mode unsupported");
    break;
  }
}

motor_control_mode_t motor_link_get_control_mode(void) {
  return s_control_mode;
}

void motor_link_set_motor_directions(int8_t left_dir, int8_t right_dir) {
  s_left_dir = (left_dir < 0) ? -1 : 1;
  s_right_dir = (right_dir < 0) ? -1 : 1;
}

static void motor_link_set_wheel_torque_commands(float left_Nm, float right_Nm,
                                                 float max_Nm) {
  if (!s_initialized || s_control_mode != MOTOR_CONTROL_TORQUE) {
    return;
  }

  if (!isfinite(left_Nm) || !isfinite(right_Nm) || !isfinite(max_Nm)) {
    left_Nm = 0.0f;
    right_Nm = 0.0f;
    max_Nm = 0.0f;
  }

  float left = motor_link_clamp(left_Nm * s_left_dir, max_Nm);
  float right = motor_link_clamp(right_Nm * s_right_dir, max_Nm);

  s_left_cmd = left;
  s_right_cmd = right;

  if (!s_enable_applied || s_bus_off) {
    return;
  }

  uint32_t now_ms = HAL_GetTick();
  if ((now_ms - s_last_target_ms) < MOTOR_LINK_TARGET_PERIOD_MS) {
    return;
  }

  bool left_changed =
      !s_left_sent_valid || (fabsf(left - s_left_sent) > MOTOR_LINK_TARGET_EPS);
  bool right_changed = !s_right_sent_valid ||
                       (fabsf(right - s_right_sent) > MOTOR_LINK_TARGET_EPS);
  if (!left_changed && !right_changed) {
    return;
  }

  if (left_changed) {
    if (motor_link_send_torque(MOTOR_LINK_CAN_LEFT_ID, left)) {
      s_left_sent = left;
      s_left_sent_valid = 1U;
    } else {
      s_left_ack_timeouts++;
    }
  }

  if (right_changed) {
    if (motor_link_send_torque(MOTOR_LINK_CAN_RIGHT_ID, right)) {
      s_right_sent = right;
      s_right_sent_valid = 1U;
    } else {
      s_right_ack_timeouts++;
    }
  }

  s_last_target_ms = now_ms;
}

void motor_link_set_wheel_Iq(float left_A, float right_A, float max_A) {
  if (!isfinite(left_A) || !isfinite(right_A) || !isfinite(max_A)) {
    motor_link_set_wheel_torque_commands(0.0f, 0.0f, 0.0f);
    return;
  }

  float kt = motor_link_resolve_kt();
  if (kt <= 0.0f) {
    APP_LOG_WARN("Motor link: cannot convert Iq to torque without motor Kt");
    motor_link_set_wheel_torque_commands(0.0f, 0.0f, 0.0f);
    return;
  }

  motor_link_set_wheel_torque_commands(left_A * kt, right_A * kt,
                                       max_A * kt);
}

bool motor_backend_steadywin_can_torque_to_iq(float torque_nm, float *iq_out);

void motor_link_set_wheel_torques(float left_Nm, float right_Nm, float max_Nm) {
  if (!isfinite(left_Nm) || !isfinite(right_Nm) || !isfinite(max_Nm)) {
    motor_link_set_wheel_torque_commands(0.0f, 0.0f, 0.0f);
    return;
  }
  motor_link_set_wheel_torque_commands(left_Nm, right_Nm, max_Nm);
}

bool motor_backend_steadywin_can_torque_to_iq(float torque_nm, float *iq_out) {
  if (iq_out == NULL || !isfinite(torque_nm)) {
    return false;
  }

  float kt = motor_link_resolve_kt();
  if (kt <= 0.0f) {
    return false;
  }

  *iq_out = torque_nm / kt;
  return true;
}

bool motor_link_get_wheel_velocities(float *left_rad_s, float *right_rad_s) {
  if (left_rad_s == NULL || right_rad_s == NULL) {
    return false;
  }

  if (!s_initialized || s_bus_off) {
    *left_rad_s = 0.0f;
    *right_rad_s = 0.0f;
    return false;
  }

  uint32_t now = HAL_GetTick();
  uint32_t age_left =
      s_left_telem.last_vel_ms ? (now - s_left_telem.last_vel_ms) : UINT32_MAX;
  uint32_t age_right = s_right_telem.last_vel_ms
                           ? (now - s_right_telem.last_vel_ms)
                           : UINT32_MAX;

  bool left_stale =
      !s_left_telem.vel_valid || age_left > MOTOR_LINK_TELEM_STALE_MS;
  bool right_stale =
      !s_right_telem.vel_valid || age_right > MOTOR_LINK_TELEM_STALE_MS;
  bool left_late = !left_stale && age_left > MOTOR_LINK_TELEM_WARN_MS;
  bool right_late = !right_stale && age_right > MOTOR_LINK_TELEM_WARN_MS;

  *left_rad_s = left_stale ? 0.0f : s_left_telem.velocity;
  *right_rad_s = right_stale ? 0.0f : s_right_telem.velocity;

  if (left_stale) {
    s_left_telem.stale_count++;
  } else if (left_late) {
    s_left_telem.late_count++;
  }

  if (right_stale) {
    s_right_telem.stale_count++;
  } else if (right_late) {
    s_right_telem.late_count++;
  }

  return !left_stale && !right_stale;
}

bool motor_link_send_command(motor_side_t side, uint8_t cmd_id) {
  (void)side;
  (void)cmd_id;
  return false;
}

bool motor_link_send_write(motor_side_t side) {
  (void)side;
  return false;
}

bool motor_link_send_calibrate(motor_side_t side) {
  (void)side;
  return false;
}

bool motor_link_send_bootloader(motor_side_t side) {
  (void)side;
  return false;
}

motor_cmd_result_t motor_link_wait_command(motor_side_t side, uint8_t cmd_id,
                                           uint32_t timeout_ms) {
  (void)side;
  (void)cmd_id;
  (void)timeout_ms;
  return MOTOR_CMD_UNKNOWN;
}

uint32_t motor_link_get_left_parser_drops(void) { return s_rx_overflow; }

uint32_t motor_link_get_right_parser_drops(void) { return s_rx_overflow; }

uint32_t motor_link_get_left_telem_stale(void) {
  return s_left_telem.stale_count;
}

uint32_t motor_link_get_right_telem_stale(void) {
  return s_right_telem.stale_count;
}

uint32_t motor_link_get_left_telem_late(void) { return s_left_telem.late_count; }

uint32_t motor_link_get_right_telem_late(void) {
  return s_right_telem.late_count;
}

uint32_t motor_link_get_left_ack_timeouts(void) { return s_left_ack_timeouts; }

uint32_t motor_link_get_right_ack_timeouts(void) {
  return s_right_ack_timeouts;
}

uint32_t motor_link_get_left_sync_losses(void) { return s_left_sync_losses; }

uint32_t motor_link_get_right_sync_losses(void) { return s_right_sync_losses; }

uint32_t motor_link_get_left_crc_errors(void) { return s_left_crc_errors; }

uint32_t motor_link_get_right_crc_errors(void) { return s_right_crc_errors; }
