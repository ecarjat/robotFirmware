#include "app_imu.h"
#include "math.h"
#include "robot_protocol.h"
#include "sensors.h"
#if SENSOR_ENABLE_BMI270
#include "imu_bmi270.h"
#endif
#if SENSOR_ENABLE_ICM42688
#include "imu_icm42688.h"
#endif
#if SENSOR_ENABLE_BMM150
#include "imu_bmm150.h"
#endif
#include "app_config.h"
#include "app_main.h"
#include "imu_bus.h"
#include "imu_sched.h"
#include "motion_control.h"
#include "param_storage.h"
#include "string.h"

#define APP_IMU_CALIB_FACE_COUNT 6U
#define APP_IMU_CALIB_IMU_COUNT 2U
#define APP_IMU_CALIB_DEFAULT_SAMPLES 800U
#define APP_IMU_CALIB_MAX_SAMPLES 800U
#define APP_IMU_CALIB_TIMEOUT_MS 3000U
#define APP_IMU_ACCEL_RANGE_G 4.0f
#define APP_IMU_GYRO_RANGE_DPS 500.0f

typedef struct {
  uint8_t valid_mask;
  float accel[APP_IMU_CALIB_FACE_COUNT][3];
  float gyro[APP_IMU_CALIB_FACE_COUNT][3];
} app_imu_calib_state_t;

static app_imu_calib_state_t s_imu_calib[APP_IMU_CALIB_IMU_COUNT];

static float app_vec_norm(const float v[3]) {
  return sqrtf((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]));
}

static float app_vec_dot(const float a[3], const float b[3]) {
  return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
}

static void app_vec_cross(const float a[3], const float b[3], float out[3]) {
  out[0] = (a[1] * b[2]) - (a[2] * b[1]);
  out[1] = (a[2] * b[0]) - (a[0] * b[2]);
  out[2] = (a[0] * b[1]) - (a[1] * b[0]);
}

static bool app_vec_normalize(float v[3]) {
  float norm = app_vec_norm(v);
  if (norm < 1e-6f) {
    return false;
  }
  float inv = 1.0f / norm;
  v[0] *= inv;
  v[1] *= inv;
  v[2] *= inv;
  return true;
}

void app_imu_init() {
  imu_sched_init();

  bool sensors_ok = true;
#if SENSOR_ENABLE_BMI270

  bool bmi_ok = imu_bmi270_init();
  if (!bmi_ok) {
    APP_LOG_ERROR("BMI270 init failed");
  }
  sensors_ok &= bmi_ok;
#endif

#if SENSOR_ENABLE_ICM42688

  bool icm_ok = imu_icm42688_init();

  if (!icm_ok) {
    APP_LOG_ERROR("ICM42688 init failed");
  }
  sensors_ok &= icm_ok;
#endif

#if SENSOR_ENABLE_BMM150

  bool bmm_ok = imu_bmm150_init();

  if (!bmm_ok) {
    APP_LOG_ERROR("BMM150 init failed");
  }
  sensors_ok &= bmm_ok;
#endif

  if (SENSOR_ENABLED_COUNT == 0) {
    APP_LOG_ERROR("No IMU sensors enabled");
  } else if (sensors_ok) {
    /*
     * EXTI Re-enable after IMU Init:
     * IMU EXTI interrupts were disabled in main.c to prevent race conditions
     * during cold boot. Now that sensors are initialized and the scheduler
     * is ready, clear any pending interrupts and re-enable EXTI.
     */
#if SENSOR_ENABLE_ICM42688
    __HAL_GPIO_EXTI_CLEAR_IT(ICM42688_INT1_Pin);
#endif
#if SENSOR_ENABLE_BMI270
    __HAL_GPIO_EXTI_CLEAR_IT(BMI270_INT1_Pin);
#endif
#if SENSOR_ENABLE_BMM150
    __HAL_GPIO_EXTI_CLEAR_IT(BMM150_INT1_Pin);
#endif

#if SENSOR_ENABLE_ICM42688
    HAL_NVIC_EnableIRQ(ICM42688_INT1_EXTI_IRQn);
#endif
#if SENSOR_ENABLE_BMI270
    HAL_NVIC_EnableIRQ(BMI270_INT1_EXTI_IRQn);
#endif
#if SENSOR_ENABLE_BMM150
    HAL_NVIC_EnableIRQ(BMM150_INT1_EXTI_IRQn);
#endif

    /*
     * Cold Boot Stabilization:
     * After enabling EXTI, give sensors time to generate their first
     * valid data-ready interrupt. On cold boot, sensors may need
     * additional settling time before DMA reads succeed reliably.
     */
    HAL_Delay(100);

    imu_bus_set_ready(1U);
#define SENSOR_REQUEST_ENTRY(name, short_name, prefix)                         \
  imu_sched_request(IMU_SCHED_SENSOR_##name);
    SENSOR_LIST_ENABLED(SENSOR_REQUEST_ENTRY)
#undef SENSOR_REQUEST_ENTRY
    imu_sched_run();
  } else {
    APP_LOG_ERROR("IMU bus not ready; one or more inits failed");
  }
}

static bool app_imu_calib_build_rotation(const app_imu_calib_state_t *state,
                                         float rot[9]) {
  if (state == NULL || rot == NULL) {
    return false;
  }
  if (state->valid_mask != ((1U << APP_IMU_CALIB_FACE_COUNT) - 1U)) {
    return false;
  }

  float x_vec[3];
  float y_vec[3];
  float z_vec[3];
  for (size_t i = 0U; i < 3U; ++i) {
    x_vec[i] = state->accel[ROBOT_IMU_FACE_X_POS_UP][i] -
               state->accel[ROBOT_IMU_FACE_X_NEG_UP][i];
    y_vec[i] = state->accel[ROBOT_IMU_FACE_Y_POS_UP][i] -
               state->accel[ROBOT_IMU_FACE_Y_NEG_UP][i];
    z_vec[i] = state->accel[ROBOT_IMU_FACE_Z_POS_UP][i] -
               state->accel[ROBOT_IMU_FACE_Z_NEG_UP][i];
  }

  if (!app_vec_normalize(x_vec) || !app_vec_normalize(y_vec) ||
      !app_vec_normalize(z_vec)) {
    return false;
  }

  float r0[3] = {x_vec[0], x_vec[1], x_vec[2]};
  float r1[3] = {y_vec[0], y_vec[1], y_vec[2]};

  float proj = app_vec_dot(r1, r0);
  for (size_t i = 0U; i < 3U; ++i) {
    r1[i] -= proj * r0[i];
  }
  if (!app_vec_normalize(r1)) {
    return false;
  }

  float r2[3];
  app_vec_cross(r0, r1, r2);
  if (app_vec_dot(r2, z_vec) < 0.0f) {
    for (size_t i = 0U; i < 3U; ++i) {
      r1[i] = -r1[i];
      r2[i] = -r2[i];
    }
  }

  rot[0] = r0[0];
  rot[1] = r0[1];
  rot[2] = r0[2];
  rot[3] = r1[0];
  rot[4] = r1[1];
  rot[5] = r1[2];
  rot[6] = r2[0];
  rot[7] = r2[1];
  rot[8] = r2[2];
  return true;
}

static bool app_imu_calib_capture_bmi270(float accel[3], float gyro[3],
                                         uint16_t samples) {
#if SENSOR_ENABLE_BMI270
  uint16_t target = samples;
  if (target == 0U) {
    target = APP_IMU_CALIB_DEFAULT_SAMPLES;
  }
  if (target > APP_IMU_CALIB_MAX_SAMPLES) {
    target = APP_IMU_CALIB_MAX_SAMPLES;
  }

  uint32_t seq = 0U;
  uint32_t count = 0U;
  float sum_accel[3] = {0.0f, 0.0f, 0.0f};
  float sum_gyro[3] = {0.0f, 0.0f, 0.0f};
  const uint32_t start = HAL_GetTick();

  while (count < target && (HAL_GetTick() - start) < APP_IMU_CALIB_TIMEOUT_MS) {
    imu_bmi270_sample_t sample;
    if (imu_bmi270_try_get_latest(&sample, &seq)) {
      sum_accel[0] += (float)sample.accel[0];
      sum_accel[1] += (float)sample.accel[1];
      sum_accel[2] += (float)sample.accel[2];
      sum_gyro[0] += (float)sample.gyro[0];
      sum_gyro[1] += (float)sample.gyro[1];
      sum_gyro[2] += (float)sample.gyro[2];
      ++count;
    }
    imu_sched_run();
    HAL_Delay(1U);
  }

  if (count == 0U) {
    return false;
  }

  const float accel_scale = (APP_IMU_ACCEL_RANGE_G * 9.80665f) / 32768.0f;
  const float gyro_scale = APP_IMU_GYRO_RANGE_DPS / 32768.0f;
  const float inv = 1.0f / (float)count;
  accel[0] = sum_accel[0] * inv * accel_scale;
  accel[1] = sum_accel[1] * inv * accel_scale;
  accel[2] = sum_accel[2] * inv * accel_scale;
  gyro[0] = sum_gyro[0] * inv * gyro_scale;
  gyro[1] = sum_gyro[1] * inv * gyro_scale;
  gyro[2] = sum_gyro[2] * inv * gyro_scale;
  return true;
#else
  (void)accel;
  (void)gyro;
  (void)samples;
  return false;
#endif
}

static bool app_imu_calib_capture_icm42688(float accel[3], float gyro[3],
                                           uint16_t samples) {
#if SENSOR_ENABLE_ICM42688
  uint16_t target = samples;
  if (target == 0U) {
    target = APP_IMU_CALIB_DEFAULT_SAMPLES;
  }
  if (target > APP_IMU_CALIB_MAX_SAMPLES) {
    target = APP_IMU_CALIB_MAX_SAMPLES;
  }

  uint32_t seq = 0U;
  uint32_t count = 0U;
  float sum_accel[3] = {0.0f, 0.0f, 0.0f};
  float sum_gyro[3] = {0.0f, 0.0f, 0.0f};
  const uint32_t start = HAL_GetTick();

  while (count < target && (HAL_GetTick() - start) < APP_IMU_CALIB_TIMEOUT_MS) {
    imu_icm42688_sample_t sample;
    if (imu_icm42688_try_get_latest(&sample, &seq)) {
      sum_accel[0] += (float)sample.accel[0];
      sum_accel[1] += (float)sample.accel[1];
      sum_accel[2] += (float)sample.accel[2];
      sum_gyro[0] += (float)sample.gyro[0];
      sum_gyro[1] += (float)sample.gyro[1];
      sum_gyro[2] += (float)sample.gyro[2];
      ++count;
    }
    imu_sched_run();
    HAL_Delay(1U);
  }

  if (count == 0U) {
    return false;
  }

  const float accel_scale = (APP_IMU_ACCEL_RANGE_G * 9.80665f) / 32768.0f;
  const float gyro_scale = APP_IMU_GYRO_RANGE_DPS / 32768.0f;
  const float inv = 1.0f / (float)count;
  accel[0] = sum_accel[0] * inv * accel_scale;
  accel[1] = sum_accel[1] * inv * accel_scale;
  accel[2] = sum_accel[2] * inv * accel_scale;
  gyro[0] = sum_gyro[0] * inv * gyro_scale;
  gyro[1] = sum_gyro[1] * inv * gyro_scale;
  gyro[2] = sum_gyro[2] * inv * gyro_scale;
  return true;
#else
  (void)accel;
  (void)gyro;
  (void)samples;
  return false;
#endif
}

static int16_t app_clamp_i16(int32_t value) {
  if (value > INT16_MAX) {
    return INT16_MAX;
  }
  if (value < INT16_MIN) {
    return INT16_MIN;
  }
  return (int16_t)value;
}

static uint8_t app_imu_calib_apply(uint8_t imu_id,
                                   const app_imu_calib_state_t *state) {
  if (state == NULL) {
    return ROBOT_RPC_STATUS_BAD_PARAM;
  }

  float accel_bias[3] = {0.0f, 0.0f, 0.0f};
  float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
  for (size_t face = 0U; face < APP_IMU_CALIB_FACE_COUNT; ++face) {
    accel_bias[0] += state->accel[face][0];
    accel_bias[1] += state->accel[face][1];
    accel_bias[2] += state->accel[face][2];
    gyro_bias[0] += state->gyro[face][0];
    gyro_bias[1] += state->gyro[face][1];
    gyro_bias[2] += state->gyro[face][2];
  }

  const float inv_faces = 1.0f / (float)APP_IMU_CALIB_FACE_COUNT;
  accel_bias[0] *= inv_faces;
  accel_bias[1] *= inv_faces;
  accel_bias[2] *= inv_faces;
  gyro_bias[0] *= inv_faces;
  gyro_bias[1] *= inv_faces;
  gyro_bias[2] *= inv_faces;

  const float inv_g = 1.0f / 9.80665f;
  int32_t accel_bias_mg[3] = {
      (int32_t)lrintf(accel_bias[0] * inv_g * 1000.0f),
      (int32_t)lrintf(accel_bias[1] * inv_g * 1000.0f),
      (int32_t)lrintf(accel_bias[2] * inv_g * 1000.0f),
  };
  int32_t gyro_bias_mdps[3] = {
      (int32_t)lrintf(gyro_bias[0] * 1000.0f),
      (int32_t)lrintf(gyro_bias[1] * 1000.0f),
      (int32_t)lrintf(gyro_bias[2] * 1000.0f),
  };

  float rotation[9];
  if (!app_imu_calib_build_rotation(state, rotation)) {
    return ROBOT_RPC_STATUS_BAD_PARAM;
  }

  imu_calib_t *calib = NULL;
  if (imu_id == 0U) {
    calib = &g_robot_params.imu_bmi270;
  } else if (imu_id == 1U) {
    calib = &g_robot_params.imu_icm42688;
  } else {
    return ROBOT_RPC_STATUS_BAD_PARAM;
  }

  calib->accel_bias[0] = app_clamp_i16(accel_bias_mg[0]);
  calib->accel_bias[1] = app_clamp_i16(accel_bias_mg[1]);
  calib->accel_bias[2] = app_clamp_i16(accel_bias_mg[2]);
  calib->gyro_bias[0] = app_clamp_i16(gyro_bias_mdps[0]);
  calib->gyro_bias[1] = app_clamp_i16(gyro_bias_mdps[1]);
  calib->gyro_bias[2] = app_clamp_i16(gyro_bias_mdps[2]);
  memcpy(calib->rotation, rotation, sizeof(rotation));

  motion_control_apply_params();
  return ROBOT_RPC_STATUS_OK;
}

uint8_t app_imu_calib_capture_face(uint8_t imu_id, uint8_t face,
                                   uint16_t samples) {
  if (face >= ROBOT_IMU_FACE_COUNT) {
    return ROBOT_RPC_STATUS_BAD_PARAM;
  }
  if (imu_id >= APP_IMU_CALIB_IMU_COUNT) {
    return ROBOT_RPC_STATUS_BAD_PARAM;
  }

  float accel[3];
  float gyro[3];
  bool ok = false;
  if (imu_id == 0U) {
#if SENSOR_ENABLE_BMI270
    ok = app_imu_calib_capture_bmi270(accel, gyro, samples);
#else
    return ROBOT_RPC_STATUS_NOT_READY;
#endif
  } else {
#if SENSOR_ENABLE_ICM42688
    ok = app_imu_calib_capture_icm42688(accel, gyro, samples);
#else
    return ROBOT_RPC_STATUS_NOT_READY;
#endif
  }

  if (!ok) {
    return ROBOT_RPC_STATUS_TIMEOUT;
  }

  app_imu_calib_state_t *state = &s_imu_calib[imu_id];
  for (size_t i = 0U; i < 3U; ++i) {
    state->accel[face][i] = accel[i];
    state->gyro[face][i] = gyro[i];
  }
  state->valid_mask |= (uint8_t)(1U << face);

  if (state->valid_mask != ((1U << APP_IMU_CALIB_FACE_COUNT) - 1U)) {
    return ROBOT_RPC_STATUS_INCOMPLETE;
  }

  return app_imu_calib_apply(imu_id, state);
}