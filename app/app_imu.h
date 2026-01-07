#ifndef APP_IMU_H
#define APP_IMU_H

#include "param_storage.h"
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t app_imu_calib_capture_face(uint8_t imu_id, uint8_t face,
                                   uint16_t samples);
void app_imu_init();

#ifdef __cplusplus
}
#endif

#endif /* APP_IMU_H */