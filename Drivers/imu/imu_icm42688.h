#ifndef IMU_ICM42688_H
#define IMU_ICM42688_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t accel[3];
    int16_t gyro[3];
    int16_t temperature;
    uint32_t timestamp_ms;
} imu_icm42688_sample_t;

bool imu_icm42688_init(void);
void imu_icm42688_poll(void);
void imu_icm42688_handle_int1(void);
bool imu_icm42688_try_get_latest(imu_icm42688_sample_t *out, uint32_t *seq);
void imu_icm42688_kick(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_ICM42688_H */
