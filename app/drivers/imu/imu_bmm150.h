#ifndef IMU_BMM150_H
#define IMU_BMM150_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t mag[3];
    uint32_t timestamp_ms;
} imu_bmm150_sample_t;

bool imu_bmm150_init(void);
void imu_bmm150_poll(void);
void imu_bmm150_handle_int1(void);
bool imu_bmm150_try_get_latest(imu_bmm150_sample_t *out, uint32_t *seq);
bool imu_bmm150_kick(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_BMM150_H */
