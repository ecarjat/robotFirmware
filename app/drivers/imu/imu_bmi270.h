#ifndef IMU_BMI270_H
#define IMU_BMI270_H

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
} imu_bmi270_sample_t;

bool imu_bmi270_init(void);
void imu_bmi270_poll(void);
void imu_bmi270_handle_int1(void);
bool imu_bmi270_try_get_latest(imu_bmi270_sample_t *out, uint32_t *seq);
bool imu_bmi270_kick(void);
uint32_t imu_bmi270_get_last_irq_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_BMI270_H */
