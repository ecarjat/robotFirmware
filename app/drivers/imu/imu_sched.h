#ifndef IMU_SCHED_H
#define IMU_SCHED_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    IMU_SCHED_SENSOR_BMI270 = 0,
    IMU_SCHED_SENSOR_ICM42688 = 1,
    IMU_SCHED_SENSOR_BMM150 = 2,
    IMU_SCHED_SENSOR_COUNT
} imu_sched_sensor_t;

void imu_sched_init(void);
void imu_sched_request(imu_sched_sensor_t sensor);
void imu_sched_run(void);
void imu_sched_tick(void);
void imu_sched_on_dma_done(imu_sched_sensor_t sensor, int status);
void imu_sched_set_min_interval(imu_sched_sensor_t sensor, uint32_t min_interval_ms);
uint32_t imu_sched_get_min_interval(imu_sched_sensor_t sensor);

#endif /* IMU_SCHED_H */
