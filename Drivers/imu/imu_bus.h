#ifndef IMU_BUS_H
#define IMU_BUS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void imu_bus_set_ready(uint8_t ready);
uint8_t imu_bus_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_BUS_H */
