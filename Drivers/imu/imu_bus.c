#include "imu_bus.h"

static volatile uint8_t s_imu_bus_ready = 0U;

void imu_bus_set_ready(uint8_t ready)
{
    s_imu_bus_ready = ready ? 1U : 0U;
}

uint8_t imu_bus_is_ready(void)
{
    return s_imu_bus_ready;
}
