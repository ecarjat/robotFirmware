#include "imu_bus.h"
#include "imu_sched.h"

void spi_bus_idle_hook(void)
{
    if (!imu_bus_is_ready())
    {
        return;
    }

    imu_sched_run();
}
